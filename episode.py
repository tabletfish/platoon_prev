#!/usr/bin/env python3.10

import argparse
import copy
import logging
import math
import time
from pathlib import Path

import carla

from ros2_native import (
    _make_transform,
    _set_vehicle_autopilot,
    destroy_actors,
    load_config_file,
    spawn_actors_from_config,
)


DEFAULT_SCENARIO_FILE = Path(__file__).with_name("scenario_town03.json")


def _distance_between(a, b):
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _get_spawn_point_by_index(map_, index):
    spawn_points = map_.get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No vehicle spawn points are available on this map.")

    if index < 0 or index >= len(spawn_points):
        raise IndexError(
            f"spawn_point_index={index} is out of range; available range is 0..{len(spawn_points) - 1}."
        )

    return spawn_points[index]


def _resolve_route_location(map_, point_config):
    if "location" in point_config:
        location = point_config["location"]
        return carla.Location(
            x=location["x"],
            y=location["y"],
            z=location.get("z", 0.0),
        )

    if "spawn_point" in point_config:
        return _make_transform(point_config["spawn_point"]).location

    if "spawn_point_index" in point_config:
        transform = _get_spawn_point_by_index(map_, int(point_config["spawn_point_index"]))
        return transform.location

    raise ValueError("Each route point must define location, spawn_point, or spawn_point_index.")


def _make_location(location_config):
    return carla.Location(
        x=location_config["x"],
        y=location_config["y"],
        z=location_config.get("z", 0.0),
    )


def _apply_scenario_to_stack(stack_config, scenario_config):
    config = copy.deepcopy(stack_config)
    objects = config.get("objects", [])
    if len(objects) != 1:
        raise ValueError("This episode runner currently expects exactly one ego vehicle in stack.json.")

    ego_config = objects[0]
    scenario_ego = scenario_config.get("ego", {})

    if "spawn_point" in scenario_ego:
        ego_config["spawn_point"] = scenario_ego["spawn_point"]
        ego_config.pop("spawn_point_index", None)
    elif "spawn_point_index" in scenario_ego:
        ego_config["spawn_point_index"] = int(scenario_ego["spawn_point_index"])
        ego_config.pop("spawn_point", None)

    ego_config["autopilot"] = bool(scenario_config.get("autopilot", ego_config.get("autopilot", False)))
    return config


def _setup_world(client, map_name, tm_port, allow_map_load, map_load_requested):
    world = client.get_world()
    current_map = world.get_map().name.rsplit("/", 1)[-1]
    if current_map != map_name:
        if not allow_map_load:
            raise RuntimeError(
                f'Current CARLA map is "{current_map}", but scenario requires "{map_name}". '
                f"Launch CARLA directly with {map_name} before running episode.py."
            )
        if not map_load_requested:
            logging.info("Loading map %s", map_name)
            client.load_world(map_name)
            return None, None, None, True
        raise RuntimeError(
            f'Waiting for CARLA to finish loading "{map_name}". Current map is still "{current_map}".'
        )

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    traffic_manager = client.get_trafficmanager(tm_port)
    traffic_manager.set_synchronous_mode(True)

    return world, original_settings, traffic_manager, map_load_requested


def _connect_world_with_retry(
    client,
    host,
    port,
    map_name,
    tm_port,
    retry_interval,
    max_wait_sec,
    allow_map_load,
):
    start_time = time.monotonic()
    map_load_requested = False

    while True:
        try:
            world, original_settings, traffic_manager, map_load_requested = _setup_world(
                client,
                map_name,
                tm_port,
                allow_map_load,
                map_load_requested,
            )
            if world is None:
                waited_sec = time.monotonic() - start_time
                logging.info(
                    "Map switch requested for %s. Waiting for CARLA to finish reloading (%.1fs elapsed).",
                    map_name,
                    waited_sec,
                )
                time.sleep(retry_interval)
                continue
            return world, original_settings, traffic_manager
        except RuntimeError as exc:
            message = str(exc)
            is_timeout = "time-out" in message and "simulator" in message
            is_map_reload = "Waiting for CARLA to finish loading" in message
            if not is_timeout and not is_map_reload:
                raise

            waited_sec = time.monotonic() - start_time
            if max_wait_sec > 0 and waited_sec >= max_wait_sec:
                raise SystemExit(
                    "Failed to connect to CARLA simulator.\n"
                    f"Checked {host}:{port} for {waited_sec:.1f} seconds.\n"
                    "Make sure CARLA is running and fully loaded before starting episode.py."
                ) from exc

            if is_map_reload:
                logging.info("%s", message)
            else:
                logging.warning(
                    "CARLA not ready at %s:%s yet; waited %.1fs so far. Retrying in %.1fs.",
                    host,
                    port,
                    waited_sec,
                    retry_interval,
                )
            time.sleep(retry_interval)


def _restore_world(world, original_settings, traffic_manager):
    if world is not None and original_settings is not None:
        world.apply_settings(original_settings)

    if traffic_manager is not None:
        traffic_manager.set_synchronous_mode(False)


def _attach_collision_sensor(world, vehicle):
    bp = world.get_blueprint_library().find("sensor.other.collision")
    sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
    state = {"collision": False}

    def _on_collision(_event):
        state["collision"] = True

    sensor.listen(_on_collision)
    return sensor, state


def _update_spectator(world, vehicle, camera_config):
    if not camera_config.get("enabled", False):
        return

    vehicle_transform = vehicle.get_transform()
    forward = vehicle_transform.get_forward_vector()
    up = vehicle_transform.get_up_vector()
    right = vehicle_transform.get_right_vector()

    distance = float(camera_config.get("distance", 8.0))
    height = float(camera_config.get("height", 3.5))
    lateral_offset = float(camera_config.get("lateral_offset", 0.0))
    pitch = float(camera_config.get("pitch", -15.0))

    location = vehicle_transform.location
    spectator_location = carla.Location(
        x=location.x - forward.x * distance + up.x * height + right.x * lateral_offset,
        y=location.y - forward.y * distance + up.y * height + right.y * lateral_offset,
        z=location.z - forward.z * distance + up.z * height + right.z * lateral_offset,
    )
    spectator_rotation = carla.Rotation(
        pitch=pitch,
        yaw=vehicle_transform.rotation.yaw,
        roll=0.0,
    )
    world.get_spectator().set_transform(carla.Transform(spectator_location, spectator_rotation))


def _tick_for_seconds(world, duration_sec):
    dt = float(world.get_settings().fixed_delta_seconds or 0.05)
    steps = max(1, int(round(duration_sec / dt)))
    for _ in range(steps):
        world.tick()


def _configure_vehicle_behavior(vehicle, traffic_manager, scenario_config):
    behavior = scenario_config.get("behavior", {})
    speed_diff = float(behavior.get("speed_percentage_difference", 0.0))
    traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff)


def _evaluate_episode(vehicle, route_locations, next_route_index, state, timers, termination, dt):
    location = vehicle.get_location()
    velocity = vehicle.get_velocity()
    speed_mps = math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)

    if route_locations and next_route_index < len(route_locations):
        goal_tolerance = float(termination.get("goal_tolerance_m", 5.0))
        if _distance_between(location, route_locations[next_route_index]) <= goal_tolerance:
            next_route_index += 1
            if next_route_index >= len(route_locations):
                return "route_completed", next_route_index

    if bool(termination.get("collision", True)) and state["collision"]:
        return "collision", next_route_index

    stuck_speed = float(termination.get("stuck_speed_mps", 0.5))
    stuck_time = float(termination.get("stuck_time_sec", 8.0))
    stuck_grace = float(termination.get("stuck_grace_sec", 0.0))
    if timers["elapsed_sec"] < stuck_grace:
        return None, next_route_index

    if speed_mps < stuck_speed:
        timers["stuck_sec"] += dt
        if timers["stuck_sec"] >= stuck_time:
            return "stuck", next_route_index
    else:
        timers["stuck_sec"] = 0.0

    return None, next_route_index


def _print_spawn_points(world):
    map_ = world.get_map()
    for index, transform in enumerate(map_.get_spawn_points()):
        location = transform.location
        rotation = transform.rotation
        print(
            f"[{index:03d}] "
            f"x={location.x:.2f} y={location.y:.2f} z={location.z:.2f} "
            f"yaw={rotation.yaw:.2f}"
        )


def main(args):
    scenario_path = Path(args.scenario).resolve()
    scenario_config = load_config_file(scenario_path)

    stack_path = Path(scenario_config.get("stack_file", "stack.json"))
    if not stack_path.is_absolute():
        stack_path = scenario_path.parent / stack_path
    stack_config = load_config_file(stack_path)
    stack_config = _apply_scenario_to_stack(stack_config, scenario_config)

    client = carla.Client(args.host, args.port)
    client.set_timeout(float(args.client_timeout))

    world = None
    original_settings = None
    traffic_manager = None

    try:
        logging.info(
            "Connecting to CARLA at %s:%s (timeout %.1fs)",
            args.host,
            args.port,
            float(args.client_timeout),
        )
        world, original_settings, traffic_manager = _connect_world_with_retry(
            client,
            args.host,
            args.port,
            scenario_config["map"],
            args.tm_port,
            float(args.retry_interval),
            float(args.max_connect_wait_sec),
            bool(scenario_config.get("allow_map_load", False)),
        )
        map_ = world.get_map()

        if args.print_spawn_points:
            _print_spawn_points(world)
            return

        route_locations = [
            _resolve_route_location(map_, point_config)
            for point_config in scenario_config.get("ego", {}).get("route_points", [])
        ]
        max_time_sec = float(scenario_config.get("episode", {}).get("max_time_sec", 90.0))
        respawn_delay_sec = float(scenario_config.get("episode", {}).get("respawn_delay_sec", 2.0))
        dt = float(world.get_settings().fixed_delta_seconds or 0.05)
        spectator_config = scenario_config.get("spectator", {})

        episode_count = 0
        while True:
            vehicles = []
            sensors = []
            collision_sensor = None
            collision_state = {"collision": False}

            try:
                vehicles, sensors, objects = spawn_actors_from_config(world, stack_config)
                vehicle = vehicles[0]

                collision_sensor, collision_state = _attach_collision_sensor(world, vehicle)
                sensors.append(collision_sensor)

                _ = world.tick()

                autopilot_enabled = bool(objects[0].get("autopilot", False))
                _set_vehicle_autopilot(vehicle, autopilot_enabled, traffic_manager)
                _configure_vehicle_behavior(vehicle, traffic_manager, scenario_config)
                _tick_for_seconds(world, float(scenario_config.get("episode", {}).get("settle_time_sec", 1.0)))
                _update_spectator(world, vehicle, spectator_config)

                episode_count += 1
                logging.info("Episode %d started", episode_count)

                elapsed_sec = 0.0
                next_route_index = 0
                timers = {"stuck_sec": 0.0, "elapsed_sec": 0.0}
                termination = scenario_config.get("termination", {})

                while True:
                    _ = world.tick()
                    elapsed_sec += dt
                    timers["elapsed_sec"] = elapsed_sec
                    _update_spectator(world, vehicle, spectator_config)

                    reason, next_route_index = _evaluate_episode(
                        vehicle,
                        route_locations,
                        next_route_index,
                        collision_state,
                        timers,
                        termination,
                        dt,
                    )
                    if reason is not None:
                        logging.info("Episode %d ended: %s", episode_count, reason)
                        break

                    if elapsed_sec >= max_time_sec:
                        logging.info("Episode %d ended: timeout", episode_count)
                        break
            except Exception as exc:
                logging.exception("Episode %d failed: %s", episode_count + 1, exc)

            finally:
                destroy_actors(sensors, vehicles)

            if args.once:
                break

            _tick_for_seconds(world, respawn_delay_sec)

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")
    except RuntimeError as exc:
        message = str(exc)
        if "time-out" in message and "simulator" in message:
            raise SystemExit(
                "Failed to connect to CARLA simulator.\n"
                f"Checked {args.host}:{args.port} for {float(args.client_timeout):.1f} seconds.\n"
                "Make sure CARLA is running and fully loaded before starting episode.py."
            ) from exc
        raise SystemExit(f"CARLA runtime error: {message}") from exc
    except Exception as exc:
        raise SystemExit(f"Episode runner failed: {exc}") from exc
    finally:
        _restore_world(world, original_settings, traffic_manager)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA episode runner")
    argparser.add_argument(
        "--host",
        metavar="H",
        default="localhost",
        help="IP of the host CARLA Simulator (default: localhost)",
    )
    argparser.add_argument(
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port of CARLA Simulator (default: 2000)",
    )
    argparser.add_argument(
        "--scenario",
        default=str(DEFAULT_SCENARIO_FILE),
        help="Scenario file to execute (default: scenario_town03.json next to this script)",
    )
    argparser.add_argument(
        "--tm-port",
        default=8000,
        type=int,
        help="Traffic Manager port (default: 8000)",
    )
    argparser.add_argument(
        "--client-timeout",
        default=15.0,
        type=float,
        help="CARLA client timeout in seconds (default: 15.0)",
    )
    argparser.add_argument(
        "--retry-interval",
        default=2.0,
        type=float,
        help="Seconds to wait before retrying a CARLA connection (default: 2.0)",
    )
    argparser.add_argument(
        "--max-connect-wait-sec",
        default=0.0,
        type=float,
        help="Maximum seconds to wait for CARLA before exiting; 0 means wait forever (default: 0.0)",
    )
    argparser.add_argument(
        "--print-spawn-points",
        action="store_true",
        help="Print Town03 spawn point indexes and exit",
    )
    argparser.add_argument(
        "--once",
        action="store_true",
        help="Run a single episode and exit",
    )
    argparser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="debug",
        help="print debug information",
    )

    parsed_args = argparser.parse_args()

    log_level = logging.DEBUG if parsed_args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", parsed_args.host, parsed_args.port)

    main(parsed_args)
