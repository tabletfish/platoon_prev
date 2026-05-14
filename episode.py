#!/usr/bin/env python3.10

import argparse
import copy
import logging
import math
import os
import random
import sys
import time
from pathlib import Path

import carla

from ros2_native import (
    _make_transform,
    destroy_actors,
    load_config_file,
    spawn_actors_from_config,
)


DEFAULT_SCENARIO_FILE = Path(__file__).with_name("scenario_town03.json")


def _load_basic_agent():
    try:
        from agents.navigation.basic_agent import BasicAgent
        return BasicAgent
    except ModuleNotFoundError:
        candidate_roots = []

        carla_root = os.environ.get("CARLA_ROOT")
        if carla_root:
            candidate_roots.append(Path(carla_root) / "PythonAPI" / "carla")

        candidate_roots.append(Path.home() / "carla_0.9.16" / "PythonAPI" / "carla")

        for candidate in candidate_roots:
            candidate_str = str(candidate)
            if candidate.exists() and candidate_str not in sys.path:
                sys.path.append(candidate_str)
                try:
                    from agents.navigation.basic_agent import BasicAgent
                    return BasicAgent
                except ModuleNotFoundError:
                    continue

    raise SystemExit(
        "Failed to import CARLA BasicAgent.\n"
        "Set CARLA_ROOT or install CARLA PythonAPI agents on the Python path."
    )


BasicAgent = _load_basic_agent()


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


def _get_actor_scenario_config(scenario_config, actor_id):
    return scenario_config.get(actor_id, {})


def _find_object_config(stack_config, actor_id):
    for object_config in stack_config.get("objects", []):
        if object_config.get("id") == actor_id:
            return object_config
    raise ValueError(f'stack.json must define an object with id="{actor_id}".')


def _resolve_actor_route_locations(map_, actor_config):
    route_locations = [
        _resolve_route_location(map_, point_config)
        for point_config in actor_config.get("route_points", [])
    ]

    if "destination" in actor_config:
        route_locations.append(_resolve_route_location(map_, actor_config["destination"]))

    return route_locations


def _project_to_driving_location(map_, location):
    waypoint = map_.get_waypoint(
        location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    return waypoint.transform.location, waypoint


def _project_spawn_transform(map_, spawn_config):
    requested_transform = _make_transform(spawn_config)
    projected_location, waypoint = _project_to_driving_location(map_, requested_transform.location)
    rotation = waypoint.transform.rotation
    rotation.roll = requested_transform.rotation.roll

    return carla.Transform(projected_location, rotation)


def _project_destination_location(map_, destination_config):
    destination_location = _resolve_route_location(map_, destination_config)
    projected_location, _ = _project_to_driving_location(map_, destination_location)
    return projected_location


def _apply_scenario_to_stack(stack_config, scenario_config):
    config = copy.deepcopy(stack_config)
    for actor_id in ("leader", "follower"):
        actor_stack_config = _find_object_config(config, actor_id)
        actor_scenario_config = _get_actor_scenario_config(scenario_config, actor_id)

        if "spawn_point" in actor_scenario_config:
            actor_stack_config["spawn_point"] = actor_scenario_config["spawn_point"]
            actor_stack_config.pop("spawn_point_index", None)
        elif "spawn_point_index" in actor_scenario_config:
            actor_stack_config["spawn_point_index"] = int(actor_scenario_config["spawn_point_index"])
            actor_stack_config.pop("spawn_point", None)

        actor_stack_config["autopilot"] = bool(actor_scenario_config.get("autopilot", False))

    return config


def _project_location_list(map_, route_locations):
    return [
        _project_destination_location(map_, {"location": {"x": location.x, "y": location.y, "z": location.z}})
        for location in route_locations
    ]


def _project_follower_spawn_from_leader(map_, leader_transform, follower_config):
    gap_m = float(follower_config.get("spawn_gap_m", 12.0))
    waypoint = map_.get_waypoint(
        leader_transform.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        raise RuntimeError("Could not project leader spawn to a driving waypoint for follower placement.")

    previous_waypoints = waypoint.previous(gap_m)
    follower_waypoint = previous_waypoints[0] if previous_waypoints else waypoint
    transform = follower_waypoint.transform
    transform.location.z += 0.05
    return transform


def _align_scenario_to_map(stack_config, scenario_config, map_):
    config = copy.deepcopy(stack_config)
    leader_stack_config = _find_object_config(config, "leader")
    follower_stack_config = _find_object_config(config, "follower")

    projected_leader_spawn = _project_spawn_transform(map_, leader_stack_config["spawn_point"])
    leader_stack_config["spawn_point"] = {
        "x": projected_leader_spawn.location.x,
        "y": projected_leader_spawn.location.y,
        "z": projected_leader_spawn.location.z + 0.05,
        "roll": projected_leader_spawn.rotation.roll,
        "pitch": projected_leader_spawn.rotation.pitch,
        "yaw": projected_leader_spawn.rotation.yaw,
    }

    follower_scenario_config = _get_actor_scenario_config(scenario_config, "follower")
    if "spawn_point" in follower_stack_config:
        projected_follower_spawn = _project_spawn_transform(map_, follower_stack_config["spawn_point"])
    else:
        projected_follower_spawn = _project_follower_spawn_from_leader(
            map_,
            projected_leader_spawn,
            follower_scenario_config,
        )
    follower_stack_config["spawn_point"] = {
        "x": projected_follower_spawn.location.x,
        "y": projected_follower_spawn.location.y,
        "z": projected_follower_spawn.location.z + 0.05,
        "roll": projected_follower_spawn.rotation.roll,
        "pitch": projected_follower_spawn.rotation.pitch,
        "yaw": projected_follower_spawn.rotation.yaw,
    }
    follower_stack_config.pop("spawn_point_index", None)

    leader_route_locations = _project_location_list(
        map_,
        _resolve_actor_route_locations(map_, _get_actor_scenario_config(scenario_config, "leader")),
    )
    follower_route_config = _get_actor_scenario_config(scenario_config, "follower")
    follower_route_locations = _resolve_actor_route_locations(map_, follower_route_config)
    if not follower_route_locations:
        follower_route_locations = list(leader_route_locations)
    else:
        follower_route_locations = _project_location_list(map_, follower_route_locations)

    return config, {"leader": leader_route_locations, "follower": follower_route_locations}


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


def _update_spectator(world, vehicles, camera_config):
    if not camera_config.get("enabled", False):
        return

    if not vehicles:
        return

    lead_vehicle = vehicles[0]
    lead_transform = lead_vehicle.get_transform()
    forward = lead_transform.get_forward_vector()
    up = lead_transform.get_up_vector()
    right = lead_transform.get_right_vector()

    distance = float(camera_config.get("distance", 8.0))
    height = float(camera_config.get("height", 3.5))
    lateral_offset = float(camera_config.get("lateral_offset", 0.0))
    pitch = float(camera_config.get("pitch", -15.0))
    yaw = lead_transform.rotation.yaw

    if len(vehicles) >= 2:
        trailing_transform = vehicles[1].get_transform()
        location = carla.Location(
            x=(lead_transform.location.x + trailing_transform.location.x) * 0.5,
            y=(lead_transform.location.y + trailing_transform.location.y) * 0.5,
            z=(lead_transform.location.z + trailing_transform.location.z) * 0.5,
        )
        spacing = _distance_between(lead_transform.location, trailing_transform.location)
        distance = max(distance, spacing * float(camera_config.get("distance_scale", 0.8)))
        height = max(height, 4.0 + spacing * float(camera_config.get("height_scale", 0.08)))
    else:
        location = lead_transform.location

    spectator_location = carla.Location(
        x=location.x - forward.x * distance + up.x * height + right.x * lateral_offset,
        y=location.y - forward.y * distance + up.y * height + right.y * lateral_offset,
        z=location.z - forward.z * distance + up.z * height + right.z * lateral_offset,
    )
    spectator_rotation = carla.Rotation(
        pitch=pitch,
        yaw=yaw,
        roll=0.0,
    )
    world.get_spectator().set_transform(carla.Transform(spectator_location, spectator_rotation))


def _tick_for_seconds(world, duration_sec):
    dt = float(world.get_settings().fixed_delta_seconds or 0.05)
    steps = max(1, int(round(duration_sec / dt)))
    for _ in range(steps):
        world.tick()


def _configure_traffic_lights(world, scenario_config):
    traffic_light_config = scenario_config.get("traffic_lights", {})
    if not bool(traffic_light_config.get("force_green", False)):
        return

    for traffic_light in world.get_actors().filter("*traffic_light*"):
        traffic_light.set_state(carla.TrafficLightState.Green)
        traffic_light.freeze(True)


def _get_speed_mps(vehicle):
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)


def _select_profile_target_speed_mps(scenario_config):
    speed_profile = scenario_config.get("speed_profile", {})
    target_speeds = [float(value) for value in speed_profile.get("target_speeds_mps", [15.0, 17.5, 20.0])]
    if not target_speeds:
        raise ValueError("speed_profile.target_speeds_mps must not be empty.")
    return random.choice(target_speeds)


def _clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def _create_speed_profile_state(scenario_config, initial_speed_mps):
    speed_profile = scenario_config.get("speed_profile", {})
    target_speed_mps = _select_profile_target_speed_mps(scenario_config)
    commanded_speed_mps = max(float(speed_profile.get("initial_speed_mps", initial_speed_mps)), initial_speed_mps)
    return {
        "phase": "accelerate",
        "target_speed_mps": target_speed_mps,
        "commanded_speed_mps": min(commanded_speed_mps, target_speed_mps),
        "accel_mps2": float(speed_profile.get("accel_mps2", 2.0)),
        "decel_mps2": float(speed_profile.get("decel_mps2", 3.5)),
        "cruise_time_sec": float(speed_profile.get("cruise_time_sec", 7.0)),
        "stop_speed_mps": float(speed_profile.get("stop_speed_mps", 0.4)),
        "stop_hold_sec": float(speed_profile.get("stop_hold_sec", 1.0)),
        "speed_tolerance_mps": float(speed_profile.get("speed_tolerance_mps", 0.6)),
        "cruise_elapsed_sec": 0.0,
        "stop_elapsed_sec": 0.0,
        "profile_completed": False,
    }


def _create_follower_pid_state(scenario_config):
    follower_control = scenario_config.get("follower_control", {})
    return {
        "h": float(follower_control.get("h", 1.0)),
        "d0": float(follower_control.get("d0", 7.0)),
        "kp": float(follower_control.get("kp", 0.4)),
        "ki": float(follower_control.get("ki", 0.01)),
        "kd": float(follower_control.get("kd", 0.1)),
        "integral_error": 0.0,
        "integral_limit": float(follower_control.get("integral_limit", 30.0)),
        "emergency_distance_m": float(follower_control.get("emergency_distance_m", 5.0)),
        "launch_speed_threshold_mps": float(follower_control.get("launch_speed_threshold_mps", 0.5)),
        "launch_lead_speed_threshold_mps": float(follower_control.get("launch_lead_speed_threshold_mps", 1.0)),
        "launch_spacing_error_threshold_m": float(follower_control.get("launch_spacing_error_threshold_m", 2.0)),
        "launch_min_throttle": float(follower_control.get("launch_min_throttle", 0.2)),
        "stop_speed_mps": float(follower_control.get("stop_speed_mps", 0.4)),
        "stop_hold_sec": float(follower_control.get("stop_hold_sec", 1.0)),
        "stop_elapsed_sec": 0.0,
        "log_interval_ticks": max(1, int(follower_control.get("log_interval_ticks", 10))),
    }


def _update_speed_profile(vehicle, agent, profile_state, dt):
    current_speed_mps = _get_speed_mps(vehicle)
    phase = profile_state["phase"]

    if phase == "accelerate":
        profile_state["commanded_speed_mps"] = min(
            profile_state["target_speed_mps"],
            profile_state["commanded_speed_mps"] + profile_state["accel_mps2"] * dt,
        )
        if current_speed_mps >= profile_state["target_speed_mps"] - profile_state["speed_tolerance_mps"]:
            profile_state["phase"] = "cruise"
            profile_state["cruise_elapsed_sec"] = 0.0

    elif phase == "cruise":
        profile_state["commanded_speed_mps"] = profile_state["target_speed_mps"]
        profile_state["cruise_elapsed_sec"] += dt
        if profile_state["cruise_elapsed_sec"] >= profile_state["cruise_time_sec"]:
            profile_state["phase"] = "decelerate"

    elif phase == "decelerate":
        profile_state["commanded_speed_mps"] = max(
            0.0,
            profile_state["commanded_speed_mps"] - profile_state["decel_mps2"] * dt,
        )
        if current_speed_mps <= profile_state["stop_speed_mps"]:
            profile_state["stop_elapsed_sec"] += dt
            if profile_state["stop_elapsed_sec"] >= profile_state["stop_hold_sec"]:
                profile_state["phase"] = "stopped"
                profile_state["profile_completed"] = True
        else:
            profile_state["stop_elapsed_sec"] = 0.0

    agent.set_target_speed(profile_state["commanded_speed_mps"] * 3.6)


def _apply_speed_profile_to_control(control, profile_state):
    if profile_state["phase"] != "decelerate":
        return control

    if profile_state["commanded_speed_mps"] > 1.5:
        return control

    control.throttle = 0.0
    control.brake = max(control.brake, 0.35)
    return control


def _build_basic_agent(vehicle, map_, scenario_config, route_locations, option_overrides=None):
    if not route_locations:
        raise ValueError("Scenario must define route_points and/or destination for BasicAgent.")

    agent_config = scenario_config.get("basic_agent", {})
    target_speed = float(agent_config.get("target_speed_kmh", 20.0))
    option_keys = (
        "ignore_traffic_lights",
        "ignore_stop_signs",
        "ignore_vehicles",
        "use_bbs_detection",
        "sampling_resolution",
        "base_tlight_threshold",
        "base_vehicle_threshold",
        "detection_speed_ratio",
        "max_brake",
        "offset",
    )
    opt_dict = {
        key: agent_config[key]
        for key in option_keys
        if key in agent_config
    }
    if option_overrides:
        opt_dict.update(option_overrides)

    agent = BasicAgent(vehicle, target_speed=target_speed, opt_dict=opt_dict, map_inst=map_)
    if bool(agent_config.get("follow_speed_limits", False)):
        agent.follow_speed_limits(True)

    route_plan = []
    start_location = vehicle.get_location()
    for target_location in route_locations:
        start_waypoint = map_.get_waypoint(start_location)
        end_waypoint = map_.get_waypoint(target_location)
        segment_plan = agent.trace_route(start_waypoint, end_waypoint)
        if not segment_plan:
            raise RuntimeError(
                f"BasicAgent could not build a route segment to x={target_location.x:.2f}, y={target_location.y:.2f}."
            )

        if route_plan and segment_plan:
            segment_plan = segment_plan[1:]
        route_plan.extend(segment_plan)
        start_location = target_location

    agent.set_global_plan(route_plan)
    return agent


def _advance_route_progress(vehicle, route_locations, next_route_index, goal_tolerance):
    if route_locations and next_route_index < len(route_locations):
        if _distance_between(vehicle.get_location(), route_locations[next_route_index]) <= goal_tolerance:
            next_route_index += 1
    return next_route_index


def _compute_follower_control(leader_vehicle, follower_vehicle, follower_agent, follower_pid_state, dt):
    lead_speed_mps = _get_speed_mps(leader_vehicle)
    ego_speed_mps = _get_speed_mps(follower_vehicle)
    distance_m = _distance_between(leader_vehicle.get_location(), follower_vehicle.get_location())
    desired_distance_m = follower_pid_state["d0"] + follower_pid_state["h"] * ego_speed_mps
    spacing_error_m = distance_m - desired_distance_m
    relative_velocity_mps = lead_speed_mps - ego_speed_mps

    follower_pid_state["integral_error"] = _clamp(
        follower_pid_state["integral_error"] + spacing_error_m * dt,
        -follower_pid_state["integral_limit"],
        follower_pid_state["integral_limit"],
    )

    u = (
        follower_pid_state["kp"] * spacing_error_m
        + follower_pid_state["ki"] * follower_pid_state["integral_error"]
        + follower_pid_state["kd"] * relative_velocity_mps
    )

    throttle = 0.0
    brake = 0.0
    if distance_m < follower_pid_state["emergency_distance_m"]:
        brake = 1.0
    elif u >= 0.0:
        throttle = _clamp(u / 3.0, 0.0, 1.0)
    else:
        brake = _clamp((-u) / 4.0, 0.0, 1.0)

    if (
        ego_speed_mps <= follower_pid_state["launch_speed_threshold_mps"]
        and lead_speed_mps >= follower_pid_state["launch_lead_speed_threshold_mps"]
        and spacing_error_m >= follower_pid_state["launch_spacing_error_threshold_m"]
    ):
        throttle = max(throttle, follower_pid_state["launch_min_throttle"])
        brake = 0.0

    lateral_control = follower_agent.run_step()
    control = carla.VehicleControl()
    control.steer = lateral_control.steer
    control.throttle = throttle
    control.brake = brake
    control.hand_brake = False
    control.reverse = False
    control.manual_gear_shift = False

    if ego_speed_mps <= follower_pid_state["stop_speed_mps"]:
        follower_pid_state["stop_elapsed_sec"] += dt
    else:
        follower_pid_state["stop_elapsed_sec"] = 0.0

    metrics = {
        "distance_m": distance_m,
        "desired_distance_m": desired_distance_m,
        "spacing_error_m": spacing_error_m,
        "relative_velocity_mps": relative_velocity_mps,
        "lead_speed_mps": lead_speed_mps,
        "ego_speed_mps": ego_speed_mps,
        "throttle": throttle,
        "brake": brake,
    }
    return control, metrics


def _log_follower_metrics(episode_count, tick_count, metrics):
    logging.info(
        (
            "Episode %d follower tick %d: d=%.2f d_des=%.2f err=%.2f "
            "rel_v=%.2f v_lead=%.2f v_follower=%.2f throttle=%.2f brake=%.2f"
        ),
        episode_count,
        tick_count,
        metrics["distance_m"],
        metrics["desired_distance_m"],
        metrics["spacing_error_m"],
        metrics["relative_velocity_mps"],
        metrics["lead_speed_mps"],
        metrics["ego_speed_mps"],
        metrics["throttle"],
        metrics["brake"],
    )


def _evaluate_episode(
    leader_vehicle,
    follower_vehicle,
    leader_profile_state,
    follower_pid_state,
    route_state,
    collision_states,
    timers,
    termination,
    dt,
):
    goal_tolerance = float(termination.get("goal_tolerance_m", 5.0))
    route_state["leader_index"] = _advance_route_progress(
        leader_vehicle,
        route_state["leader_route_locations"],
        route_state["leader_index"],
        goal_tolerance,
    )
    route_state["follower_index"] = _advance_route_progress(
        follower_vehicle,
        route_state["follower_route_locations"],
        route_state["follower_index"],
        goal_tolerance,
    )

    if leader_profile_state["profile_completed"] and (
        follower_pid_state["stop_elapsed_sec"] >= follower_pid_state["stop_hold_sec"]
    ):
        return "profile_completed"

    if collision_states["leader"]["collision"]:
        return "leader_collision"

    if collision_states["follower"]["collision"]:
        return "follower_collision"

    speed_mps = _get_speed_mps(leader_vehicle)
    if leader_profile_state["phase"] == "decelerate" and leader_profile_state["commanded_speed_mps"] <= 1.0:
        timers["stuck_sec"] = 0.0
        return None

    stuck_speed = float(termination.get("stuck_speed_mps", 0.5))
    stuck_time = float(termination.get("stuck_time_sec", 8.0))
    stuck_grace = float(termination.get("stuck_grace_sec", 0.0))
    if timers["elapsed_sec"] < stuck_grace:
        return None

    if speed_mps < stuck_speed:
        timers["stuck_sec"] += dt
        if timers["stuck_sec"] >= stuck_time:
            return "leader_stuck"
    else:
        timers["stuck_sec"] = 0.0

    return None


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
        _configure_traffic_lights(world, scenario_config)

        if args.print_spawn_points:
            _print_spawn_points(world)
            return

        stack_config, route_locations_by_actor = _align_scenario_to_map(stack_config, scenario_config, map_)
        max_time_sec = float(scenario_config.get("episode", {}).get("max_time_sec", 90.0))
        respawn_delay_sec = float(scenario_config.get("episode", {}).get("respawn_delay_sec", 2.0))
        dt = float(world.get_settings().fixed_delta_seconds or 0.05)
        spectator_config = scenario_config.get("spectator", {})

        episode_count = 0
        while True:
            vehicles = []
            sensors = []
            collision_states = {}
            leader_agent = None
            follower_agent = None
            leader_profile_state = None
            follower_pid_state = None

            try:
                vehicles, sensors, objects = spawn_actors_from_config(world, stack_config)
                vehicles_by_id = {
                    object_config["id"]: vehicle
                    for vehicle, object_config in zip(vehicles, objects)
                }
                leader_vehicle = vehicles_by_id["leader"]
                follower_vehicle = vehicles_by_id["follower"]

                for actor_id, vehicle in vehicles_by_id.items():
                    collision_sensor, collision_state = _attach_collision_sensor(world, vehicle)
                    sensors.append(collision_sensor)
                    collision_states[actor_id] = collision_state

                _ = world.tick()

                _tick_for_seconds(world, float(scenario_config.get("episode", {}).get("settle_time_sec", 1.0)))
                leader_agent = _build_basic_agent(
                    leader_vehicle,
                    map_,
                    scenario_config,
                    route_locations_by_actor["leader"],
                )
                follower_agent = _build_basic_agent(
                    follower_vehicle,
                    map_,
                    scenario_config,
                    route_locations_by_actor["follower"],
                    option_overrides={
                        "ignore_traffic_lights": True,
                        "ignore_stop_signs": True,
                        "ignore_vehicles": True,
                    },
                )
                leader_profile_state = _create_speed_profile_state(scenario_config, _get_speed_mps(leader_vehicle))
                follower_pid_state = _create_follower_pid_state(scenario_config)
                _update_spectator(world, [leader_vehicle, follower_vehicle], spectator_config)

                episode_count += 1
                logging.info(
                    "Episode %d started (leader target speed %.1f m/s)",
                    episode_count,
                    leader_profile_state["target_speed_mps"],
                )

                elapsed_sec = 0.0
                tick_count = 0
                route_state = {
                    "leader_index": 0,
                    "follower_index": 0,
                    "leader_route_locations": route_locations_by_actor["leader"],
                    "follower_route_locations": route_locations_by_actor["follower"],
                }
                timers = {"stuck_sec": 0.0, "elapsed_sec": 0.0}
                termination = scenario_config.get("termination", {})

                while True:
                    _ = world.tick()
                    elapsed_sec += dt
                    tick_count += 1
                    timers["elapsed_sec"] = elapsed_sec
                    _update_speed_profile(leader_vehicle, leader_agent, leader_profile_state, dt)
                    leader_control = leader_agent.run_step()
                    leader_control = _apply_speed_profile_to_control(leader_control, leader_profile_state)
                    leader_vehicle.apply_control(leader_control)

                    follower_control, follower_metrics = _compute_follower_control(
                        leader_vehicle,
                        follower_vehicle,
                        follower_agent,
                        follower_pid_state,
                        dt,
                    )
                    follower_vehicle.apply_control(follower_control)
                    if tick_count % follower_pid_state["log_interval_ticks"] == 0:
                        _log_follower_metrics(episode_count, tick_count, follower_metrics)

                    _update_spectator(world, [leader_vehicle, follower_vehicle], spectator_config)

                    reason = _evaluate_episode(
                        leader_vehicle,
                        follower_vehicle,
                        leader_profile_state,
                        follower_pid_state,
                        route_state,
                        collision_states,
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
