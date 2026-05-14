#!/usr/bin/env python

# Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Based on:
# https://raw.githubusercontent.com/carla-simulator/carla/ue5-dev/PythonAPI/examples/ros2/ros2_native.py
#
# Custom changes for this copy:
# 1. Added support for "objects": [] so a single stack.json can spawn multiple vehicles.
# 2. Added per-vehicle "spawn_point_index" and "spawn_point" config keys.
# 3. Added per-vehicle "autopilot" config key.
# 4. Added "--tm-port" CLI argument so the Traffic Manager port can be changed.
# 5. Sensor ROS names are prefixed with the vehicle id to avoid name collisions.

import argparse
import json
import logging
from pathlib import Path

try:
    import carla
except ModuleNotFoundError as exc:
    raise SystemExit(
        "Failed to import the CARLA Python API.\n"
        "Run this script with Python 3.10 on this machine, for example:\n"
        "  python3.10 ros2_native.py\n"
        "If you are in a different directory, pass the config explicitly:\n"
        "  python3.10 ros2_native.py --file /path/to/stack.json"
    ) from exc


DEFAULT_CONFIG_FILE = Path(__file__).with_name("stack.json")


def _make_transform(spawn_point):
    """Build a CARLA transform from a JSON object."""
    return carla.Transform(
        location=carla.Location(
            x=spawn_point["x"],
            y=spawn_point["y"],
            z=spawn_point["z"],
        ),
        rotation=carla.Rotation(
            roll=spawn_point["roll"],
            pitch=spawn_point["pitch"],
            yaw=spawn_point["yaw"],
        ),
    )


def load_config_file(path):
    with open(path, encoding="utf-8") as file_obj:
        return json.load(file_obj)


def _resolve_vehicle_spawn_point(map_, config):
    # New config key:
    # - "spawn_point": explicit world transform for the vehicle.
    # - "spawn_point_index": index into map_.get_spawn_points().
    # "spawn_point" takes priority when both are present.
    if "spawn_point" in config:
        return _make_transform(config["spawn_point"])

    spawn_points = map_.get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No vehicle spawn points are available on this map.")

    spawn_index = int(config.get("spawn_point_index", 0))
    if spawn_index < 0 or spawn_index >= len(spawn_points):
        raise IndexError(
            f"spawn_point_index={spawn_index} is out of range; "
            f"available range is 0..{len(spawn_points) - 1}."
        )

    return spawn_points[spawn_index]


def _candidate_spawn_points(map_, spawn_point):
    candidates = [spawn_point]
    waypoint = map_.get_waypoint(
        spawn_point.location,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )
    if waypoint is None:
        return candidates

    offsets_m = (2.0, 4.0, 6.0, 8.0, 12.0, 16.0, 24.0)
    seen = {
        (
            round(spawn_point.location.x, 2),
            round(spawn_point.location.y, 2),
            round(spawn_point.location.z, 2),
            round(spawn_point.rotation.yaw, 2),
        )
    }

    for offset in offsets_m:
        for next_wp in waypoint.next(offset):
            transform = carla.Transform(next_wp.transform.location, spawn_point.rotation)
            transform.location.z += 0.05
            key = (
                round(transform.location.x, 2),
                round(transform.location.y, 2),
                round(transform.location.z, 2),
                round(transform.rotation.yaw, 2),
            )
            if key not in seen:
                seen.add(key)
                candidates.append(transform)

        for prev_wp in waypoint.previous(offset):
            transform = carla.Transform(prev_wp.transform.location, spawn_point.rotation)
            transform.location.z += 0.05
            key = (
                round(transform.location.x, 2),
                round(transform.location.y, 2),
                round(transform.location.z, 2),
                round(transform.rotation.yaw, 2),
            )
            if key not in seen:
                seen.add(key)
                candidates.append(transform)

    spawn_points = sorted(
        map_.get_spawn_points(),
        key=lambda transform: transform.location.distance(spawn_point.location),
    )
    for fallback_spawn_point in spawn_points[:10]:
        transform = carla.Transform(fallback_spawn_point.location, spawn_point.rotation)
        transform.location.z = fallback_spawn_point.location.z + 0.05
        key = (
            round(transform.location.x, 2),
            round(transform.location.y, 2),
            round(transform.location.z, 2),
            round(transform.rotation.yaw, 2),
        )
        if key not in seen:
            seen.add(key)
            candidates.append(transform)

    return candidates


def _destroy_existing_vehicle_with_role_name(world, role_name):
    for vehicle in world.get_actors().filter("vehicle.*"):
        if vehicle.attributes.get("role_name") == role_name:
            vehicle.destroy()


def _clear_vehicles_near_spawn(world, location, radius_m):
    if radius_m <= 0:
        return

    for vehicle in world.get_actors().filter("vehicle.*"):
        if vehicle.get_transform().location.distance(location) <= radius_m:
            vehicle.destroy()


def _setup_vehicle(world, config):
    vehicle_type = config.get("type")
    vehicle_id = config.get("id")
    logging.debug("Spawning vehicle: %s (%s)", vehicle_id, vehicle_type)

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    matches = bp_library.filter(vehicle_type)
    if not matches:
        raise ValueError(f'No vehicle blueprint matches "{vehicle_type}".')

    bp = matches[0]
    bp.set_attribute("role_name", vehicle_id)
    bp.set_attribute("ros_name", vehicle_id)

    _destroy_existing_vehicle_with_role_name(world, vehicle_id)

    spawn_point = _resolve_vehicle_spawn_point(map_, config)
    clear_radius_m = float(config.get("clear_spawn_radius_m", 12.0))
    _clear_vehicles_near_spawn(world, spawn_point.location, clear_radius_m)
    vehicle = None
    for candidate_spawn_point in _candidate_spawn_points(map_, spawn_point):
        vehicle = world.try_spawn_actor(bp, candidate_spawn_point)
        if vehicle is not None:
            break

    if vehicle is None:
        raise RuntimeError(
            f'Failed to spawn vehicle "{vehicle_id}". '
            "The spawn point and nearby lane positions may be blocked by another actor."
        )

    return vehicle


def _setup_sensors(world, vehicle, vehicle_config):
    bp_library = world.get_blueprint_library()
    sensors = []

    vehicle_id = vehicle_config.get("id")
    for sensor in vehicle_config.get("sensors", []):
        logging.debug("Spawning sensor for %s: %s", vehicle_id, sensor)

        matches = bp_library.filter(sensor.get("type"))
        if not matches:
            raise ValueError(f'No sensor blueprint matches "{sensor.get("type")}".')

        bp = matches[0]

        # New behavior:
        # Prefix the sensor ROS name with the vehicle id so repeated sensor ids
        # such as "rgb" do not collide across multiple vehicles.
        sensor_name = f'{vehicle_id}_{sensor.get("id")}'
        bp.set_attribute("ros_name", sensor_name)
        bp.set_attribute("role_name", sensor_name)

        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        wp = carla.Transform(
            location=carla.Location(
                x=sensor["spawn_point"]["x"],
                y=-sensor["spawn_point"]["y"],
                z=sensor["spawn_point"]["z"],
            ),
            rotation=carla.Rotation(
                roll=sensor["spawn_point"]["roll"],
                pitch=-sensor["spawn_point"]["pitch"],
                yaw=-sensor["spawn_point"]["yaw"],
            ),
        )

        spawned_sensor = world.spawn_actor(bp, wp, attach_to=vehicle)
        spawned_sensor.enable_for_ros()
        sensors.append(spawned_sensor)

    return sensors


def _load_objects(config):
    # New top-level config key:
    # - "objects": [{...}, {...}]
    # Legacy single-vehicle config is still accepted for compatibility.
    if "objects" in config:
        return config["objects"]
    return [config]


def spawn_actors_from_config(world, config):
    vehicles = []
    sensors = []

    objects = _load_objects(config)
    for vehicle_config in objects:
        vehicle = _setup_vehicle(world, vehicle_config)
        vehicles.append(vehicle)
        sensors.extend(_setup_sensors(world, vehicle, vehicle_config))

    return vehicles, sensors, objects


def destroy_actors(sensors, vehicles):
    for sensor in reversed(sensors):
        sensor.destroy()

    for vehicle in reversed(vehicles):
        vehicle.destroy()


def _set_vehicle_autopilot(vehicle, enabled, traffic_manager):
    if not enabled:
        return

    # New per-vehicle config key:
    # - "autopilot": true/false (defaults to true).
    try:
        vehicle.set_autopilot(True, traffic_manager.get_port())
    except TypeError:
        vehicle.set_autopilot(True)


def main(args):
    world = None
    original_settings = None
    vehicles = []
    sensors = []

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)

        world = client.get_world()

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_synchronous_mode(True)

        config = load_config_file(args.file)
        vehicles, sensors, objects = spawn_actors_from_config(world, config)

        _ = world.tick()

        for vehicle, vehicle_config in zip(vehicles, objects):
            autopilot_enabled = bool(vehicle_config.get("autopilot", True))
            _set_vehicle_autopilot(vehicle, autopilot_enabled, traffic_manager)

        logging.info("Running with %d vehicle(s)...", len(vehicles))
        for vehicle in vehicles:
            logging.info(
                "Spawned vehicle id=%s role_name=%s type=%s",
                vehicle.id,
                vehicle.attributes.get("role_name", ""),
                vehicle.type_id,
            )

        while True:
            _ = world.tick()

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")

    finally:
        if world is not None and original_settings is not None:
            world.apply_settings(original_settings)

        destroy_actors(sensors, vehicles)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA ROS2 native")
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
        "-f",
        "--file",
        default=str(DEFAULT_CONFIG_FILE),
        help="Config file to execute (default: stack.json next to this script)",
    )
    argparser.add_argument(
        "--tm-port",
        default=8000,
        type=int,
        help="Traffic Manager port (new argument, default: 8000)",
    )
    argparser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="debug",
        help="print debug information",
    )

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", args.host, args.port)

    main(args)
