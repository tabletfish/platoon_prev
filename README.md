# CARLA Native ROS2 Episode Runner

This directory contains a single-ego CARLA native ROS2 setup built around:

- `ros2_native.py`: native ROS2 vehicle/sensor spawning helpers
- `episode.py`: repeated episode runner
- `stack.json`: ego vehicle and sensor stack
- `scenario_town03.json`: map and episode configuration
- `references/scenario_main.py`: external reference script kept separately from the runnable stack

## Prerequisites

- CARLA 0.9.16 installed at `/home/jungjinwoo/carla_0.9.16`
- Python `3.10`
- CARLA Python API importable from `python3.10`

## Run

1. Start CARLA:

```bash
cd ~/carla_0.9.16
./CarlaUE4.sh -quality-level=Low
```

2. Start the episode runner:

```bash
cd ~/종합설계/label_file
python3.10 episode.py

```

3. 위치 추출:

```bash
python3.10 -c "import carla; c=carla.Client('localhost',2000); c.set_timeout(5.0); w=c.get_world(); t=w.get_spectator().get_transform(); print(f'x={t.location.x:.2f}, y={t.location.y:.2f}, z={t.location.z:.2f}, roll={t.rotation.roll:.2f}, pitch={t.rotation.pitch:.2f}, yaw={t.rotation.yaw:.2f}')"
```

`episode.py` will connect to CARLA on `localhost:2000`, switch the map to `Town03_Opt`, spawn the ego vehicle, and repeat episodes.

## Useful Commands

Run a single episode:

```bash
python3 episode.py 
```

Print spawn points for the current scenario map:

```bash
python3 episode.py --print-spawn-points
```

Run the lower-level native stack only:

```bash
python3 ros2_native.py -f stack.json
```

Do not run `ros2_native.py` at the same time as `episode.py`.
`episode.py` already spawns the vehicles and sensors and owns the CARLA tick loop.

## Notes

- On this machine, `python3` may point to Anaconda Python and fail to import `carla`. Use `python3.10`.
- `Town03` crashes in the current CARLA setup, so the scenario uses `Town03_Opt`.
