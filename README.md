# CARLA Native ROS2 Episode Runner

This directory contains a single-ego CARLA native ROS2 setup built around:

- `ros2_native.py`: native ROS2 vehicle/sensor spawning helpers
- `episode.py`: repeated episode runner
- `stack.json`: ego vehicle and sensor stack
- `scenario_town03.json`: map and episode configuration

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
python3 episode.py
```

`episode.py` will connect to CARLA on `localhost:2000`, switch the map to `Town03_Opt`, spawn the ego vehicle, and repeat episodes.

## Useful Commands

Run a single episode:

```bash
python3.10 episode.py --once
```

Print spawn points for the current scenario map:

```bash
python3.10 episode.py --print-spawn-points
```

Run the lower-level native stack only:

```bash
python3.10 ros2_native.py -f stack.json
```

## Notes

- On this machine, `python3` may point to Anaconda Python and fail to import `carla`. Use `python3.10`.
- `Town03` crashes in the current CARLA setup, so the scenario uses `Town03_Opt`.
