# Remote Parking Simulation Environment

This repository provides a ROS 2 and Gazebo simulation environment for LIMO-based remote parking. It includes a multi-vehicle parking world, vehicle teleoperation, parking algorithm execution, and vehicle position reset support.

## Development and Validation Purpose

This common Gazebo base environment was created to support team members in feature development and preliminary validation. The initial environment consists of 30 static vehicles and four LIMO vehicles, allowing perception, path-planning, and control functions to be developed and validated under consistent conditions.

For repeated tests, vehicle positions can be reset without restarting Gazebo. Team members can also adapt the number of vehicles and the scenarios to suit the goals of each algorithm. This repository is intended as a simulation-based development and validation environment.

## Features

- Launches the Gazebo parking world and LIMO vehicle models
- Runs the slot tracker and parking control nodes together
- Provides keyboard teleoperation and driving boundary limits for LIMO2
- Resets vehicle positions without restarting Gazebo

## Packages

| Package | Purpose |
| --- | --- |
| `limo_car` | LIMO models, Ackermann configuration, and RViz configuration |
| `limo_gazebosim` | Simulation worlds, vehicle spawning and reset tools, and launch support |
| `my_valet_parking` | Slot tracking, teleoperation, pull-out control, and parking control |
| `parking_pkg` | Parking scenario state management and integrated launch support |

## Running the Environment

Requirements: Ubuntu 22.04, ROS 2 Humble, and Gazebo Classic.

When the simulation is ready, launch the parking scenario:

```bash
ros2 launch parking_pkg valet.launch.py
```

Key tools:

- `src/my_valet_parking/scripts/run_parking_algorithm.sh`: runs slot tracking and parking control
- `src/my_valet_parking/my_valet_parking/limo2_arrow_teleop.py`: provides keyboard control for LIMO2
- `src/limo_gazebosim/scripts/reset_limo_positions.sh`: resets vehicle positions while keeping Gazebo running

## Sources

Third-party packages and license information are listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
