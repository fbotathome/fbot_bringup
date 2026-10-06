<div align="center">

<img width="4447" height="719" alt="fbot_bringup" src="https://github.com/user-attachments/assets/f1dad98e-2948-4dcb-a563-d1827117c89f" />

![UBUNTU](https://img.shields.io/badge/UBUNTU-22.04-orange?style=for-the-badsge&logo=ubuntu)
![python](https://img.shields.io/badge/python-3.10-blue?style=for-the-badsge&logo=python)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badsge&logo=ros)

Bringup and launch files for BORIS — camera, vision, speech, arm, face, etc.

Overview • Architecture • Installation • Usage • Development

</div>

---

## Overview

fbot_bringup contains ROS 2 launch files and small helpers used to start and configure the robot stack: camera drivers, vision nodes, speech components, arm controllers, etc. The launches are intended to be used individually or combined in a fbot_behavior launchfile.

---

## Package layout

```
fbot_bringup/
├── launch/
│   ├── camera.launch.py
│   ├── vision.launch.py
│   ├── full_bringup.launch.py
│   ├── face_recognition.launch.py
│   ├── hotword_detector.launch.py
│   ├── riva_speech_to_text.launch.py
│   └── ... (other launches)
├── package.xml
└── README.md
```

The `launch/` folder holds most of the functionality — use these launch files to start specific subsystems or the whole robot.

---

## Installation

Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10

Quick setup
```bash
cd ~/fbot_ws/src
git clone https://github.com/fbotathome/fbot_bringup 
cd ~/fbot_ws
colcon build --packages-select fbot_bringup
source install/setup.bash
```

---

## Robot body: `robot.launch.py`

The robot (description, base, lasers, IMU, EKF, optional navigation / neck) is started by **one** launch. A task launch in `fbot_behavior` is:

```
robot.launch.py            once, with flags (use_navigation, map_file, use_neck, ...)
manipulator.launch.py      only if the task uses the arm
<skill launches>           camera, vision, speech, ...
```

```bash
ros2 launch fbot_bringup robot.launch.py                                   # base + lasers + IMU + EKF
ros2 launch fbot_bringup robot.launch.py use_navigation:=true map_file:=lab_2026_2.yaml use_neck:=true
ros2 launch fbot_bringup robot.launch.py robot_version:=v2                 # BORIS v2
ros2 launch fbot_bringup manipulator.launch.py                             # the arm (xArm6), next to robot.launch.py
```

| Launch | Starts |
|--------|--------|
| `robot.launch.py` | everything below, selected with `use_*` flags |
| `base.launch.py` | `robot_state_publisher`, `ros2_control` (hoverboard), diff-drive controller. Run alone for a drive test |
| `sensors.launch.py` | two Hokuyo lasers (`/scan2`, `/scan3`), BNO055 IMU, optional Sick (`/scan`, `sick.launch.py`) |
| `localization.launch.py` | EKF, owner of `odom -> base_footprint` |
| `navigation.launch.py` | Nav2 (AMCL + map) or SLAM, nav only |
| `neck.launch.py` | neck controller + face |
| `manipulator.launch.py` | arm: MoveIt + driver + `fbot_manipulator` (`arm_type` xarm6 / wx200, `xarm_fake`, `robot_ip`), attached to `arm_mount_link` (`mount_xyz`, `mount_rpy`). Included by tasks, not by `robot.launch.py` |

Model, geometry and parameter locations are described in `fbot_description/README.md`. The old `description.launch.py` and `interbotix_arm.launch.py` were removed: use `robot.launch.py` (+ `manipulator.launch.py` for the arm).

---

## Usage

Common subsystem launches:
```bash
ros2 launch fbot_bringup vision.launch.py
ros2 launch fbot_bringup camera.launch.py use_realsense:=true
ros2 launch fbot_bringup world.launch.py
ros2 launch fbot_bringup face_recognition.launch.py
ros2 launch fbot_bringup hotword_detector.launch.py
ros2 launch fbot_bringup riva_speech_to_text.launch.py
ros2 launch fbot_bringup synthesizer_speech.launch.py
ros2 launch fbot_bringup manipulator.launch.py arm_type:=wx200   # Interbotix arm (option)
```

Notes
- Many launch files accept arguments (namespaces, enable flags). Inspect the top of each `launch/*.launch.py` for available DeclareLaunchArgument entries.
- `camera.launch.py` uses a runtime check (OpaqueFunction) to validate that at least one camera source is enabled.

---

## Development

1. Create a branch: (`git checkout -b feat/my-launchfile`)
2. Implement and test changes (launchers live under `launch/`)
4. Commit changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feat/amazing-feature`)
6. Open a PR against `main`

---

## Useful links

- ROS 2 Humble: https://docs.ros.org/en/humble/index.html
- Writing launch files (ROS 2): https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html