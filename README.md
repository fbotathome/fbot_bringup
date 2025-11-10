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

## Usage

Examples assume workspace sourced.

Launch camera (Realsense optional):
```bash
# default use_realsense is false; enable with argument
ros2 launch fbot_bringup camera.launch.py use_realsense:=true
```

Common subsystem launches:
```bash
ros2 launch fbot_bringup vision.launch.py
ros2 launch fbot_bringup world.launch.py
ros2 launch fbot_bringup face_recognition.launch.py
ros2 launch fbot_bringup hotword_detector.launch.py
ros2 launch fbot_bringup riva_speech_to_text.launch.py
ros2 launch fbot_bringup synthesizer_speech.launch.py
ros2 launch fbot_bringup interbotix_arm.launch.py
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