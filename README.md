# CBR 2025 - Flying Robot League Autonomous Drone

This repository contains the autonomous navigation system developed by [Black Bee Drones](https://github.com/Black-Bee-Drones) for the Flying Robot League (FRL) category of the Brazilian Robotics Competition (CBR) 2025.

## Competition Overview

The CBR 2025 Flying Robot League consists of four phases testing autonomous navigation, object manipulation, human-robot interaction, and confined space exploration. All missions operate in an indoor environment without GPS, relying on visual odometry and computer vision.

## Technical Stack

### Hardware
- **Flight Controller**: [Pixhawk 2.4.8](https://docs.px4.io/main/en/flight_controller/pixhawk-2.html) with [ArduPilot](https://ardupilot.org/)
- **Onboard Computer**: [Jetson Orin Nano Super](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
- **Cameras**:
  - [Intel RealSense D435i](https://www.realsenseai.com/products/depth-camera-d435i/) (front-facing, 640×480)
  - IMX219 (down-facing, 1640×1232)
- **Navigation**: [NVIDIA Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
- **Additional Sensors**: Lidar rangefinder for altitude measurement

### Software
- **[ROS 2](https://docs.ros.org/)**: Middleware for system communication
- **[MAVROS](https://github.com/mavlink/mavros)**: Bridge between ROS 2 and ArduPilot via MAVLink
- **[Nectar SDK v0.1.0](https://github.com/Black-Bee-Drones/nectar-sdk/releases/tag/v0.1.0)** (formerly `mirela_sdk`): Unified interface for drone control, computer vision, and AI detection
- **[Yasmin](https://github.com/uleroboticsgroup/yasmin)**: State machine framework for mission orchestration
- **[OpenCV](https://opencv.org/)**: Image processing and computer vision
- **[Ultralytics YOLO](https://docs.ultralytics.com/)**: Object detection models (YOLOv11n, YOLOv11n-pose)

## Missions

### Phase 1: Localization and Mapping

**Objective**: Explore an 8×8 m arena, detect and land on six landing bases (1×1 m plates at heights 0-1.5 m), and return to takeoff position.

**Key Features**:
- Boustrophedon grid coverage pattern (5×5 m search area)
- YOLO-based landing base detection with TensorRT optimization
- Multi-detection processing with duplicate prevention
- Two-phase landing approach: centering at 2.4 m, then controlled descent to 1.2 m

**Documentation**: [mapping/README.md](mapping/README.md)

### Phase 2: Package Transport

**Objective**: Transport three first-aid kits from initial positions to three empty landing bases using a servo-controlled gripper.

**Key Features**:
- Visual servoing for package pickup with proportional control
- Package orientation alignment using aspect ratio detection
- Gripper control via PWM signals (AUX OUT 4)
- Pickup and dropoff state machines with recovery mechanisms

**Documentation**: [delivery/README.md](delivery/README.md)

### Phase 3: Human-Robot Interaction

**Objective**: Control drone movement through body pose recognition, enabling gesture-based navigation to land on all six bases.

**Key Features**:
- YOLO11n-pose model for real-time human pose estimation
- Custom gesture recognition algorithm with keypoint analysis
- Debouncing mechanism (2 frames for single actions, 8 frames for continuous movements)
- Direct gesture-to-velocity command mapping

**Documentation**: [interaction/README.md](interaction/README.md)

### Phase 4: Confined Space Navigation

**Objective**: Navigate through a dark, enclosed maze (1.5 m height), detect QR codes on walls, and exit to land on a designated base.

**Key Features**:
- Grid-based exploration with 1 m × 1 m sections
- Two-height scanning strategy (0.8 m and 0.3 m)
- QR code detection using qreader library with YOLOv8 backend
- WebSocket interface for ESP-attached lidar sensor
- Hardcoded waypoint path for navigation

**Documentation**: [maze/README.md](maze/README.md)

## File Structure

```
cbr-2025/
├── mapping/          # Phase 1: Localization and mapping
│   ├── mapping/      # State machine and navigation logic
│   └── README.md     # Detailed documentation
├── delivery/         # Phase 2: Package transport
│   ├── delivery/     # Pickup/dropoff state machines
│   ├── models/       # YOLO detection models
│   └── README.md     # Detailed documentation
├── interaction/      # Phase 3: Human-robot interaction
│   ├── interaction/  # Gesture recognition and control
│   └── README.md     # Detailed documentation
├── maze/            # Phase 4: Maze navigation
│   ├── maze/        # QR detection and navigation
│   └── README.md    # Detailed documentation
└── README.md        # This file
```

## Running Missions

Each phase is implemented as a separate ROS 2 package. Refer to individual module READMEs for execution instructions:

- **Phase 1**: `ros2 run mapping mangalarga`
- **Phase 2**: `ros2 run delivery mangalarga`
- **Phase 3**: `ros2 run interaction sm_interaction`
- **Phase 4**: `ros2 run maze mangalarga`

## References

### Competition & Organization
- [CBR 2025 - Flying Robot League](https://cbr.robocup.org.br/)

### Software & Frameworks
- [Nectar SDK v0.1.0](https://github.com/Black-Bee-Drones/nectar-sdk/releases/tag/v0.1.0)
- [ROS 2 Documentation](https://docs.ros.org/)
- [MAVROS](https://github.com/mavlink/mavros)
- [Yasmin State Machine](https://github.com/uleroboticsgroup/yasmin)
- [NVIDIA Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)

## Team

**Black Bee Drones** - Latin America's first academic autonomous drone team  
Federal University of Itajubá (UNIFEI), Brazil

---

*This documentation describes the technical implementation used during CBR 2025. For questions or contributions, please refer to the [Black Bee Drones GitHub organization](https://github.com/Black-Bee-Drones).*
