>C++ program that broadcasts selected robot interfaces' data for diagnostics

# [Diagnostic broadcaster](https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html#broadcasters)

Problem has been described in https://github.com/KNR-PW/LRT_meldog_ros/issues/15

## Operating system
https://releases.ubuntu.com/jammy/

## Dependencies (all for humble)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)

## Installation
- ```git clone https://github.com/KNR-PW/LRT_diagnostic_broadcaster.git```
- ```colcon build```

## Selecting interfaces
> [!IMPORTANT]
> You need to put your interfaces and joints in ``/include/diagnostic_broadcaster/diagnostic_broadcaster.hpp`` in their respective
> string arrays ```joint_names_``` and ```interface_names```

> [!NOTE]
> For ```joint_names_ = {"joint1", "joint2"} interface_names = {temperature}``` broadcasted will be interfaces ``"joint1/temperature"`` and ``"joint2/temperature"``
