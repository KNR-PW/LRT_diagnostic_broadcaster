>ros2_control broadcaster that publishes selected hardware interfaces for diagnostics

# [Diagnostic broadcaster](https://control.ros.org/humble/doc/ros2_controllers/doc/controllers_index.html#broadcasters)

Problem has been described in https://github.com/KNR-PW/LRT_meldog_ros/issues/15

## Operating system
https://releases.ubuntu.com/jammy/

## Dependencies (all for humble)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
- [generate_parameters_library](https://github.com/PickNikRobotics/generate_parameter_library)
## Installation
- ```git clone https://github.com/KNR-PW/LRT_diagnostic_broadcaster.git```
- ```colcon build```

## Selecting interfaces
> [!IMPORTANT]
> You need to put your interfaces and joints in ``/include/diagnostic_broadcaster/diagnostic_broadcaster.hpp`` in their respective
> string arrays ```joint_names_``` and ```interface_names```

> [!WARNING]
> For new interfaces you need to update ``diagnostic_msgs/diagnotics.msg`` by adding new variable and ``diagnostic_broadcaster/src/diagnostic_broadcaster.cpp`` where you add new line in ```init_joint_data()``` function (read the comments for more information).

> [!NOTE]
> For ```joint_names_ = {"joint1", "joint2"} interface_names = {temperature}``` broadcasted will be interfaces ``"joint1/temperature"`` and ``"joint2/temperature"``

## Parameters 
This broadcaster uses [generate_parameters_library](https://github.com/PickNikRobotics/generate_parameter_library) to handle parameters. You can specify which joints and interfaces you want to be broadcasted.

> [!NOTE]
> By specifying which joints should be broadcasted ```state_interfaces_config.type``` changes from ```controller_interface::interface_configuration_type::ALL``` to ```controller_interface::interface_configuration_type::INDIVIDUAL;```. 

```update_treshold``` parameter is currently used for setting up data update threshold for ```temperature``` and ```motor_effort``` interfaces


``diagnostic_broadcaster/diagnostic_parameters.yaml``:
```rb
diagnostic_broadcaster:
  joint_names:
    type: string_array
    read_only: true
    default_value: []
    validation:
      unique<>: null
    description: "Names of joints"
    

  interface_names:
    type: string_array
    description: "Names of diagnostic interfaces"
    default_value: ["temperature", "fault", "motor_effort"]
    read_only: true
    validation:
      subset_of<>: [["temperature", "fault", "motor_effort"]]
      unique<>: null
      not_empty<>: null

  interface_params:
    __map_interface_names:
      update_threshold:
        type: double
        default_value: 0.1
        read_only: true
        description: "When interface values change by this amount, it will be noticed"

```

