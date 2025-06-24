# leandra_atos_interface

The `leandra_atos_interface` package provides a ROS 2 component node that acts as an interface between the ATOS test automation system and the LeanDRA vehicle platform. It manages state transitions, trajectory handling, and communication of localization and status information.

## Features

- ROS 2 component node (`LeandraAtosInterfaceNode`) for integration in composable containers.
- Implements ISO 22133 state machine logic for test object lifecycle.
- Handles ATOS messages (OSEM, STRT, TRAJ) and converts them to internal state and ROS messages.
- Converts geographic coordinates to UTM using GeographicLib.
- Publishes trajectory as `nav_msgs/msg/Path` and velocity profiles.
- Monitors LeanDRA status and blocks state transitions if the platform is not ready.

## Build

This package uses ROS 2 and CMake. Dependencies include `rclcpp`, `nav_msgs`, `geometry_msgs`, `leandra_dbw_msgs`, and `GeographicLib`.

```sh
colcon build --packages-select leandra_atos_interface
```

## Usage

Launch the interface node in a composable container:

```sh
ros2 launch leandra_atos_interface leandra_atos_interface.launch.py
```

Parameters can be set in the launch file, e.g. `ip_address`.

## Main Classes

- [`LeandraAtosInterface::LeandraAtosInterface`](src/leandra_atos_interface/include/leandra_atos_interface/leandra_atos_interface.hpp): Implements the ISO 22133 test object and handles ATOS messages, trajectory, and LeanDRA status.
- [`LeandraAtosInterface::LeandraAtosInterfaceNode`](src/leandra_atos_interface/include/leandra_atos_interface/leandra_atos_interface_node.hpp): ROS 2 node wrapper for the interface.

## Key Files

- [src/leandra_atos_interface/src/leandra_atos_interface.cpp](src/leandra_atos_interface/src/leandra_atos_interface.cpp): Main implementation.
- [src/leandra_atos_interface/include/leandra_atos_interface/leandra_atos_interface.hpp](src/leandra_atos_interface/include/leandra_atos_interface/leandra_atos_interface.hpp): Interface class definition.
- [src/leandra_atos_interface/include/leandra_atos_interface/leandra_atos_interface_node.hpp](src/leandra_atos_interface/include/leandra_atos_interface/leandra_atos_interface_node.hpp): Node class definition.
- [src/leandra_atos_interface/launch/leandra_atos_interface.launch.py](src/leandra_atos_interface/launch/leandra_atos_interface.launch.py): Example launch file.

## State Machine

The interface enforces state transitions according to ISO 22133. For example, arming is only allowed if LeanDRA is ready:

- Disarmed → PreArming (checks LeanDRA)
  - If ready: PreArming → Armed
  - If not ready: PreArming → Abort/Emergency Stop

See [`CustomPreArming`](src/leandra_atos_interface/src/leandra_atos_interface.cpp) for details.

## License

See [LICENSE.md](src/leandra_atos_interface/LICENSE.md).

---

For more details, see the code and comments in