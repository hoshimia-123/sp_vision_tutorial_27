# ROS 2 Navigation Project — Progress and Debugging Report

## 1. Project Overview

- Repository: `sp_vision_tutorial_27`
- Current branch: `final_project_nav`
- Environment: Ubuntu VM, ROS 2 Humble, VS Code
- Fork remote (`origin`): `https://github.com/hoshimia-123/sp_vision_tutorial_27.git`
- Upstream remote: `https://github.com/TongjiSuperPower/sp_vision_tutorial_27.git`

## 2. Current Status

The workspace has been built successfully before, and the simulation and major ROS 2 nodes can start. However, end-to-end autonomous navigation has not been verified successfully.

Observed issue: the robot can move in the simulation, but it has previously become stuck at the first maze corner. Manual mouse control in the simulator sometimes moves the robot, but this does not establish that autonomous navigation works.

## 3. Files Modified

The current Git status reported these modified files:

- `src/sp_controller_server/plugins/pid_controller.cpp`
- `src/sp_controller_server/plugins/pid_controller.hpp`
- `src/sp_nav_bringup/config/nav_params.yaml`

Untracked directories also exist: `lecture2/`, `lecture3/`, and `lecture4/`. These are not intended to be included in the navigation diagnostic commit unless they are later confirmed relevant.

## 4. Build History

- The workspace previously built successfully with 10 packages.
- `sp_controller_server` also built successfully after fixing a compile error caused by an unnamed `pose` parameter.
- A successful build does not prove that autonomous navigation works correctly at runtime.

## 5. Relevant Controller Parameters

The recorded PID controller defaults include:

- `linear_kp`: 0.8
- `linear_ki`: 0.0
- `linear_kd`: 0.0
- `angular_kp`: 2.0
- `angular_ki`: 0.0
- `angular_kd`: 0.0
- `max_linear_speed`: 0.5
- `max_angular_speed`: 1.5
- `lookahead_distance`: 0.25
- `goal_tolerance`: 0.15

These values were also entered in `src/sp_nav_bringup/config/nav_params.yaml`. Runtime-loaded values have not yet been independently verified.

## 6. Runtime Observations

The following nodes were visible in `ros2 node list`:

- `/controller_server`
- `/esdf_map_publisher`
- `/nav_interface_node`
- `/planner_server`
- `/rviz2`
- `/sim_robot_node`
- `/sp_decision` (appeared three times)

ROS 2 warned that nodes share an exact name. The cause and impact have not been investigated.

### Global path topic

Command:

`ros2 topic info /global_path`

Observed result:

- Type: `nav_msgs/msg/Path`
- Publisher count: 1
- Subscription count: 2

The command `ros2 topic echo /global_path --once` was previously left waiting without visible output. It remains unconfirmed whether a fresh RViz goal reliably triggers a valid path publication.

### Velocity command topic

Earlier topic inspection showed:

- Publisher: `controller_server`
- Subscriber: `sim_robot_node`
- Topic: `/sentry/cmd_vel`
- Message type: `geometry_msgs/msg/Twist`

The existence of these endpoints does not establish that valid velocity commands are continuously published or that the robot tracks the planned path correctly.

## 7. Current Debugging Questions

Please inspect the actual source code and identify likely causes before recommending parameter changes.

1. Does `PidController::computeVelocityCommands()` correctly track a `nav_msgs/msg/Path` through sharp maze corners?
2. Is the controller using the robot pose and coordinate frames consistently?
3. Are target selection, lookahead distance, heading error, goal tolerance, and velocity limiting implemented correctly?
4. Does the planner produce a valid path after an RViz `2D Goal Pose` is sent?
5. Could duplicate `/sp_decision` nodes or behavior-tree configuration affect goal handling?
6. What exact commands and observations would distinguish a planning failure from a controller failure?

## 8. Requested Diagnosis

Please:

1. Review the three modified source/configuration files and relevant surrounding code.
2. Separate confirmed defects from hypotheses.
3. Recommend the smallest evidence-based changes first.
4. Provide build and runtime verification commands for ROS 2 Humble.
5. Avoid assuming that the problem is caused by PID gains before checking path publication and velocity commands.

## 9. Known Limitations

This report summarizes observations recorded during debugging. It does not include a fresh source-code review, complete runtime logs, or a verified end-to-end navigation test. Conclusions should be validated against the repository and new diagnostic output.
