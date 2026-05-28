# OpenArm ROS 2 Packages

This folder contains the source packages for the OpenArm ROS 2 workspace.

Before running any package from a new terminal, source ROS 2 and this workspace:

```bash
cd ~/OPEN_ARM/packages
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

To see the arguments accepted by any launch file:

```bash
ros2 launch <package_name> <launch_file.py> --show-args
```

## Package Overview

| Package | Purpose | Launch files |
| --- | --- | --- |
| `openarm_bringup` | Starts robot description, `ros2_control`, controllers, grippers, and RViz. | `openarm.launch.py`, `openarm.bimanual.launch.py` |
| `openarm_description` | URDF/Xacro, meshes, RViz configs, and visualization-only launch. | `display_openarm.launch.py` |
| `openarm_bimanual_moveit_config` | MoveIt config for bimanual OpenArm. | `demo.launch.py`, `move_group.launch.py`, `moveit_rviz.launch.py`, `spawn_controllers.launch.py`, `static_virtual_joint_tfs.launch.py`, `setup_assistant.launch.py` |
| `openarm_quest_teleop` | Quest 3 teleoperation support using MoveIt Servo and a WebSocket bridge. | `bimanual_teleop.launch.py` |
| `openarm_can` | CAN support code. | No launch files found here. |
| `openarm` | Core OpenArm package under `openarm_ros2`. | No launch files found here. |
| `openarm_hardware` | Real OpenArm `ros2_control` hardware plugin. | No launch files found here. |

## Launch Files

### Single Arm Bringup

Source:

```text
openarm_ros2/openarm_bringup/launch/openarm.launch.py
```

Run with real hardware:

```bash
ros2 launch openarm_bringup openarm.launch.py
```

Run with fake hardware:

```bash
ros2 launch openarm_bringup openarm.launch.py use_fake_hardware:=true
```

Run with a specific CAN interface:

```bash
ros2 launch openarm_bringup openarm.launch.py can_interface:=can0
```

Run with the forward position controller:

```bash
ros2 launch openarm_bringup openarm.launch.py robot_controller:=forward_position_controller
```

Common arguments:

| Argument | Default | Notes |
| --- | --- | --- |
| `description_package` | `openarm_description` | Package containing the robot description. |
| `description_file` | `v10.urdf.xacro` | URDF/Xacro file. |
| `arm_type` | `v10` | Arm version. |
| `use_fake_hardware` | `false` | Set `true` for mock hardware. |
| `robot_controller` | `joint_trajectory_controller` | Also supports `forward_position_controller`. |
| `arm_prefix` | empty | Prefix for single-arm joint/topic naming. |
| `can_interface` | `can0` | CAN interface for real hardware. |
| `controllers_file` | `openarm_v10_controllers.yaml` | Controller config file. |

This launches:

- `robot_state_publisher`
- `controller_manager` / `ros2_control_node`
- `joint_state_broadcaster`
- selected arm controller
- `gripper_controller`
- RViz with `arm_only.rviz`

### Bimanual Bringup

Source:

```text
openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py
```

Run with real hardware:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

Run with fake hardware:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py use_fake_hardware:=true
```

Run with explicit CAN interfaces:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py right_can_interface:=can0 left_can_interface:=can1
```

Run with namespacing:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_prefix:=robot1
```

Common arguments:

| Argument | Default | Notes |
| --- | --- | --- |
| `description_package` | `openarm_description` | Package containing the robot description. |
| `description_file` | `v10.urdf.xacro` | URDF/Xacro file. |
| `arm_type` | `v10` | Arm version. |
| `use_fake_hardware` | `false` | Set `true` for mock hardware. |
| `robot_controller` | `joint_trajectory_controller` | Also supports `forward_position_controller`. |
| `arm_prefix` | empty | Optional namespace. |
| `right_can_interface` | `can0` | Right arm CAN interface. |
| `left_can_interface` | `can1` | Left arm CAN interface. |
| `controllers_file` | `openarm_v10_bimanual_controllers.yaml` | Controller config file. |

This launches:

- bimanual `robot_state_publisher`
- bimanual `controller_manager` / `ros2_control_node`
- `joint_state_broadcaster`
- left and right arm controllers
- left and right gripper controllers
- RViz with `bimanual.rviz`

### Visualization Only

Source:

```text
openarm_description/launch/display_openarm.launch.py
```

Single arm visualization:

```bash
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10
```

Bimanual visualization:

```bash
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=true
```

Without hand/end-effector:

```bash
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 ee_type:=none
```

Arguments:

| Argument | Default | Notes |
| --- | --- | --- |
| `arm_type` | required | Example: `v10`. |
| `ee_type` | `openarm_hand` | Use `none` for no end-effector. |
| `bimanual` | `false` | Set `true` for dual-arm visualization. |

This launches:

- `robot_state_publisher`
- `joint_state_publisher_gui`
- RViz

Use this when you only want to inspect the model and move joints manually in the GUI.

### Bimanual MoveIt Demo

Source:

```text
openarm_ros2/openarm_bimanual_moveit_config/launch/demo.launch.py
```

Recommended fake-hardware test:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

Real hardware:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py right_can_interface:=can0 left_can_interface:=can1
```

Forward position controller:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py robot_controller:=forward_position_controller
```

Common arguments:

| Argument | Default | Notes |
| --- | --- | --- |
| `description_package` | `openarm_description` | Package containing the robot description. |
| `description_file` | `v10.urdf.xacro` | URDF/Xacro file. |
| `arm_type` | `v10` | Arm version. |
| `use_fake_hardware` | `false` | Set `true` for mock hardware. |
| `robot_controller` | `joint_trajectory_controller` | Also supports `forward_position_controller`. |
| `right_can_interface` | `can0` | Right arm CAN interface. |
| `left_can_interface` | `can1` | Left arm CAN interface. |
| `controllers_file` | `openarm_v10_bimanual_controllers.yaml` | Controller config file. |

This launches:

- bimanual robot description
- `ros2_control`
- joint, arm, and gripper controllers
- `move_group`
- MoveIt RViz

Use this as the main starting point for bimanual MoveIt testing.

### MoveIt Helper Launch Files

These files are generated-style helpers from `moveit_configs_utils`.

Start only MoveIt `move_group`:

```bash
ros2 launch openarm_bimanual_moveit_config move_group.launch.py
```

Start only MoveIt RViz:

```bash
ros2 launch openarm_bimanual_moveit_config moveit_rviz.launch.py
```

Spawn controllers from the MoveIt config:

```bash
ros2 launch openarm_bimanual_moveit_config spawn_controllers.launch.py
```

Publish static virtual joint transforms:

```bash
ros2 launch openarm_bimanual_moveit_config static_virtual_joint_tfs.launch.py
```

Open the MoveIt Setup Assistant:

```bash
ros2 launch openarm_bimanual_moveit_config setup_assistant.launch.py
```

For normal operation, prefer `demo.launch.py`; use these only when you are debugging or editing the MoveIt setup.

### Quest 3 Bimanual Teleop

Source:

```text
openarm_quest_teleop/launch/bimanual_teleop.launch.py
```

Important: `bimanual_teleop.launch.py` expects `/robot_state_publisher` to already be running. Start `demo.launch.py` first.

Terminal 1, start the robot and MoveIt:

```bash
cd ~/OPEN_ARM/packages
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

Terminal 2, start MoveIt Servo for both arms:

```bash
cd ~/OPEN_ARM/packages
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openarm_quest_teleop bimanual_teleop.launch.py
```

Terminal 3, start the Quest bridge node:

```bash
cd ~/OPEN_ARM/packages
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run openarm_quest_teleop quest_bridge --ros-args -p ws_port:=9090
```

The bridge publishes:

| Topic | Purpose |
| --- | --- |
| `/left/servo_node/delta_twist_cmds` | Left arm Cartesian twist commands. |
| `/right/servo_node/delta_twist_cmds` | Right arm Cartesian twist commands. |
| `/left_gripper_position` | Left gripper position command. |
| `/right_gripper_position` | Right gripper position command. |

Default bridge parameters:

| Parameter | Default |
| --- | --- |
| `ws_port` | `9090` |
| `position_scale` | `1.0` |
| `deadman_button` | `grip` |
| `deadman_threshold` | `0.5` |
| `max_linear_vel` | `0.4` |
| `max_angular_vel` | `0.8` |
| `gripper_open` | `0.044` |
| `gripper_closed` | `0.0` |

## Recommended Workflows

### Check That ROS Sees the Packages

```bash
cd ~/OPEN_ARM/packages
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg list | grep openarm
```

### Just View the Robot

```bash
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=true
```

### Test Bimanual MoveIt Without Real Hardware

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

### Run Real Bimanual Hardware

Make sure the CAN interfaces exist first:

```bash
ip link show can0
ip link show can1
```

Then launch:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py right_can_interface:=can0 left_can_interface:=can1
```

### Run Quest Teleop With Fake Hardware

Terminal 1:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

Terminal 2:

```bash
ros2 launch openarm_quest_teleop bimanual_teleop.launch.py
```

Terminal 3:

```bash
ros2 run openarm_quest_teleop quest_bridge --ros-args -p ws_port:=9090
```

## Notes

- The current launch files use `use_fake_hardware:=true` or `use_fake_hardware:=false`.
- Some older README text mentions `hardware_type`, but that is not an argument in the current launch files checked here.
- `openarm_quest_teleop bimanual_teleop.launch.py` cannot even print `--show-args` unless `/robot_state_publisher` is already running, because it reads the `robot_description` parameter during launch-file loading.
- The installed launch files are under `~/OPEN_ARM/packages/install/...`, but the source files are under `~/OPEN_ARM/packages/src/...`.
- If you edit a source launch file, rebuild and source again:

```bash
cd ~/OPEN_ARM/packages
colcon build --symlink-install
source install/setup.bash
```

## Troubleshooting

### RViz Shows a Trajectory, but Planning Status Is Failed

Symptom in RViz:

- You select `left_arm` or `right_arm` in the MoveIt MotionPlanning panel.
- Clicking `Plan` or `Plan & Execute` returns `Failed`.
- If `Use Cartesian Path` is enabled, RViz may still draw a trajectory and show messages like `Achieved 100.000000 % of Cartesian path`.
- Even though the path is drawn, the status still becomes `Failed`.

Important log lines:

```text
[move_group-3] [ERROR] ... No acceleration limit was defined for joint openarm_left_joint1! You have to define acceleration limits in the URDF or joint_limits.yaml
[move_group-3] [ERROR] ... Response adapter 'AddTimeOptimalParameterization' failed to generate a trajectory.
[move_group-3] [ERROR] ... PlanningResponseAdapter 'AddTimeOptimalParameterization' failed with error code FAILURE
[rviz2-4] [ERROR] ... MoveGroupInterface::plan() failed or timeout reached
```

For Cartesian path:

```text
[move_group-3] [INFO] ... Computed Cartesian path with 46 points (followed 100.000000% of requested trajectory)
[rviz2-4] [ERROR] ... No acceleration limit was defined for joint openarm_left_joint1!
[rviz2-4] [INFO] ... Computing time stamps FAILED
```

What this means:

MoveIt is able to find a geometric path, especially with Cartesian path enabled. The failure happens after that, when MoveIt tries to add timestamps to the trajectory using the `AddTimeOptimalParameterization` adapter. That adapter needs acceleration limits for every moving joint. In this workspace, the MoveIt config currently disables acceleration limits and sets them to zero.

The file causing this is:

```text
~/OPEN_ARM/packages/src/openarm_ros2/openarm_bimanual_moveit_config/config/joint_limits.yaml
```

Current problematic pattern:

```yaml
has_acceleration_limits: false
max_acceleration: 0.0
```

This appears for the arm joints:

```text
openarm_left_joint1
openarm_left_joint2
openarm_left_joint3
openarm_left_joint4
openarm_left_joint5
openarm_left_joint6
openarm_left_joint7
openarm_right_joint1
openarm_right_joint2
openarm_right_joint3
openarm_right_joint4
openarm_right_joint5
openarm_right_joint6
openarm_right_joint7
```

It also appears for:

```text
openarm_left_finger_joint1
openarm_right_finger_joint1
```

Fix:

Edit `joint_limits.yaml` and give every planned joint a positive acceleration limit.

For a conservative first test, change each arm joint from this:

```yaml
has_acceleration_limits: false
max_acceleration: 0.0
```

to this:

```yaml
has_acceleration_limits: true
max_acceleration: 5.0
```

For the finger joints, use a separate small positive value, for example:

```yaml
has_acceleration_limits: true
max_acceleration: 10.0
```

The exact acceleration values should be tuned for the real robot. For fake hardware and RViz planning tests, the key requirement is that `has_acceleration_limits` is `true` and `max_acceleration` is greater than `0.0`.

After editing, rebuild and source:

```bash
cd ~/OPEN_ARM/packages
colcon build --symlink-install
source install/setup.bash
```

Then restart the MoveIt demo:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

Test again in RViz:

1. Select `left_arm` or `right_arm`.
2. Move the interactive marker.
3. Click `Plan`.
4. If the plan succeeds, click `Execute` or use `Plan & Execute`.

Why Cartesian path looked partly successful:

`Use Cartesian Path` computes waypoints in Cartesian space first. That geometric waypoint generation can succeed, which is why RViz draws the path and reports `100%`. But MoveIt still needs to timestamp the final trajectory before execution. Because the acceleration limits were missing, timestamp generation failed, so the overall request was marked failed.

The `left_gripper` message is separate:

```text
No active joints or end effectors found for group 'left_gripper'. Make sure that kinematics.yaml is loaded in this node's namespace.
```

This is not the main reason `left_arm` planning failed. The main blocking error is the missing acceleration limit for `openarm_left_joint1`. The gripper warning means the gripper planning group does not have active joints or kinematics configured the way the RViz MotionPlanning plugin expects. Focus on fixing the arm acceleration limits first.

Useful check after restarting:

```bash
ros2 param get /move_group robot_description_planning
```

If planning still fails, check whether the running install copy has the updated limits:

```bash
grep -n "openarm_left_joint1" -A4 ~/OPEN_ARM/packages/install/openarm_bimanual_moveit_config/share/openarm_bimanual_moveit_config/config/joint_limits.yaml
```

Expected output should contain:

```yaml
has_acceleration_limits: true
max_acceleration: 5.0
```

### Error: Zero Acceleration and Velocity

Symptom after trying to fix acceleration limits:

```text
[move_group-3] [ERROR] ... Error while integrating forward: zero acceleration and velocity. Are any relevant acceleration components limited to zero?
[move_group-3] [ERROR] ... Trajectory not valid after integrateForward and integrateBackward.
[move_group-3] [ERROR] ... Couldn't create trajectory
[rviz2-4] [INFO] ... Computing time stamps FAILED
```

What this means:

MoveIt no longer says the acceleration limit is missing, but the time-parameterization step still sees at least one relevant joint with a usable limit of `0.0`. A common cause is duplicate YAML keys.

Bad YAML example:

```yaml
openarm_left_joint2:
  has_velocity_limits: true
  max_velocity: 16.754666
  has_acceleration_limits: true
  max_acceleration: 5.0
  max_acceleration: 0.0
```

In YAML, duplicate keys are not additive. The later value wins, so MoveIt effectively reads:

```yaml
max_acceleration: 0.0
```

Fix:

Make sure each joint has only one `max_acceleration` key, and that it is positive:

```yaml
openarm_left_joint2:
  has_velocity_limits: true
  max_velocity: 16.754666
  has_acceleration_limits: true
  max_acceleration: 5.0
```

The cleaned file should have no matches for zero acceleration:

```bash
grep -n "max_acceleration: 0.0" ~/OPEN_ARM/packages/src/openarm_ros2/openarm_bimanual_moveit_config/config/joint_limits.yaml
```

No output is expected.

Check that the installed package sees the same file:

```bash
ls -l ~/OPEN_ARM/packages/install/openarm_bimanual_moveit_config/share/openarm_bimanual_moveit_config/config/joint_limits.yaml
```

With `--symlink-install`, this should point back to:

```text
~/OPEN_ARM/packages/src/openarm_ros2/openarm_bimanual_moveit_config/config/joint_limits.yaml
```

Then fully stop and restart the MoveIt launch. A running `move_group` will not reload this YAML automatically.

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

You can confirm what the currently running `move_group` has loaded with:

```bash
ros2 param get /move_group robot_description_planning.joint_limits.openarm_left_joint2.max_acceleration
```

If it prints:

```text
Double value is: 0.0
```

then the running process still has stale or bad limits. Stop the old launch with `Ctrl+C`, make sure no duplicate `move_group`/RViz launches are still running, and start the demo again.

### Plan Succeeds, but Separate Execute Fails

Symptom:

```text
[move_group-3] [INFO] ... Motion plan was computed successfully.
[rviz2-4] [INFO] ... Planning request complete!
[move_group-3] [INFO] ... Validating trajectory with allowed_start_tolerance 0.01
[move_group-3] [ERROR] ... Invalid Trajectory: start point deviates from current robot state more than 0.01 at joint 'openarm_left_joint1'.
[move_group-3] [INFO] ... Execution completed: ABORTED
[rviz2-4] [ERROR] ... MoveGroupInterface::execute() failed or timeout reached
```

What this means:

The trajectory itself is valid, but it was planned from an older robot state. When `Execute` is clicked later, MoveIt compares the first waypoint of the saved plan against the current `/joint_states`. If any joint differs by more than `0.01` rad, MoveIt aborts execution to avoid commanding a sudden jump.

This is why `Plan & Execute` can work immediately after separate `Plan` then `Execute` fails:

- `Plan` creates a trajectory from the current state at plan time.
- If the robot state changes before `Execute`, the saved trajectory start is stale.
- `Plan & Execute` replans and executes in one request, so the trajectory start matches the current state.

Common causes:

- Waiting too long between `Plan` and `Execute`.
- Moving the robot, interactive marker, or joint state after planning.
- Running fake hardware/controllers that update the current state after the plan was made.
- Multiple RViz or `move_group` instances running at the same time.
- `/joint_states` not matching the start state stored in the planned trajectory.

Recommended workflow in RViz:

1. Use `Plan & Execute` when testing motion.
2. If using separate `Plan` and `Execute`, click `Execute` immediately after `Plan`.
3. If `Execute` aborts, click `Plan` again, then execute the new plan.
4. Keep `Start State` as `<current>` unless intentionally testing a stored start state.
5. Avoid dragging markers or changing states between planning and execution.

Useful checks:

```bash
ros2 topic echo /joint_states --once
```

Check for duplicate nodes:

```bash
ros2 node list | grep -E "move_group|rviz2"
```

If duplicates exist, stop old launch terminals with `Ctrl+C` and restart only one demo launch:

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py use_fake_hardware:=true
```

The `allowed_start_tolerance 0.01` value is a safety check. Increasing it can hide this warning, but it can also allow execution from a state that does not match the plan. For real hardware, prefer replanning from the current state instead of relaxing the tolerance.

## MoveIt RViz Planning Options

The checkboxes in RViz's MotionPlanning panel do not all change the planner in the same way. Some affect path generation, some affect IK goal creation, and some are for external control.

### Use Cartesian Path

Meaning:

MoveIt tries to move the end effector in a straight line in Cartesian space.

Instead of asking OMPL to search freely through joint space, MoveIt interpolates small end-effector waypoints between the current pose and the target pose, then solves IK for those waypoints.

Typical log:

```text
Received request to compute Cartesian path
Attempting to follow 1 waypoints for link 'openarm_left_link7'
Computed Cartesian path with 49 points (followed 100.000000% of requested trajectory)
```

Use it for:

- Short, straight hand motions.
- Debugging whether IK and trajectory execution work.
- Motions where you care about the hand moving directly from A to B.

Limitations:

- It is not a full global planner.
- It may fail for large moves.
- It may fail near singularities.
- It may fail if IK jumps between solutions.
- It may fail if the straight-line hand path goes through collision.

Why this currently works best on this robot:

For the OpenArm fake-hardware RViz test, Cartesian path can be easier than full OMPL planning because the target is usually a small marker movement. It bypasses some of the harder sampling-planner behavior and directly asks: can the hand move straight from here to there?

Recommended setting while debugging:

```text
Use Cartesian Path: ON for small straight moves
```

### Collision-Aware IK

Meaning:

This affects the IK solution used when RViz creates the start or goal state from the interactive marker. It does not replace OMPL, and it is not the same as collision checking the final trajectory.

When dragging the marker:

- Off: IK may accept a joint solution even if robot links are colliding.
- On: IK tries to find a collision-free joint solution for the marker pose.

Use it for:

- Checking whether a marker pose can be reached without self-collision.
- Safer goal-state setup.

Why it may fail:

If the marker pose is reachable only through a colliding configuration, collision-aware IK rejects it. In RViz this can look like the marker or ghost robot refuses to move to the desired pose, or planning produces no useful trajectory.

Recommended setting while debugging:

```text
Collision-aware IK: OFF while debugging reachability
Collision-aware IK: ON when checking safer/collision-free marker poses
```

### Approx IK Solutions

Meaning:

This lets MoveIt accept an approximate IK solution when it cannot solve the exact marker pose or orientation.

When off:

- The IK solver tries to match the requested pose more exactly.

When on:

- The IK solver may accept a nearby joint configuration.
- The end effector may not land exactly on the marker.

Use it for:

- Slightly unreachable marker poses.
- Over-constrained wrist orientations.
- Debugging whether failure is caused by exact IK strictness.

Limitations:

- It does not guarantee planning success.
- It may produce a goal that is visibly different from the marker.
- It can hide the fact that the requested pose is not actually reachable.

Recommended setting while debugging:

```text
Approx IK Solutions: OFF for exact tests
Approx IK Solutions: ON if marker poses are nearly reachable but exact IK fails
```

### External Comm.

Meaning:

This allows external ROS nodes to communicate with and control RViz's MotionPlanning interactive marker interface.

It is mainly used for joystick-style RViz marker control. MoveIt joystick tutorials require enabling `Allow External Comm.` so another node can update the RViz goal marker and trigger plan/execute behavior.

Use it for:

- Joystick control of RViz interactive markers.
- External tools that intentionally drive RViz's MotionPlanning display.

Do not confuse it with MoveIt Servo:

The Quest teleop pipeline in this workspace does not need RViz External Comm. for normal operation because Quest teleop publishes commands to MoveIt Servo topics:

```text
Quest -> quest_bridge -> /left/servo_node/delta_twist_cmds
Quest -> quest_bridge -> /right/servo_node/delta_twist_cmds
```

Recommended setting for this workspace:

```text
External Comm.: OFF for normal MoveIt/RViz planning
External Comm.: ON only if intentionally controlling RViz markers externally
```

### Replanning

Meaning:

This allows MoveIt to retry planning if planning or execution needs another attempt, usually because the scene or robot state changed.

Use it for:

- Dynamic scenes.
- Perception-based planning.
- Cases where obstacles may move.
- Real robot workflows where the environment can change between planning attempts.

Limitations:

- It does not fix bad IK.
- It does not fix missing or bad joint limits.
- It does not fix stale saved plans; for those, replan from `<current>`.
- It may not visibly change much in simple fake-hardware RViz tests.

Recommended setting for this workspace:

```text
Replanning: OFF or ON is fine for simple fake-hardware tests
Use Plan & Execute when you want the freshest start state
```

### Practical Settings for OpenArm Debugging

Good first setup:

```text
Planning Group: left_arm or right_arm
Start State: <current>
Goal State: <current> or interactive marker target
Use Cartesian Path: ON for small straight marker moves
Collision-aware IK: OFF at first
Approx IK Solutions: OFF at first
External Comm.: OFF
Replanning: OFF or ON, not critical
Velocity Scaling: 0.10
Accel Scaling: 0.10
```

If marker IK fails:

```text
Try Approx IK Solutions: ON
Try a smaller marker movement
Try a less strict wrist orientation
Try Collision-aware IK: OFF to see whether collision checking is the blocker
```

If collision safety matters:

```text
Use Collision-aware IK: ON
Keep the goal state out of red/collision visualization
Use Plan & Execute only after checking the path
```

If separate `Plan` works but `Execute` fails:

```text
Use Plan & Execute
or click Execute immediately after Plan
or replan from Start State: <current>
```

### Summary Table

| Option | What it affects | When to use | Why it may fail |
| --- | --- | --- | --- |
| `Use Cartesian Path` | Path-generation method | Straight hand motions | Large moves, collision, IK jumps, singularities |
| `Collision-aware IK` | Marker IK / goal-state generation | Safer marker poses | Desired marker pose only has colliding IK solutions |
| `Approx IK Solutions` | Marker IK tolerance | Nearly reachable poses | Goal may not exactly match marker |
| `External Comm.` | External control of RViz marker UI | Joystick/external RViz marker tools | Does nothing useful by itself |
| `Replanning` | Retry behavior | Dynamic scenes or changing environment | Does not fix IK, limits, or stale trajectory starts |

Sources:

- MoveIt RViz quickstart, including Cartesian path and collision-aware IK behavior: https://moveit.picknik.ai/humble/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html
- MoveIt RViz external communication / joystick marker control: https://docs.ros.org/en/hydro/api/moveit_ros_visualization/html/doc/joystick.html
