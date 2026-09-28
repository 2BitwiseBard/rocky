# ros2/: ROS 2 Jazzy workspace packages (D009)

Scaffolded and parity-checked, never built here. `colcon build` needs
ROS 2 Jazzy (Ubuntu 24.04, or a Pi 5). CI and the dev machine have no ROS,
so this is what is verified without it:

- `rocky_description/urdf/` is **generated from `cad/params.yaml`**
  (`generate_urdf.py`; CI fails if the committed URDF is stale). It passes
  `sim/check_urdf_parity.py`: the 20 joints match the MJCF (axes and
  ranges), FK agrees with the MJCF to 0.05 mm and with the gait engine's
  closed-form FK to 1e-4 mm, and the total mass is 2.694 kg on both.
  **Regenerate after every params change**; never hand-edit the URDF.
- `hw_bridge_node.py` and `gait_node.py` import the pip-installed core
  (`pip install -e .` at the repo root), the same tested code that runs the
  bench and the sim. The bus I/O is the Python driver (79 tests on the
  byte-level mock, [driver/README.md](../driver/README.md)).
- The C++ `RockySystem` skeleton was compiled against stub ROS headers
  (a one-off check, not in CI).

## Packages

| package | type | contents |
|---|---|---|
| `rocky_description` | ament_cmake | URDF/xacro (+ generator), display launch |
| `rocky_msgs` | rosidl | GaitCommand, BodyPoseCommand, HandCommand, ContactState, ServoHealth |
| `rocky_gait` | ament_python | gait node wrapping `pebble_gait` (Twist-compatible) |
| `rocky_control` | ament_cmake | controllers.yaml + bringup launch (`hw` argument) |
| `rocky_driver` | ament_cmake | Python HW bridge (`hw:=topic`) + C++ SystemInterface skeleton (`hw:=serial`) |

**Message semantics worth knowing:**
- `BodyPoseCommand.height_m` is an *offset* from params `gait.body_height`
  (0 = default). `gait_node` rebuilds the gait when it changes. The lean,
  roll, pitch and yaw fields are reserved and not applied yet.
- `ServoHealth.fault_bits` are the servos' status error bits
  (`rocky_driver.protocol.ERROR_BITS`), and are 0 on the SCS0009.
  `current_a` is NaN on the SCS0009, which has no current sense.

## Hardware backends

```
hw:=mock    ros2_control GenericSystem: RViz / kinematic testing, zero deps
hw:=topic   topic_based_ros2_control <-> hw_bridge_node.py <-> rocky_driver
            (recommended while bringing up hardware: all bus I/O stays in
             the tested Python driver; C++ never touches a byte)
hw:=serial  rocky_driver/RockySystem C++ plugin: a skeleton today
```

`RockySystem` takes each servo id from the joint NAME, as params `bus:`
lays it out. `yaw<i>`, `hip<i>` and `knee<i>` map to 3i+1…3i+3, and
`claw<i>` to 16+i. The ros2_control block interleaves the claws, so the
index would be wrong. An unknown name makes `on_init` fail. The state
interfaces are exported exactly as the xacro declares them: position,
velocity and effort for the leg joints, position only for the claws.

The serial I/O is still TODO: port it from `driver/rocky_driver/protocol.py`
when the control loop needs to shed the topic hop (at 50 Hz and 20 servos
it doesn't yet). `on_activate` must follow the D052 V2 soft-enable order:
1. read the present positions;
2. set TORQUE_LIMIT 400 and ACC 10;
3. park the goal at the present position;
4. switch torque on;
5. release the limits once the first move has landed.

**D052 V2: what the ROS path has and doesn't have yet** (the cockpit's
`sim/hw_bridge.py` is the reference).

`hw_bridge_node.py`:
- streams through `rocky_driver.SoftStream`: the goal is parked where each
  servo is, at 40 % torque, before enable; then a smoothstep entry at
  200 cps, NaN hold and a 4.7 rad/s clamp;
- ignores a joint command that misses a leg joint;
- publishes the real status error bits;
- publishes NO contacts until the switches are wired.

`gait_node.py`:
- takes its gait from `cad/params.yaml`;
- fits every `/cmd_vel` into `WaveGait.budget()`, with the same rate clamp;
- turns a non-finite velocity command into a stop and holds the last good
  joint targets when one is not finite.

Still missing: the reflex supervisor (no IMU or contact inputs on this
path), the sim2real heartbeat, the leg-level fault cut and the
degraded-leg re-entry. Neither node has run under ROS yet, so treat
`hw:=topic` as bench-only: one leg, with a hand on the power switch.

## First bringup (any Ubuntu 24.04 + Jazzy machine)

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros2-control \
     ros-jazzy-ros2-controllers ros-jazzy-topic-based-ros2-control \
     ros-jazzy-xacro
pip install -e ".[hw]"                      # from the repo root: pebble_gait + rocky_driver + pyserial
mkdir -p ~/rocky_ws/src && ln -s "$(pwd)"/ros2/* ~/rocky_ws/src/
cd ~/rocky_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch rocky_description display.launch.py        # RViz + sliders
ros2 launch rocky_control control.launch.py hw:=mock   # full control stack
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.045}}"                            # it walks (kinematically)
# hardware day:
ros2 launch rocky_control control.launch.py hw:=topic
```

## Known TODOs

- `RockySystem` serial I/O (above).
- Claw joint state read-back in `hw_bridge_node` (needs the hand v0.3
  calibration story).
- The lean / roll / pitch / yaw fields of `BodyPoseCommand` in `gait_node`.
- A `rocky_sim` package (MuJoCo ↔ ROS bridge). Sim work runs directly in
  `sim/` without ROS, which is faster for iteration.
- `rocky_teleop`: a gamepad → `joy` remap.
- An IMU (BNO085) node, and `ContactState` from the real microswitches.
