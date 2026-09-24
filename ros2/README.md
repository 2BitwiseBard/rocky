# ros2/ — ROS 2 Jazzy workspace packages (D009)

Scaffolded + parity-verified in the no-hardware sessions; `colcon build`
happens on the Ubuntu 24.04 laptop / Pi 5 (this repo's cloud sessions have no
ROS). What IS verified here and now:

- `rocky_description/urdf/` is **generated from `cad/params.yaml`**
  (`generate_urdf.py`) and passes `sim/check_urdf_parity.py`: 20/20 joints
  match the MJCF (axes + ranges), FK agrees with the MJCF to 0.05 mm and with
  the gait engine's closed-form FK to 1e-4 mm, mass budget matches to 1 g.
  **Regenerate after every params change** — never hand-edit the urdf.
- `hw_bridge_node.py` + `gait_node.py` import the pip-installed core
  (`pip install -e .` at repo root) — the same tested code that runs the
  bench and the sim.

## Packages (plan §5.1)

| package | type | contents |
|---|---|---|
| `rocky_description` | ament_cmake | URDF/xacro (+ generator), display launch |
| `rocky_msgs` | rosidl | GaitCommand, BodyPoseCommand, HandCommand, ContactState, ServoHealth |
| `rocky_gait` | ament_python | gait node wrapping `pebble_gait` (Twist-compatible) |
| `rocky_control` | ament_cmake | controllers.yaml + bringup launch (hw arg) |
| `rocky_driver` | ament_cmake | Python HW bridge (hw:=topic) + C++ SystemInterface skeleton (hw:=serial) |

## Hardware backend strategy

```
hw:=mock    ros2_control GenericSystem — RViz/kinematic testing, zero deps
hw:=topic   topic_based_ros2_control <-> hw_bridge_node.py <-> rocky_driver
            (RECOMMENDED for Phase 1-2: all bus I/O stays in the 58-test
             Python driver; C++ never touches a byte)
hw:=serial  rocky_driver/RockySystem C++ plugin — skeleton today; port the
            protocol from driver/rocky_driver/protocol.py when the control
            loop needs to shed the topic hop (it won't at 50 Hz/20 servos)
```

**D052 V2 — what the ROS path does and does NOT have yet** (the cockpit's
`sim/hw_bridge.py` is the reference): `hw_bridge_node.py` now streams through
`rocky_driver.SoftStream` (goal parked where each servo is at 40 % torque
before enable, smoothstep entry at 200 cps, NaN hold, 4.7 rad/s clamp),
ignores a joint command that misses a leg joint, and publishes NO contacts
until the switches are wired; `gait_node.py` takes its gait from
`cad/params.yaml` and fits every `/cmd_vel` into `WaveGait.budget()`, with the
same rate clamp. Still missing: the reflex supervisor (no IMU / contact
inputs on this path), the sim2real heartbeat, the leg-level fault cut and
the degraded-leg re-entry. Neither node has been run under ROS (no rclpy on
the dev box) — treat `hw:=topic` as bench-only, one leg, hand on the power
switch, until it has.

## First bringup on the laptop (when ROS exists)

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros2-control \
     ros-jazzy-ros2-controllers ros-jazzy-topic-based-ros2-control \
     ros-jazzy-xacro
cd rocky && pip install -e .                # pebble_gait + rocky_driver
mkdir -p ~/rocky_ws/src && ln -s $(pwd)/ros2/* ~/rocky_ws/src/
cd ~/rocky_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch rocky_description display.launch.py        # RViz + sliders
ros2 launch rocky_control control.launch.py hw:=mock   # full control stack
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.045}}"                            # it walks (kinematically)
# hardware day:
ros2 launch rocky_control control.launch.py hw:=topic
```

## Known TODOs

- claw joint state read-back (needs hand v0.3 calibration story)
- `rocky_sim` package (MuJoCo<->ROS bridge) — sim work currently runs
  directly in `sim/` without ROS, which is faster for iteration
- `rocky_teleop` (Xbox Elite mapping) — trivial `joy` remap, Phase 2
- IMU (BNO085) node + `ContactState` from real microswitches — Phase 3
