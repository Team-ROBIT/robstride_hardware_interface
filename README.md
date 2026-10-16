# robstride_hardware_interface

## 1. Introduction

A [ros2_control](https://github.com/ros-controls/ros2_control) `hardware_interface::SystemInterface` plugin for [RobStride](https://www.robstride.com/) CAN actuators (RS00–RS06), built on [robstride_sdk](https://github.com/kmj060703/RobstrideSDK) and [robstride_interfaces](https://github.com/kmj060703/robstride_interfaces).

`read()` and `write()` never block on CAN I/O: they touch only the SDK's lock-free per-motor state and send frames in batches with `sendmmsg()`. The update rate is limited by CAN bus bandwidth, not by request/response round trips (see §10).

## 2. Prerequisites

- ROS 2 Jazzy
- A SocketCAN adapter per bus (e.g. canable/candlelight USB-CAN), RobStride motors, a motor power supply
- `robstride_sdk` and `robstride_interfaces` in the same workspace

## 3. Installation

```bash
cd ~/${WORKSPACE}/src
git clone https://github.com/kmj060703/RobstrideSDK.git
git clone https://github.com/kmj060703/robstride_hardware_interface.git
git clone https://github.com/kmj060703/robstride_interfaces.git

cd ~/${WORKSPACE}
colcon build --packages-select robstride_sdk robstride_interfaces robstride_hardware_interface
source install/setup.bash
```

## 4. CAN interface setup

RobStride motors run at 1 Mbps. Bring each interface up before launching:

```bash
./scripts/setup_can.sh can0                 # 1 Mbps, txqueuelen 1000
./scripts/setup_can.sh can0 1000000 1000    # explicit bitrate / txqueuelen
```

`txqueuelen` matters: activation sends a burst of frames per motor, and the kernel default of 10 drops most of it.

Many gs_usb/candlelight adapters do not support bus-off auto-recovery (`restart-ms`); the script detects this and continues. On those adapters a bus-off needs a manual `down`/`up` or a re-plug. The symptom is `SendFrames()` failing with `ENOBUFS` while the interface still reports `UP`; the plugin logs it explicitly.

## 5. Configuration

### Hardware parameters

| Param | Meaning | Default |
|---|---|---|
| `master_id` | Host CAN id, the source address of outgoing frames | `253` (`0xFD`) |
| `number_of_joints` | Optional cross-check against the number of `<joint>` blocks | unchecked |
| `error_timeout_ms` | Time without fresh feedback before a joint counts as lost | `4000` |
| `motor_can_timeout_ms` | Each motor's own `CAN_TIMEOUT` watchdog (`0x7028`): after this long without a command the motor releases torque. `0` disables it, so if the bus dies a motor keeps holding its last target. A motor that loses power still comes back limp | `error_timeout_ms` |
| `freeze_on_joint_loss` | See §6 | `true` |
| `torque_enable` | Whether activation engages torque. `false` brings the rig up readable but limp until `~/set_torque` is called | `true` |
| `activation_limit_margin` | Activation fails if a joint reads more than this many rad outside its URDF limits (§6). `0` disables the check | `0.35` (≈20°) |

### Joints and gpios

Each motor is described twice:

- **`<joint>`**: the ros2_control contract (motor `id`, command interface, state interfaces).
- **`<gpio>`**: the motor as a device (model, bus, run mode, gains, health interfaces).

A joint's `id` must equal a gpio's `ID`; `on_init` refuses to start otherwise.

| Joint param | Meaning |
|---|---|
| `id` | Motor's CAN id. Must match a gpio's `ID` |

| gpio param | Meaning |
|---|---|
| `type` | `robstride` |
| `ID` | Motor's CAN id |
| `can_interface` | SocketCAN interface of this motor (motors may be on different buses) |
| `actuator_type` | `"00"`..`"06"`, `"0"`..`"6"`, or `"RS00"`..`"RS06"` |
| `control_mode` | `motion` (default), `position_pp`, `velocity`, `current`, `position_csp` |
| `kp`, `kd` | Initial gains, `motion` mode only |
| `max_torque` | `motion` mode: the most the spring term `kp × (target − position)` may push, N·m. The target is kept within `max_torque / kp` of the measured position, so a gripper blocked by an object holds at this force instead of going to full current. Unset or `0`: no cap |
| `direction` | `1` (default) or `-1`. Joint frame = `direction` × motor frame for position, velocity and effort. Raw parameter writes through `~/set_data_to_robstride` stay in the motor frame |

| gpio interface | Kind | Meaning |
|---|---|---|
| `kp`, `kd` | command | Retune gains while running (same destination as `~/set_data_to_robstride`) |
| `temperature` | state | Motor temperature (°C) |
| `run_state` | state | `RunState` (`2` = torqued) |
| `fault_bits` | state | Latched fault word |

**The joint's command interface must match its gpio's `control_mode`**: `velocity` mode through `velocity`, `current` through `effort`, position modes through `position`. `on_init` rejects any other pairing. Only declared interfaces are exported.

```xml
<ros2_control name="robstride" type="system">
  <hardware>
    <plugin>robstride_hardware_interface/RobstrideHardware</plugin>
    <param name="master_id">253</param>
    <param name="error_timeout_ms">4000</param>
  </hardware>

  <joint name="rs_joint1">
    <param name="id">1</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <gpio name="rs1">
    <param name="type">robstride</param>
    <param name="ID">1</param>
    <param name="actuator_type">00</param>
    <param name="can_interface">can0</param>
    <param name="control_mode">motion</param>
    <param name="kp">40.0</param>
    <param name="kd">0.3</param>
    <param name="direction">-1</param>
    <command_interface name="kp"/>
    <command_interface name="kd"/>
    <state_interface name="temperature"/>
    <state_interface name="run_state"/>
    <state_interface name="fault_bits"/>
  </gpio>
</ros2_control>
```

A joint driven through `velocity` should be `type="continuous"` in the URDF. With command limit enforcement on, a `revolute` joint outside its limits has its velocity clamped to zero in both directions, so an overshoot cannot be recovered. Position-commanded joints are unaffected.

For a bench test without hardware, replace the `<hardware>` block with `mock_components/GenericSystem`.

## 6. Activation and joint loss

### Activation never produces motion

Engaging torque only ever means "hold where you are". Closed-loop modes keep their target in the motor's RAM, which survives a deactivate/reactivate, so enabling with a stale `loc_ref` would servo back to wherever it was last aimed. `on_activate()` therefore loads every target before any torque exists:

1. Open each bus and start its read thread.
2. Arm each motor's `CAN_TIMEOUT` watchdog and select its `run_mode`.
3. Send a Stop frame to every motor and wait for the reply. Stop is answered with the same feedback as any command, so it is a position probe that cannot move anything (and it clears torque left on by a run that died).
4. Load each mode's target (`loc_ref` / `spd_ref` / `iq_ref`) and the command interfaces from that measured position.
5. **Check the readings against the URDF limits.** Any joint more than `activation_limit_margin` outside its limits fails activation with the joint names and readings (with `torque_enable=false` it only logs). A wrong reading this large means the motor forgot its zero; see §7.
6. If `torque_enable` is `false`, stop here: motors stay readable and free to move.
7. Otherwise send Enable and wait up to `error_timeout_ms` for each motor to report `run_state == MOTOR`, resending every 200 ms.
8. If any joint never confirms, disable everything, close the buses and return `ERROR`.

If a joint reports no position in step 3, activation is refused rather than engaging torque against an unknown target.

### Losing a joint

A joint is lost when:

- `error_timeout_ms` passes without fresh feedback, or
- with torque requested, it leaves the enabled state after having been enabled. A motor whose supply blips reboots with torque off and can answer again inside `error_timeout_ms`, so staleness alone would miss it. A fault that drops the motor out of the enabled state counts the same way.

With `freeze_on_joint_loss: true` (default) the component **freezes**:

- The lost joint stays disabled and is not re-enabled. A motor that rebooted is already limp, so nothing can hold it.
- Every reachable joint keeps torque and holds the position it had at the loss.
- Controller commands are ignored.
- `read()` returns `OK`: nothing is torn down and no controller is deactivated.

The freeze is latched. A recovered joint stays limp, because its position may be off and the controllers' setpoints have moved on. Only a fresh activation resumes control. `~/reboot_robstride` clears a fault but does not bring the joint back; reactivate afterwards.

With `freeze_on_joint_loss: false`, `read()` returns `ERROR`, the framework deactivates the component and torque is cut on every joint.

Independently, `write()` sends a Disable frame instead of a target to a joint that is stale or reporting a fault. While a joint is untorqued, `read()` tracks its command to the measured position so it re-engages where it actually is.

### Position after a power cycle

A motor only knows its angle within one turn at power-up, so after a power cycle the reading can be off by a multiple of 2π. The reported position is wrapped into one turn centered on the middle of the joint's URDF limits (0 without limits), and `write()` adds the dropped turns back. A joint without limits is treated as continuous.

This recovers whole turns only. A zero the motor forgot shifts the reading by an arbitrary amount that wrapping cannot fix; step 5 of activation exists to catch it.

Position feedback wraps at ±4π (firmware behavior). See [robstride_sdk's README](https://github.com/kmj060703/RobstrideSDK/blob/main/README.md#5-actuator-models-and-limits).

## 7. Zero and diagnostics

### Setting the zero

`~/set_torque` and `~/set_zero_robstride` talk only to the motor; they do not override an active controller, which keeps writing its own setpoint. So deactivate the controller first and support the arm:

```bash
ros2 control switch_controllers --deactivate arm_controller --controller-manager /controller_manager
ros2 service call /robstride_hardware_interface/set_torque std_srvs/srv/SetBool "{data: false}"
# move the arm to the mechanical zero pose, then per joint:
ros2 service call /robstride_hardware_interface/set_zero_robstride robstride_interfaces/srv/SetZeroRobstride "{id: 5}"
```

- `~/set_zero_robstride` refuses while the motor's torque is on: zeroing under torque makes the controller chase a relabeled position.
- It sends the mechanical zero (Type 6), waits up to 400 ms for the motor's answer (none: `result: false`), then writes `zero_sta = 1` (start-up range -π..π).
- The Type-6 zero survives a power cycle without saving (checked on an RS00, 2026-10-17). `zero_sta` does not: it needs `save_parameters` (Type 22, manual 4.2.2), e.g. `"{id: 5, item_name: save_parameters, item_data: 0.0}"` on `~/set_data_to_robstride`.
- A motor reset by a power fault (undervoltage / overcurrent) has come back with joints 5 and 7 reading about ±180° off, while a clean power cycle kept the zero. After such a fault, power-cycle the motors cleanly and check the readings before re-setting any zero.
- While re-setting the zero, set `activation_limit_margin` to `0` if activation refuses the wrong readings.

### Status flags and faults

Every feedback frame carries six flags (ID bits 8..13). `read()` logs a joint's flags when they change (`Joint 'arm_joint4' ... reports undervoltage+overtemperature`) and lists them in the "Lost [...]" message. After a power blip they are usually the only clue to the cause.

`~/get_data_from_robstride` items: `position`, `velocity`, `torque` (or `effort`), `temperature`, `run_state`, `fault_bits`, `warning_bits`, `status_flags`.

| `status_flags` bit | Meaning |
|---|---|
| 0 | undervoltage |
| 1 | overcurrent |
| 2 | overtemperature |
| 3 | encoder fault |
| 4 | stall overload |
| 5 | uncalibrated |

`fault_bits` and `warning_bits` come from the Type-0x15 fault frame, read little-endian (bit 2 = undervoltage, bit 0 = overtemperature, ...). Versions before this one read the word big-endian, which swapped the bits; only "nonzero" was correct.

## 8. Topics and services

All under the hardware component's node namespace (`~`):

| Name | Type | Description |
|---|---|---|
| `~/robstride_state` | `robstride_interfaces/msg/RobstrideState` | Per-joint id/enabled/run_state/fault_bits, published every `read()` cycle |
| `~/get_data_from_robstride` | `robstride_interfaces/srv/GetDataFromRobstride` | Live telemetry of one motor (items in §7) |
| `~/set_data_to_robstride` | `robstride_interfaces/srv/SetDataToRobstride` | `kp`, `kd`, `save_parameters`, or any parameter index |
| `~/set_zero_robstride` | `robstride_interfaces/srv/SetZeroRobstride` | Set a motor's mechanical zero (§7) |
| `~/set_torque` | `std_srvs/srv/SetBool` | Enable/disable all configured motors |
| `~/reboot_robstride` | `robstride_interfaces/srv/RebootRobstride` | Clear a motor's latched fault. The caller decides which faults are safe to clear. The joint re-enables only after reactivation (§6) |

Field descriptions: [robstride_interfaces' README](https://github.com/kmj060703/robstride_interfaces/blob/main/README.md).

## 9. Threading model

The plugin inherits `rclcpp::Node` for the services, so it spins itself on its own executor thread (`service_executor_`, started in `on_init`), independent of the RT cycle. Calling `spin_some()` inside `read()` let a blocking callback freeze `read()`/`write()` long enough to trip the staleness check and deactivate the hardware.

Service callbacks therefore must not write the RT cycle's plain buffers. They park values in an atomic that `write()` picks up (see `pending_kp_`, `torque_enabled_desired_`).

## 10. Bus bandwidth

A 1 Mbps CAN bus is shared by every motor on it. Each cycle needs `2 · N_motors` frames (a command out and a feedback back per motor), and an 8-byte extended frame takes about 130–190 µs depending on bit stuffing:

```
utilization ≈ 2 · N_motors · update_rate_hz · frame_time_s
```

Measure rather than trust the estimate:

```bash
canbusload can0@1000000 -r -t -b -e
```

Near saturation, or if staleness errors appear under load, reduce `update_rate` or the number of motors per bus. A bus at 100% has no margin for retries or parameter writes and starts missing the `error_timeout_ms` deadline.

## 11. License

Apache License 2.0.
