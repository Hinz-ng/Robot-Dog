# Technical Reference

[Project guide](README.md) · [Design](DESIGN.md) · [Progress](PROGRESS.md)

This page outlines the software boundaries and joint interface for readers exploring the implementation. Hardware-specific calibration and powered operating procedures are maintained separately.

## Control architecture

| Layer | Responsibility | Implementation |
|---|---|---|
| Joint controller | Motor commutation, current control, joint state and local protection | STM32 firmware |
| Robot master | Joint commands, CAN communication, coordination and system safety decisions | ESP32-S3 firmware |
| Higher-level control | Future gait generation, learning-based policies and robot-level logging | Development target |

Field-oriented control regulates the motor's torque-producing current. The master communicates with the joint over CAN, a shared wired bus. Fast motor control remains local to each joint.

## Joint interface

Commands describe the desired behaviour at the actuator output:

| Input | Meaning | Unit |
|---|---|---|
| `p_des` | Target joint position | rad |
| `v_des` | Target joint velocity | rad/s |
| `kp` | Position stiffness | N·m/rad |
| `kd` | Velocity damping | N·m·s/rad |
| `tau_ff` | Additional commanded torque | N·m |

The controller combines them as:

```text
torque = kp × (target position − position)
       + kd × (target velocity − velocity)
       + additional torque
```

The result passes through torque and current limits before driving the motor. Feedback contains joint position, velocity and a torque estimate. The damping term is part of this torque calculation; there is no separate cascaded velocity PID in the joint interface.

The reported torque is derived from motor current, the calibrated motor torque constant and the reduction ratio. It does not subtract transmission friction or include a measured drivetrain-efficiency correction. It is therefore not an independent measurement of external joint torque or foot force.

## Read the code

| Location | Purpose |
|---|---|
| [`src/`](../src/) | STM32 actuator firmware and bench harness |
| [`src/mit_law.h`](../src/mit_law.h) | Spring-and-damper law and joint-state estimator |
| [`src/tier0/`](../src/tier0/) | CAN-facing joint runtime and local fault handling |
| [`src/fleet_config.h`](../src/fleet_config.h) | Configuration shared across the actuator fleet |
| [`src/joint_cal.h`](../src/joint_cal.h) | Calibration belonging to individual units |
| [`master/`](../master/) | ESP32-S3 master firmware |
| [`can_protocol/`](../can_protocol/) | Shared command, feedback and message definitions |
| [`platformio.ini`](../platformio.ini) | Actuator build environments and pinned dependencies |

## Configuration and validation boundaries

The public snapshot and newer local work use different sensor and board configurations; see [Progress](PROGRESS.md#public-code-and-recent-development). Calibration belongs to the physical unit and sensor assembly that produced it. Mechanical rebuilds can also change friction, compliance and suitable control gains.

Software builds and mathematical checks establish implementation behaviour. Bench measurements establish hardware behaviour for a stated assembly and configuration. Neither alone establishes safe loaded operation of a complete robot. These overview pages are not a commissioning or powered-test procedure.
