# Quadruped Actuator

I'm building a robot dog that can sprint, jump and eventually backflip: 4 kg, 12 joints, every
part designed and built from scratch. A robot like that is only as good as its leg motors, so
this repo starts there, with the actuator.

<!-- Photo or short clip of the actuator on the bench goes here. -->

## The idea

Small hobby servos can't jump. Dynamic legged robots like MIT's Mini Cheetah use
**quasi-direct-drive** actuators instead: a strong brushless motor with a small gear reduction,
so the leg stays fast and springy and can feel the ground through its own motor current.
This is my low-cost version of that:

| | |
|---|---|
| Motor | TYI 4006 KV360 brushless outrunner |
| Reduction | 9:1 belt drive (10 mm GT2) |
| Motor driver | STM32G431 field-oriented-control board, running SimpleFOC |
| Encoder | 14-bit magnetic, 0.04 mrad resolution at the joint |
| Joint loop | ~12 kHz |
| Leg | five-bar linkage (80 mm / 100 mm links), two actuators per leg, servo for hip ab/ad |
| Power | 5S LiPo |

## How a joint behaves

Each joint acts like a programmable spring and damper. The main controller sends five numbers
(target position, target velocity, stiffness, damping and a feed-forward torque) and the joint
reports back its position, velocity and torque. Low stiffness makes the leg soft for landing,
high stiffness makes it rigid for pushing off. It's the same interface the big research
quadrupeds use, so a gait controller or a reinforcement-learning policy can drive it directly.

```
Gait / RL policy
      │
ESP32-S3 master      CAN bus, IMU, safety, phone control panel
      │  CAN
STM32 per joint      motor commutation, current control, joint limits
```

## Progress

- ✅ Motor and driver measured on the bench (torque constant, resistance, inductance, rotor inertia)
- ✅ Current control, encoder and alignment calibration
- ✅ Belt drive built and accepted
- ✅ Spring/damper control on one belt-driven joint, safe gain limits measured
- ✅ Controlled over CAN from the ESP32 and a phone, with e-stop, dead-man switch and
  cable-pull tests passing
- 🔄 Two joints on one bus
- ⬜ First full leg
- ⬜ The whole dog

Every constant in the firmware comes from a bench measurement, not a datasheet. The driver
boards are clones, and they disagree with their documentation more often than you'd expect.

## Documentation

| Doc | What's in it |
|---|---|
| [Engineering hub](docs/README.md) | live status board and an index of everything below |
| [Hardware](docs/HARDWARE.md) | parts, clone vs genuine board differences, pin map, thermal limits |
| [Constants](docs/CONSTANTS.md) | every measured number in one table |
| [Firmware](docs/FIRMWARE.md) | the joint command contract and how a bench session runs |
| [Control](docs/CONTROL.md) | current loop design and tuning |
| [Sensing](docs/SENSING.md) | encoder, motor alignment, current sensing |
| [Belt drive](docs/BELT_DRIVE.md) | belt build procedure, idlers, pulley printing recipe |
| [CAN bring-up](docs/CAN_BRINGUP.md) | getting the STM32s and the ESP32 talking |
| [Robot design](docs/ROBOT_DESIGN.md) | leg geometry, gear ratio and battery choices |
| [Failure modes](docs/FAILURE_MODES.md) | the bugs that cost the most time, and how to spot them |
| [New joint sheet](docs/cal/BELT_OFF_BASELINE.md) | step-by-step bring-up for each new actuator |

## Repo layout

```
src/        joint firmware (STM32, PlatformIO)
master/     ESP32-S3 master firmware
contract/   CAN message definitions shared by both
tools/      CAN bring-up sketches and helpers
docs/       everything above
```
