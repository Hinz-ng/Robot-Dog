# Design

[Project guide](README.md) · [Progress](PROGRESS.md) · [Technical reference](TECHNICAL.md)

The project begins with the actuator because its friction, response and mechanical stiffness will shape every later movement. The aim is a joint whose behaviour can be measured and controlled well enough to support a lightweight dynamic quadruped.

## A belt-driven actuator

A brushless outrunner drives the joint through a timing belt and a larger output pulley. The 9:1 reduction trades motor speed for output torque while keeping the transmission relatively simple. Printed components allow pulley and housing geometry to be revised without a custom gearbox manufacturing process.

This choice brings practical trade-offs:

| Choice | Benefit | Constraint |
|---|---|---|
| Modest reduction | Limits the multiplication of motor inertia at the joint | Requires useful torque from the motor and driver |
| Timing belt | Accessible components and a compact transmission layout | Belt tension, elastic deflection and tooth engagement affect behaviour |
| Printed mechanical parts | Fast iteration on housings and pulleys | Fit and print consistency need checking on the actual assembly |
| Motor-current-based torque estimate | Provides torque feedback without a dedicated output torque sensor | Calibration, friction and transmission losses limit the estimate |

The belt is an elastic part of the mechanism. Increasing commanded stiffness cannot make the transmission infinitely rigid, and a gain that works on a bare motor is not automatically suitable for an assembled leg.

## A five-bar leg

Two driven upper links connect to two lower links that meet at the foot. Together with the fixed spacing between the driven pivots, they form a five-bar linkage. Coordinating the two input angles places the foot within a plane.

Keeping these drive motors near the hip concentrates their mass near the body rather than carrying it down the lower leg. The cost is coupled geometry: both motors affect foot position, and the available motion must respect linkage configuration and mechanical clearance.

The current geometric model uses nominal 80 mm upper links, 100 mm lower links and 43 mm between the driven axes. These are CAD-based design dimensions; they do not establish measured foot-position accuracy. A separate hip motion is intended to move the leg sideways. Its mechanical implementation is still being developed.

## Controlling the response to contact

Instead of using position alone, the joint command includes target position, target velocity, stiffness, damping and an additional torque term. Stiffness determines how strongly the joint resists position error; damping determines how strongly it resists relative motion.

These settings make compliant behaviour possible, but successful landing or ground-contact control still depends on the assembled mechanism, limits and higher-level controller. Those robot-level behaviours remain development goals.

## From one joint to a robot

Local STM32 controllers handle motor control and joint protection. An ESP32-S3 master coordinates commands and communication. Future gait or learning-based control sits above that boundary, allowing motion planning to evolve without taking over motor commutation.

The next step is a repeatable, unloaded leg-positioning demonstration. [Progress](PROGRESS.md) separates the evidence already available from the work required to reach it.
