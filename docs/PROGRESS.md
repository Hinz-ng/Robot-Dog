# Development Progress

[Project guide](README.md) · [Design](DESIGN.md) · [Technical reference](TECHNICAL.md)

**Reviewed: 15 November 2026.** The project is at the actuator and first-leg development stage. Walking, jumping and backflips are future goals.

## Demonstrated on the bench

| Area | Evidence established | Scope |
|---|---|---|
| Motor control and calibration | Electrical characterisation, encoder alignment and current-control tests | Individual tested motor, sensor and driver assemblies |
| Belt transmission | A printed pulley and idler arrangement has been built and tested | Dated actuator configurations; rebuilding changes their applicability |
| Adjustable joint behaviour | Spring-and-damper control, command limits and selected gain settings have been tested | One reference belt-driven actuator, rather than a complete leg |
| Remote joint control | CAN commands, feedback and stop/recovery cases have been exercised | Earlier STM32G431 bench configurations; acceptance does not transfer automatically to another board or assembly |

Together, these results establish a useful actuator platform. They do not yet establish loaded leg performance or whole-robot capability.

## Recent development

First-leg work has added joint coordination, geometry calculations and staged integration tools. Target-and-return command delivery has been demonstrated on both joints in the test leg. Useful physical tracking and accurate foot positioning remain to be established on the final assembly.

Additional STM32G473-based actuators have progressed through bare-motor calibration and bench checks. This is a separate hardware configuration with its own integration work; high-voltage, loaded and CAN/fault qualification remain distinct steps.

The mechanical layout is also evolving. The development CAD shown in the [project guide](README.md) includes a proposed sideways hip drive. Mounting, clearance and assembly details are still being resolved.

## Next milestones

1. **A repeatable unloaded leg demonstration.** Finalise the relevant assembly, establish its physical reference and usable motion region, then verify that commanded foot positions are reached consistently.
2. **Loaded leg operation.** Establish force delivery, transmission limits and thermal behaviour on the assembled leg before increasing the operating envelope.
3. **Whole-robot integration.** Develop body power distribution, coordinated control and walking before attempting dynamic manoeuvres.

These milestones describe the development sequence, without predicting completion dates.

## Public code and recent development

The public default branch reviewed for this documentation was commit [`d78a55b`](https://github.com/Hinz-ng/Robot-Dog/commit/d78a55bcd1f7b4997a17d778b1f58c9b40e044a4), dated 11 October 2026. It contains an earlier STM32G431/MT6816 firmware snapshot.

The recent-development summary above reflects local engineering records through 14 November 2026, including MT6835 sensing, G473 bench work and first-leg development. Those updates are not all present in that public snapshot. Readers reproducing or reviewing firmware should use the configuration and history of the code revision they have checked out.
