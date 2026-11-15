#pragma once
#include "leg_kinematics.h"

// Fusion leg_assembly v12 geometry recorded 2026-10-11 in leg_sim/leg_params.py;
// user confirmed same physical geometry 2026-11-02. Not a new bench measurement.
// Independent from the old coaxial-hip force estimate in Tier 0 fleet_config.h.
static const legkin::Geometry LEG_GEOMETRY={
  {.080f,.080f},{.100f,.100f},.043f,15.f*legkin::KIN_PI/180.f
};
// No absolute link angles or fixed branch signs have been measured at session ZERO.
// Set only after the reference fixture/geometry checks in docs/KINEMATICS.md.
// NAN and zero signs deliberately fail closed even if 'known' is toggled alone.
static const legkin::Reference LEG_KINEMATIC_REFERENCE={false,{NAN,NAN},{0,{0,0}}};
