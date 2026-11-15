#pragma once
#include "leg_bench_policy.h"

// 2026-11-01 user: both p increase for photo-view CCW; pre-spacer leg attached.
// 2026-11-07 user requests wider outer envelope about the manual session zero.
// Use +/-120 deg targets inside +/-125 deg actual cutoffs; not a stopping-distance claim.
// These are bench limits, not a collision-free five-bar workspace certificate.
// Node order is J01 (photo right), J03 (photo left); all angles are output rad.
static const LegBenchConfig LEG_CONFIG = {
  true,
  { {1,1,0,-2.0943951f,2.0943951f,-2.1816616f,2.1816616f},
    {3,1,0,-2.0943951f,2.0943951f,-2.1816616f,2.1816616f} },
  10.0f, 0.1f, 0.02f, 0.05f,
  50, 100, 3, 10.0f, 10 // control gap is below Tier-0 DAMP at 15 ms
};
