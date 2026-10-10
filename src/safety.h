#pragma once
#include "actuator_hw.h"   // motor -- the objects this acts on
// ============================================================================
// safety.h -- THE SINGLE DISABLE PATH.
// ============================================================================
// CLAUDE.md: "Single safety path. New e-stop call sites need design review."
// The harness and Tier 0 both stop the motor through stopMotor(); neither has
// its own disable sequence.
//
// MECHANISM ONLY. The envelope (overspeed, auto-stop, guard thresholds) belongs
// to the caller: open_test.cpp for the bench, tier0_config.h for Tier 0.
//
// UPWARD DEPENDENCIES: stopMotor() uses the includer's SerialUART and `running`
// (Tier 0 declares both in tier0.h). Changing the signature is a change to the
// disable path and needs design review plus bench verification.
//
// KNOWN GAP: the path is shared, not yet single. autocalib.h still calls
// motor.disable() directly (acExit(), acCoast(), the end of phase 5, and the
// enable/initFOC/disable pairs in phase 2, 'V' and alignment). Routing them
// through stopMotor() adds prints and shifts timing inside characterisation
// phases, so it is a behaviour change with its own bench verification.
// ============================================================================

void stopMotor(const char* reason) {
  motor.disable(); running = false;
  SerialUART.print(F("STOPPED (")); SerialUART.print(reason); SerialUART.println(F(")"));
}
