#pragma once
// ============================================================================
// safety.h -- THE SINGLE DISABLE PATH.
// ============================================================================
// claude.md section 9: "Single safety path. New e-stop call sites need design
// review." This file exists so the harness and Tier-0 cannot each grow their own
// copy of the disable sequence. Extracted verbatim from open_test.cpp
// (2026-09-10); byte-identical binary was the acceptance gate for the move.
//
// ---------------------------------------------------------------------------
// WHAT THIS FILE DELIBERATELY DOES NOT CONTAIN: THE ENVELOPE.
// ---------------------------------------------------------------------------
// OVERSPEED_RADS (150), AUTO_STOP_MS (20000), OVERSPEED_GRACE_MS (300) and
// GUARD_HITS (8) stay in open_test.cpp and are NOT moved here. Per claude.md's
// three-homes table those are "this bench harness's tuning and safety envelope
// ... not shipped to the robot" -- a 20-second auto-stop is a bench affordance,
// and shipping it to a robot inside a shared header would be a category error.
// MECHANISM LIVES HERE; THE ENVELOPE IS THE CALLER'S. A consumer (Tier-0) must
// supply its own thresholds and call stopMotor() to act on them.
//
// ---------------------------------------------------------------------------
// KNOWN GAP -- READ BEFORE ASSUMING THE RULE HOLDS. Recorded 2026-09-10.
// ---------------------------------------------------------------------------
// Extracting stopMotor() makes the path shareable; it does NOT yet make it
// single. The tree currently has 9 motor.disable() and 10 motor.enable() sites.
// Five in autocalib.h are a bare `motor.disable(); running = false;` -- this
// function minus the reason print (autocalib.h 428, 557, 1085, and the
// enable/disable pairs at 648-650 and 1611-1613).
// So a grep asserting "exactly one disable outside safety.h" FAILS TODAY, and
// failed before this refactor too. It is a finding, not an acceptance gate: a
// correct extraction cannot pass it. Routing those five through stopMotor() adds
// prints and shifts timing inside characterisation phases -- a BEHAVIOUR change,
// so it is a separate commit with its own bench verification, not this one.
//
// Layer: above actuator_hw.h (it owns disable, which is how the single-path rule
// survives Tier-0). Includes nothing upward.
// ============================================================================

void stopMotor(const char* reason) {
  motor.disable(); running = false;
  SerialUART.print(F("STOPPED (")); SerialUART.print(reason); SerialUART.println(F(")"));
}
