#pragma once
// ============================================================================
// actuator_hw.h -- the actuator's hardware objects.
// ============================================================================
// EXTRACTED VERBATIM from open_test.cpp (2026-09-10) as stage 3a: a pure
// RELOCATION, gated on identical section sizes and an unchanged symbol table.
// The init ORDER, the boot Vbus seed and the CFG banner are still inline in
// open_test.cpp's setup() and arrive here in stage 3b, which is a restructure
// and carries a bench gate instead.
//
// Include point is DERIVED, not chosen: it must sit AFTER VBUS_FALLBACK (which
// vbus_filt is initialised from) and BEFORE vbusProbe(), which uses `motor`.
//
// ---------------------------------------------------------------------------
// *** UNIT RISK -- EVERY CURRENT THAT CROSSES THIS BOUNDARY IS REPORTED AMPS. ***
// ---------------------------------------------------------------------------
// i_scale (J01 0.9621, J02 0.9690) is NOT applied anywhere in this harness.
// joint_cal.h::calKtCmd() exists to convert a desired torque into a COMMAND
// current and currently HAS NO CALLER. Tier-0 is the first consumer, so this is
// the exact place a silent 3-4% torque error would enter the robot. Every limit
// below -- current_limit, the PI limits, CURR_MAX -- is in reported amps until
// something calls calKtCmd().
//
// ---------------------------------------------------------------------------
// ODR -- READ BEFORE INCLUDING THIS FROM A SECOND TRANSLATION UNIT.
// ---------------------------------------------------------------------------
// This header DEFINES these objects rather than declaring them. That is correct
// and harmless while open_test.cpp is the only TU, and it is a MULTIPLE
// DEFINITION error the first time Tier-0 includes it from two. The fix is an
// .h/.cpp pair -- which reopens the static-allocation question this refactor
// deliberately leaves closed (RAM is at 73.9%, ~8.5 kB headroom), so it is
// sequenced after Tier-0 starts, not now.
// ============================================================================

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(
    A_PHASE_UH, A_PHASE_UL,
    A_PHASE_VH, A_PHASE_VL,
    A_PHASE_WH, A_PHASE_WL
);
// Clone sense chain is gain-compensated -> genuine constants. Do not change.
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MT6816SPI encoder = MT6816SPI();           // ENC_BITS-bit absolute, ENC_CPR counts/rev

float vbus_filt           = VBUS_FALLBACK;
bool  vbus_valid          = false;
