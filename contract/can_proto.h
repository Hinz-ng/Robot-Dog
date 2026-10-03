#pragma once
// ============================================================================
// can_proto.h -- THE TIER-0 CAN CONTRACT ON THE WIRE. (CAN-T0, 2026-10-03)
// ============================================================================
// Frame IDs, field packing and scaling for mit_types.h's {p_des, v_des, kp,
// kd, tau_ff} -> {p, v, tau}. Both tiers include this header and nothing else
// from each other (contract/ is the only shared directory). PURE: no Arduino,
// no HAL, no I/O -- so the same golden vectors (can_proto_vectors.h) run on the
// STM32G431 and on the ESP32-S3, each on its own compiler.
//
// *** v0 -- NOT FROZEN. *** Freezes together with 7l (homing / absolute zero),
// the RL observation/action vector, and the S1e real-frame-width capture
// (docs/CAN_BRINGUP.md 23.6). Freeze checklist item already known: every
// velocity field must cover >= 1.1x the no-load OUTPUT speed at peak pack
// voltage (~75-88 rad/s at 21 V); v0's +/-40.9 (cmd) and +/-65.5 (state) do not.
//
// CAN_CONTRACT_VERSION: bump on ANY change to an ID, a layout, an LSB, a range,
// or an opcode's meaning. Tier 0 refuses an ARM carrying a different version,
// and the master refuses to arm a node whose STATUS reports one. A stale binary
// obeying a different layout is the error class this exists to stop.
//
// UNITS: output side, SI, exactly as mit_types.h. GEAR_RATIO and encoder counts
// never appear on the wire.
//
// ENCODING RULES
//   * Every field is an INTEGER COUNT with a defined LSB, two's complement if
//     signed. Zero is exactly zero. (The MIT-Cheetah min/max mapping puts zero
//     half an LSB off -- e.g. v_des = 0 is not representable -- so it is not used.)
//   * Rounding is to nearest, halves away from zero.
//   * COMMANDS are REJECTED, never saturated, when non-finite or outside the
//     representable range: canPackCmd() returns false and nothing is sent.
//     "Reject, never repair" (mit_law.h) -- a clipped command still moves a motor.
//   * MEASUREMENTS (STATE) saturate at the range end; the end value itself is
//     the flag. A non-finite measurement packs as 0 and returns false.
//   * Byte order little-endian, packed byte by byte (never a struct memcpy), so
//     the layout does not depend on either compiler's struct rules.
// ============================================================================
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "mit_types.h"

static const uint8_t CAN_CONTRACT_VERSION = 1;

// ---------------------------------------------------------------------------
// IDs -- 11-bit standard. n = JOINT_ID, 1..12.
// Bus discipline is POLLED: the master sends, the addressed joint replies, the
// master moves on. A joint never transmits unsolicited, so there is no
// arbitration contention and single-shot (DAR = 1 / ss = 1, CAN_BRINGUP 23.4b)
// costs nothing. ESTOP is 0x000 so it wins arbitration against anything.
// ---------------------------------------------------------------------------
static const uint16_t CAN_ID_ESTOP       = 0x000;  // M -> all, DLC 0
static const uint16_t CAN_ID_ADMIN_BASE  = 0x010;  // M -> J,   DLC 3
static const uint16_t CAN_ID_CMD_BASE    = 0x100;  // M -> J,   DLC 8
static const uint16_t CAN_ID_STATE_BASE  = 0x180;  // J -> M,   DLC 8 (reply to CMD and to non-IDENT ADMIN)
static const uint16_t CAN_ID_STATUS_BASE = 0x700;  // J -> M,   DLC 8 (reply to IDENT)
static const uint8_t  CAN_NODE_MIN = 1;
static const uint8_t  CAN_NODE_MAX = 12;

static inline uint16_t canIdAdmin (uint8_t n) { return (uint16_t)(CAN_ID_ADMIN_BASE  + n); }
static inline uint16_t canIdCmd   (uint8_t n) { return (uint16_t)(CAN_ID_CMD_BASE    + n); }
static inline uint16_t canIdState (uint8_t n) { return (uint16_t)(CAN_ID_STATE_BASE  + n); }
static inline uint16_t canIdStatus(uint8_t n) { return (uint16_t)(CAN_ID_STATUS_BASE + n); }

// Node number carried by `id` if it lies in [base+1, base+12], else 0.
static inline uint8_t canNodeOf(uint16_t id, uint16_t base) {
  if (id < base + CAN_NODE_MIN || id > base + CAN_NODE_MAX) return 0;
  return (uint8_t)(id - base);
}

// ---------------------------------------------------------------------------
// ADMIN frame: [0] opcode  [1] joint-id echo  [2] contract version.
// The echo is a second addressing check: a frame on 0x010+n whose byte 1 is
// not n is refused, so an ID typo in the master cannot arm the wrong joint.
// ---------------------------------------------------------------------------
enum CanAdminOp : uint8_t {
  CAN_OP_IDENT       = 1,   // reply: STATUS
  CAN_OP_ARM         = 2,   // refused if already armed, faulted, or not ready
  CAN_OP_DISARM      = 3,
  CAN_OP_ZERO        = 4,   // session zero at the current pose; refused while armed
  CAN_OP_CLEAR_FAULT = 5,   // refused while armed
  CAN_OP_POLL        = 6,   // no action, reply STATE. How the master MONITORS a
                            // node it has not armed: it never sends a CMD to a
                            // joint it did not arm in this session (a rebooted
                            // master must not resume a joint mid-DAMP).
};
static const uint8_t CAN_ADMIN_DLC = 3;

// ---------------------------------------------------------------------------
// Latched fault codes (STATE status bits 4-7). 0 = none. Cleared only by
// CLEAR_FAULT; a fault always means the motor was disabled through stopMotor().
// ---------------------------------------------------------------------------
enum CanFault : uint8_t {
  CAN_FAULT_NONE           = 0,
  CAN_FAULT_CMD_TIMEOUT    = 1,   // no valid CMD for T_OFF while armed
  CAN_FAULT_OVERSPEED      = 2,
  CAN_FAULT_SENSE_MISMATCH = 3,
  CAN_FAULT_P_ENVELOPE     = 4,
  CAN_FAULT_ESTOP          = 5,   // 0x000 received
  CAN_FAULT_LOCAL_STOP     = 6,   // Tier-0 console 'x'
  CAN_FAULT_ENCODER        = 7,   // MT6816 No_Mag
};

// Command phase while armed (STATE status bits 1-2). See tier0_config.h.
enum CanPhase : uint8_t {
  CAN_PHASE_LIVE = 0,   // fresh commands arriving
  CAN_PHASE_HOLD = 1,   // commands late; last command held
  CAN_PHASE_DAMP = 2,   // commands lost; pure damping substituted
};

// STATE status byte: bit0 armed | bits1-2 phase | bit3 a clamp bound since the
// previous reply | bits4-7 latched fault.
static inline uint8_t canStatusByte(bool armed, uint8_t phase, bool clamped, uint8_t fault) {
  return (uint8_t)((armed ? 1u : 0u) | ((phase & 3u) << 1) | (clamped ? 8u : 0u) | ((fault & 15u) << 4));
}
static inline bool    canStArmed  (uint8_t s) { return (s & 1u) != 0; }
static inline uint8_t canStPhase  (uint8_t s) { return (uint8_t)((s >> 1) & 3u); }
static inline bool    canStClamped(uint8_t s) { return (s & 8u) != 0; }
static inline uint8_t canStFault  (uint8_t s) { return (uint8_t)(s >> 4); }

// ---------------------------------------------------------------------------
// SCALING. LSBs are the contract; ranges follow from the field widths.
// ---------------------------------------------------------------------------
//                              field   bits   LSB                   range
static const float CAN_LSB_P_DES  = 6.28318531f / 32768.0f; // int16  +/-2*pi rad (0.192 mrad)
static const float CAN_LSB_V_DES  = 0.02f;                  // int12  +/-40.96 rad/s
static const float CAN_LSB_KP     = 0.1f;                   // uint12 0..409.5 N.m/rad
static const float CAN_LSB_KD     = 0.001f;                 // uint12 0..4.095 N.m.s/rad
static const float CAN_LSB_TAU_FF = 0.005f;                 // int12  +/-10.24 N.m
static const float CAN_LSB_P      = 1e-5f;                  // int24  +/-83.9 rad (finer than one
                                                            //        output count, 4.26e-5 rad)
static const float CAN_LSB_V      = 0.002f;                 // int16  +/-65.5 rad/s
static const float CAN_LSB_TAU    = 0.0005f;                // int16  +/-16.4 N.m
// Quantisation x the demonstrated gain box (b1: kp <= 41, kd <= 0.365):
// p_des 0.192 mrad x 41 = 7.9 mN.m; v_des 0.02 x 0.365 = 7.3 mN.m -- both below
// the 14 mN.m kd count-flicker blip measured at b1.

// x -> integer count in [lo, hi]. false if non-finite or out of range.
static inline bool canQuant(float x, float lsb, int32_t lo, int32_t hi, int32_t& q) {
  if (!isfinite(x)) return false;
  const float r  = x / lsb;
  const float rr = (r >= 0.0f) ? floorf(r + 0.5f) : ceilf(r - 0.5f);
  if (!(rr >= (float)lo && rr <= (float)hi)) return false;
  q = (int32_t)rr;
  return true;
}
// Saturating version for measurements. Non-finite -> 0, returns false.
static inline bool canQuantSat(float x, float lsb, int32_t lo, int32_t hi, int32_t& q) {
  if (!isfinite(x)) { q = 0; return false; }
  const float r  = x / lsb;
  const float rr = (r >= 0.0f) ? floorf(r + 0.5f) : ceilf(r - 0.5f);
  q = (rr <= (float)lo) ? lo : (rr >= (float)hi) ? hi : (int32_t)rr;
  return true;
}
// Sign-extend the low `bits` of u.
static inline int32_t canSext(uint32_t u, uint8_t bits) {
  const uint32_t m = 1u << (bits - 1);
  u &= (1u << bits) - 1u;
  return (int32_t)(u ^ m) - (int32_t)m;      // portable: no implementation-defined shift
}

// ---------------------------------------------------------------------------
// CMD  (M -> J, 0x100+n, DLC 8)
//   bytes 0-1  p_des   int16 LE
//   bytes 2-7  48-bit LE word: v_des int12 [0..11] | kp uint12 [12..23]
//                             | kd uint12 [24..35] | tau_ff int12 [36..47]
// ---------------------------------------------------------------------------
static inline bool canPackCmd(const MitCmd& c, uint8_t out[8]) {
  int32_t qp, qv, qkp, qkd, qff;
  if (!canQuant(c.p_des_rad,      CAN_LSB_P_DES,  -32768, 32767, qp))  return false;
  if (!canQuant(c.v_des_rads,     CAN_LSB_V_DES,  -2048,  2047,  qv))  return false;
  if (!canQuant(c.kp_Nm_per_rad,  CAN_LSB_KP,      0,     4095,  qkp)) return false;
  if (!canQuant(c.kd_Nms_per_rad, CAN_LSB_KD,      0,     4095,  qkd)) return false;
  if (!canQuant(c.tau_ff_Nm,      CAN_LSB_TAU_FF, -2048,  2047,  qff)) return false;
  const uint16_t up = (uint16_t)qp;
  out[0] = (uint8_t)up; out[1] = (uint8_t)(up >> 8);
  const uint64_t w =  ((uint64_t)((uint32_t)qv  & 0xFFFu))
                   | (((uint64_t)((uint32_t)qkp & 0xFFFu)) << 12)
                   | (((uint64_t)((uint32_t)qkd & 0xFFFu)) << 24)
                   | (((uint64_t)((uint32_t)qff & 0xFFFu)) << 36);
  for (int i = 0; i < 6; i++) out[2 + i] = (uint8_t)(w >> (8 * i));
  return true;
}

// Any 8 bytes decode to finite numbers inside the field ranges, so this cannot
// fail; whether the command is ACCEPTABLE is the receiver's validator's call
// (mit_law.h mitCmdValid against Tier 0's own, narrower ranges).
static inline void canUnpackCmd(const uint8_t in[8], MitCmd& c) {
  const int32_t qp = canSext((uint32_t)in[0] | ((uint32_t)in[1] << 8), 16);
  uint64_t w = 0;
  for (int i = 0; i < 6; i++) w |= ((uint64_t)in[2 + i]) << (8 * i);
  c.p_des_rad      = (float)qp * CAN_LSB_P_DES;
  c.v_des_rads     = (float)canSext((uint32_t)(w & 0xFFFu), 12) * CAN_LSB_V_DES;
  c.kp_Nm_per_rad  = (float)(uint32_t)((w >> 12) & 0xFFFu) * CAN_LSB_KP;
  c.kd_Nms_per_rad = (float)(uint32_t)((w >> 24) & 0xFFFu) * CAN_LSB_KD;
  c.tau_ff_Nm      = (float)canSext((uint32_t)((w >> 36) & 0xFFFu), 12) * CAN_LSB_TAU_FF;
}

// ---------------------------------------------------------------------------
// STATE  (J -> M, 0x180+n, DLC 8)
//   bytes 0-2  p    int24 LE     bytes 3-4  v  int16 LE
//   bytes 5-6  tau  int16 LE     byte  7    status (canStatusByte)
// tau is MEASURED: irepToTorqueOut(Iq), the feedback half of the torque boundary.
// ---------------------------------------------------------------------------
static inline bool canPackState(const MitState& s, uint8_t status, uint8_t out[8]) {
  int32_t qp, qv, qt;
  bool ok = true;
  ok &= canQuantSat(s.p_rad,  CAN_LSB_P,   -(1 << 23), (1 << 23) - 1, qp);
  ok &= canQuantSat(s.v_rads, CAN_LSB_V,   -32768,     32767,         qv);
  ok &= canQuantSat(s.tau_Nm, CAN_LSB_TAU, -32768,     32767,         qt);
  const uint32_t up = (uint32_t)qp;
  out[0] = (uint8_t)up; out[1] = (uint8_t)(up >> 8); out[2] = (uint8_t)(up >> 16);
  out[3] = (uint8_t)((uint32_t)qv); out[4] = (uint8_t)((uint32_t)qv >> 8);
  out[5] = (uint8_t)((uint32_t)qt); out[6] = (uint8_t)((uint32_t)qt >> 8);
  out[7] = status;
  return ok;
}

static inline void canUnpackState(const uint8_t in[8], MitState& s, uint8_t& status) {
  s.p_rad  = (float)canSext((uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16), 24) * CAN_LSB_P;
  s.v_rads = (float)canSext((uint32_t)in[3] | ((uint32_t)in[4] << 8), 16) * CAN_LSB_V;
  s.tau_Nm = (float)canSext((uint32_t)in[5] | ((uint32_t)in[6] << 8), 16) * CAN_LSB_TAU;
  status   = in[7];
}

// ---------------------------------------------------------------------------
// STATUS  (J -> M, 0x700+n, DLC 8) -- the reply to IDENT. Identifies WHAT is
// running, so a stale or wrong binary is caught before it is armed.
//   [0] build type ('T' = Tier 0)   [1] contract version
//   [2-5] git hash u32 LE           [6] joint id   [7] flags
// ---------------------------------------------------------------------------
static const uint8_t CAN_BUILD_TIER0 = 'T';
enum CanStatusFlag : uint8_t {
  CAN_SF_DIRTY      = 1u << 0,   // built from a tree with uncommitted changes
  CAN_SF_UID_OK     = 1u << 1,   // MCU UID matches this JOINT_ID's joint_cal.h row
  CAN_SF_CALIBRATED = 1u << 2,   // zea, dir, Ke present -> stored-ZEA commutation possible
  CAN_SF_HSE_OK     = 1u << 3,   // HSERDY came up: the bit clock is the 8 MHz crystal
  CAN_SF_FOC_READY  = 1u << 4,
  CAN_SF_CS_OK      = 1u << 5,   // current sense linked
  CAN_SF_ARMED      = 1u << 6,
  CAN_SF_SELFTEST   = 1u << 7,   // can_proto_vectors.h passed on THIS build (refuse to arm if not)
};
struct CanStatus {
  uint8_t  build_type;
  uint8_t  contract_version;
  uint32_t git_hash;
  uint8_t  joint_id;
  uint8_t  flags;
};
static inline void canPackStatus(const CanStatus& st, uint8_t out[8]) {
  out[0] = st.build_type; out[1] = st.contract_version;
  for (int i = 0; i < 4; i++) out[2 + i] = (uint8_t)(st.git_hash >> (8 * i));
  out[6] = st.joint_id; out[7] = st.flags;
}
static inline void canUnpackStatus(const uint8_t in[8], CanStatus& st) {
  st.build_type = in[0]; st.contract_version = in[1];
  st.git_hash = (uint32_t)in[2] | ((uint32_t)in[3] << 8) | ((uint32_t)in[4] << 16) | ((uint32_t)in[5] << 24);
  st.joint_id = in[6]; st.flags = in[7];
}

static inline void canPackAdmin(uint8_t op, uint8_t node, uint8_t out[CAN_ADMIN_DLC]) {
  out[0] = op; out[1] = node; out[2] = CAN_CONTRACT_VERSION;
}
