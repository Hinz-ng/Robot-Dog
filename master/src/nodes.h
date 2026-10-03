#pragma once
#include <Arduino.h>
#include "can_proto.h"
#include "master_config.h"
// ============================================================================
// nodes.h -- what the master knows about each joint. (CAN-T0)
// ============================================================================
// Shared between the poll task (core 0, writes state, reads cmd) and the
// console (loop(), writes cmd/requests, reads state). Every cross-task access
// goes through m_mux; copies are taken inside the critical section and used
// outside it.
// ============================================================================

static portMUX_TYPE m_mux = portMUX_INITIALIZER_UNLOCKED;

enum : uint8_t {          // one-shot requests from the console to the poll task
  REQ_NONE = 0, REQ_ARM, REQ_DISARM, REQ_ZERO, REQ_CLEAR, REQ_IDENT,
};

struct Node {
  uint8_t  id;                 // JOINT_ID / CAN node
  // ---- written by the console ----
  MitCmd   cmd;                // the command sent each cycle -- ONLY if session_armed
  uint8_t  req;                // pending admin request
  // ---- written by the poll task ----
  bool     session_armed;      // THIS master armed it and has seen the armed bit since
  MitState st;                 // last STATE
  uint8_t  st_status;
  uint32_t st_ms;              // millis() of the last STATE
  CanStatus ident;             // last STATUS
  uint32_t ident_ms;           // 0 = never
  uint32_t last_ident_poll_ms;
  uint32_t sent, replies, missed, miss_run, rejected_pack, soft_limited;
  uint8_t  last_req_result;    // status byte of the reply to the last admin request
  bool     last_req_done;
};

static Node m_node[M_N_NODES];

// Sine generator, per node, run by the poll task: p_des = c + a*sin(2*pi*f*t).
struct SineGen { bool on; float centre, amp, freq_hz; uint32_t t0_ms; };
static SineGen m_sine[M_N_NODES];

static int nodeIndex(uint8_t id) {
  for (uint8_t i = 0; i < M_N_NODES; i++) if (M_NODES[i] == id) return i;
  return -1;
}

static void nodesInit() {
  for (uint8_t i = 0; i < M_N_NODES; i++) {
    m_node[i] = Node{};
    m_node[i].id = M_NODES[i];
    m_sine[i] = SineGen{};
  }
}

static const char* faultName(uint8_t f) {
  switch (f) {
    case CAN_FAULT_NONE:           return "none";
    case CAN_FAULT_CMD_TIMEOUT:    return "CMD_TIMEOUT";
    case CAN_FAULT_OVERSPEED:      return "OVERSPEED";
    case CAN_FAULT_SENSE_MISMATCH: return "SENSE_MISMATCH";
    case CAN_FAULT_P_ENVELOPE:     return "P_ENVELOPE";
    case CAN_FAULT_ESTOP:          return "ESTOP";
    case CAN_FAULT_LOCAL_STOP:     return "LOCAL_STOP";
    case CAN_FAULT_ENCODER:        return "ENCODER";
    default:                       return "?";
  }
}
static const char* phaseName(uint8_t p) {
  switch (p) { case CAN_PHASE_LIVE: return "LIVE"; case CAN_PHASE_HOLD: return "HOLD";
               case CAN_PHASE_DAMP: return "DAMP"; default: return "?"; }
}
