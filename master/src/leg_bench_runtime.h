#pragma once
// Included after nodes, bus and poll helpers. Only the poll task calls service.
// Host fake-bus tests bind the same adapter to deterministic node snapshots.
#include "leg_bench_policy.h"
static LegSession m_leg_session={};
static uint8_t m_leg_outstanding=0;
static uint32_t m_leg_last_control_ms[2]={};

static bool legPhoneExpired(uint32_t now) {
  return m_leg_phone_owned && now-m_leg_phone_ms>M_WEB_DEADMAN_MS;
}
static void legPhoneHeartbeat() {
  portENTER_CRITICAL(&m_mux);m_leg_phone_ms=millis();portEXIT_CRITICAL(&m_mux);
}

static bool legQueue(const LegRequest& request) {
  portENTER_CRITICAL(&m_mux);
  const bool accepted=!m_leg_request_pending && !m_leg_abort_pending;
  if(accepted){m_leg_request=request;m_leg_request_pending=true;}
  portEXIT_CRITICAL(&m_mux);
  return accepted;
}
static void legService(uint8_t replies=0,uint8_t failed=0) {
  LegObservation observation={}; LegRequest request={}; bool have_request=false;
  portENTER_CRITICAL(&m_mux);
  const uint32_t now=millis();
  for(int i=0;i<2;i++) {
    const auto& n=m_node[i]; auto& sample=observation.node[i];
    sample.seen=n.st_ms!=0;sample.age_ms=now-n.st_ms;sample.missed=n.miss_run;
    sample.p=n.st.p_rad;sample.v=n.st.v_rads;sample.status=n.st_status;
    sample.control_age_ms=now-m_leg_last_control_ms[i];
  }
  if(legPhoneExpired(now)) {request.kind=LEG_ABORT;have_request=true;m_leg_request_pending=false;}
  else if(m_leg_abort_pending){
    if(m_leg_session.phase!=LEG_FAULT){request.kind=LEG_ABORT;have_request=true;}
    m_leg_abort_pending=false;m_leg_request_pending=false;
  } else if(m_leg_request_pending){request=m_leg_request;have_request=true;m_leg_request_pending=false;}
  portEXIT_CRITICAL(&m_mux);
  observation.admin_reply_mask=replies;observation.admin_failed_mask=failed;
  LegActions actions={}; const LegPhase previous=m_leg_session.phase;
  legSessionStep(m_leg_session,LEG_CONFIG,observation,have_request?&request:nullptr,now,actions);
  if(have_request && request.kind==LEG_ARM && !actions.rejected && m_leg_session.phase==LEG_ARMING) {
    portENTER_CRITICAL(&m_mux);m_leg_phone_owned=request.from_phone;m_leg_phone_ms=now;portEXIT_CRITICAL(&m_mux);
  }
  if(actions.estop) {
    m_estop=true;allSessionsOff();estopBroadcast();m_leg_outstanding=0;
    portENTER_CRITICAL(&m_mux);
    for(int i=0;i<2;i++)m_node[i].req=REQ_NONE;
    m_leg_request_pending=false;
    portEXIT_CRITICAL(&m_mux);
    evt("LEG STOP: %s",actions.reason?actions.reason:"policy");
  } else if(actions.admin_mask) {
    const uint8_t op=actions.admin_op==CAN_OP_ZERO?REQ_ZERO:
                     actions.admin_op==CAN_OP_ARM?REQ_ARM:REQ_DISARM;
    m_leg_outstanding=actions.admin_mask;
    portENTER_CRITICAL(&m_mux);
    for(int i=0;i<2;i++)if(actions.admin_mask&(1u<<i))m_node[i].req=op;
    portEXIT_CRITICAL(&m_mux);
  }
  if(actions.rejected)evt("LEG refused: %s",actions.reason?actions.reason:"policy");
  if(previous!=m_leg_session.phase)evt("LEG phase %s (configured=%u)",legPhaseName(m_leg_session.phase),legConfigValid(LEG_CONFIG));
  portENTER_CRITICAL(&m_mux);
  m_leg_public=m_leg_session;
  if(m_leg_session.phase!=LEG_ARMING && m_leg_session.phase!=LEG_ARMED)m_leg_phone_owned=false;
  if(m_leg_session.phase==LEG_ARMED)for(int i=0;i<2;i++)m_node[i].cmd=m_leg_session.cmd[i];
  portEXIT_CRITICAL(&m_mux);
}
static void legAdminResult(int i,bool success) {
  const uint8_t bit=(uint8_t)(1u<<i);
  if(!(m_leg_outstanding&bit))return;
  if(success && m_leg_session.phase==LEG_ARMING && canStArmed(m_node[i].st_status) &&
     canStPhase(m_node[i].st_status)==CAN_PHASE_LIVE)m_leg_last_control_ms[i]=millis();
  m_leg_outstanding&=(uint8_t)~bit;
  legService(success?bit:0,success?0:bit);
}
static void legControlReply(int i,uint32_t sent_ms,uint8_t status) {
  if(canStArmed(status) && !canStFault(status) && canStPhase(status)==CAN_PHASE_LIVE)
    m_leg_last_control_ms[i]=sent_ms;
}
static bool legCommandForNode(int i,MitCmd& command) {
  legService(); // Recheck the deadline after IDENT/admin work, immediately before CMD.
  if(m_leg_session.phase!=LEG_ARMED || !m_node[i].session_armed || m_estop)return false;
  command=m_leg_session.cmd[i];return true;
}
