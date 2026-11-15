#pragma once
#include <stdint.h>
#include <string.h>

// Caller holds the same lock used by the sample writer. Resume never erases data.
static inline const char* logControl(const char* action, bool storage, bool armed,
                                    uint32_t capacity, uint32_t& count, bool& recording) {
  if (!strcmp(action,"stop")) { recording=false; return "log paused"; }
  if (!strcmp(action,"clear")) {
    if (recording || armed) return "refused: pause log and disarm before clearing";
    count=0; return "log cleared";
  }
  if (strcmp(action,"start")) return "usage: log [start|stop|clear|dump]";
  if (!storage || count>=capacity) return "refused: log unavailable/full; download then clear";
  recording=true; return "log recording (append)";
}
