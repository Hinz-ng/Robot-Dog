#pragma once
// Host-test stand-in for <Arduino.h>: only what fleet_config.h / joint_cal.h use.
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>

class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))

// Print that captures into a string, so a test can search the banner text.
struct Print {
  std::string buf;
  void print(const __FlashStringHelper* s) { buf += reinterpret_cast<const char*>(s); }
  void print(const char* s) { buf += s; }
  void print(char c) { buf += c; }
  void print(int v) { buf += std::to_string(v); }
  void print(unsigned v) { buf += std::to_string(v); }
  void print(long v) { buf += std::to_string(v); }
  void print(unsigned long v) { buf += std::to_string(v); }
  void print(double v, int d = 2) { char t[64]; snprintf(t, sizeof t, "%.*f", d, v); buf += t; }
  template <class T> void println(T v) { print(v); buf += "\n"; }
  void println(double v, int d) { print(v, d); buf += "\n"; }
  void println() { buf += "\n"; }
};
