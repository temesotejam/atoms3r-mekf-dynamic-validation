#pragma once

#include <stdint.h>
#include <string.h>

namespace autonomous_timing {

inline bool allowedUs(uint32_t value) {
  return value == 0 || value == 3000 || value == 6000 || value == 9000;
}

// HTTP input is an exact choice, not a permissive numeric conversion.
inline bool parseMs(const char* text, uint32_t& value_us) {
  if (!text) return false;
  if (strcmp(text, "0") == 0) value_us = 0;
  else if (strcmp(text, "3") == 0) value_us = 3000;
  else if (strcmp(text, "6") == 0) value_us = 6000;
  else if (strcmp(text, "9") == 0) value_us = 9000;
  else return false;
  return true;
}

class Selection {
 public:
  explicit Selection(uint32_t default_us)
      : selected_us_(default_us), run_us_(default_us) {}

  bool select(uint32_t value_us, bool stopped_and_available) {
    if (!stopped_and_available || !allowedUs(value_us)) return false;
    selected_us_ = value_us;
    return true;
  }
  void captureRun() { run_us_ = selected_us_; }
  uint32_t selectedUs() const { return selected_us_; }
  uint32_t runUs() const { return run_us_; }

 private:
  uint32_t selected_us_;
  uint32_t run_us_;
};

}  // namespace autonomous_timing
