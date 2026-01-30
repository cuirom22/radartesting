#pragma once
#include <Arduino.h>

struct RadarData {
  float range_m = NAN;
  float vel_mps = NAN;
  float conf    = NAN;
  uint32_t last_rx_ms = 0;
  uint32_t seq = 0;
};

class RadarLink {
public:
  void begin(Stream& s) { _s = &s; }
  void poll();                     // call often in loop()
  bool hasFresh(uint32_t timeout_ms) const;
  const RadarData& data() const { return _data; }

private:
  Stream* _s = nullptr;
  String _line;
  RadarData _data;

  void handleLine(const String& line);
};
