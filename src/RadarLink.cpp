#include "RadarLink.h"

void RadarLink::poll() {
  if (!_s) return;

  while (_s->available()) {
    char c = (char)_s->read();
    if (c == '\n') {
      _line.trim();
      if (_line.length() > 0) handleLine(_line);
      _line = "";
    } else if (c != '\r') {
      _line += c;
      if (_line.length() > 200) _line = ""; // basic safety against garbage spam
    }
  }
}

bool RadarLink::hasFresh(uint32_t timeout_ms) const {
  return (millis() - _data.last_rx_ms) < timeout_ms;
}

void RadarLink::handleLine(const String& line) {
  // Accept either:
  // RADAR,range,vel,conf
  // RADAR,seq,range,vel,conf
  float r,v,c;
  unsigned long seq;

  if (sscanf(line.c_str(), "RADAR,%lu,%f,%f,%f", &seq, &r, &v, &c) == 4) {
    _data.seq = (uint32_t)seq;
    _data.range_m = r;
    _data.vel_mps = v;
    _data.conf = c;
    _data.last_rx_ms = millis();
    return;
  }

  if (sscanf(line.c_str(), "RADAR,%f,%f,%f", &r, &v, &c) == 3) {
    _data.seq++;
    _data.range_m = r;
    _data.vel_mps = v;
    _data.conf = c;
    _data.last_rx_ms = millis();
    return;
  }
}
