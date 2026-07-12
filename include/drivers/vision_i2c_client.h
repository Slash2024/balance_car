#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "config/board_pins.h"

namespace balance_car::drivers
{
struct VisionSample
{
  uint16_t sequence;
  bool found;
  bool held;
  bool targetValid;
  int16_t centerErrorPermille;
  int16_t targetErrorPermille;
  int16_t angleCdeg;
  uint8_t validRows;
  uint8_t missedFrames;
  uint8_t thresholdUsed;
};

class VisionI2cClient
{
public:
  VisionI2cClient(TwoWire &wire, const config::VisionI2cPins &pins);

  bool begin();
  bool read(VisionSample &sample);
  bool isHealthy() const;

private:
  static constexpr uint8_t kAddress = 0x42;
  static constexpr uint32_t kFrequencyHz = 400000;
  TwoWire &_wire;
  const config::VisionI2cPins &_pins;
  bool _healthy = false;
};
} // namespace balance_car::drivers
