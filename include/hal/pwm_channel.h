#pragma once

#include <Arduino.h>

namespace balance_car::hal
{
class PwmChannel
{
public:
  explicit PwmChannel(uint8_t channel);

  bool begin(uint8_t pin, uint32_t frequencyHz, uint8_t resolutionBits);
  void writeDuty(uint32_t duty);
  void stop();
  uint32_t maximumDuty() const;

private:
  uint8_t _channel;
  uint8_t _pin = 0;
  uint32_t _maximumDuty = 0;
  bool _initialized = false;
};
} // namespace balance_car::hal
