#pragma once

#include <Arduino.h>

#include "drivers/vision_i2c_client.h"

namespace balance_car::app
{
// 独立的视觉通信调试器，不与 IMU 遥测混合输出。
class VisionDebug
{
public:
  explicit VisionDebug(drivers::VisionI2cClient &client);

  void begin();
  void update(uint32_t nowMs);

private:
  drivers::VisionI2cClient &_client;
  drivers::VisionSample _latest = {};
  uint32_t _lastReadMs = 0;
  uint32_t _lastPrintMs = 0;
};
} // namespace balance_car::app
