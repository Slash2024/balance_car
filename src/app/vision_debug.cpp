#include "app/vision_debug.h"

namespace balance_car::app
{
namespace
{
constexpr uint32_t kReadPeriodMs = 20;
constexpr uint32_t kPrintPeriodMs = 500;
}

VisionDebug::VisionDebug(drivers::VisionI2cClient &client) : _client(client)
{
}

void VisionDebug::begin()
{
  Serial.println(_client.begin() ? "[VISION] I2C master ready" : "[VISION] I2C master init failed");
}

void VisionDebug::update(uint32_t nowMs)
{
  if (nowMs - _lastReadMs >= kReadPeriodMs)
  {
    _lastReadMs = nowMs;
    _client.read(_latest);
  }
  if (nowMs - _lastPrintMs < kPrintPeriodMs)
  {
    return;
  }

  _lastPrintMs = nowMs;
  Serial.printf("[VISION] i2c=%s seq=%u found=%d held=%d target=%d angle=%.2f valid=%u miss=%u thr=%u\n",
                _client.isHealthy() ? "OK" : "ERR", _latest.sequence,
                _latest.found ? 1 : 0, _latest.held ? 1 : 0,
                _latest.targetErrorPermille, _latest.angleCdeg / 100.0f,
                _latest.validRows, _latest.missedFrames, _latest.thresholdUsed);
}
} // namespace balance_car::app
