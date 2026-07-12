#pragma once

#include "config/board_pins.h"
#include "config/vehicle_config.h"
#include "drivers/wheel.h"

#include <Arduino.h>

namespace balance_car::drivers
{
struct WheelSpeed
{
  int32_t leftTicks;
  int32_t rightTicks;
  int32_t leftTickDelta;
  int32_t rightTickDelta;
  float leftTicksPerSecond;
  float rightTicksPerSecond;
  float leftMetersPerSecond;
  float rightMetersPerSecond;
};

class EncoderDriver
{
public:
  EncoderDriver(const config::EncoderPins &pins, const config::EncoderConfiguration &configuration);

  bool begin();
  bool isInitialized() const;
  void reset();
  int32_t readTicks(Wheel wheel) const;
  WheelSpeed sample(float deltaSeconds);

private:
  static void IRAM_ATTR onLeftInterrupt();
  static void IRAM_ATTR onRightInterrupt();
  void IRAM_ATTR handleLeftEdge();
  void IRAM_ATTR handleRightEdge();
  int32_t readTicksUnsafe(Wheel wheel) const;

  const config::EncoderPins &_pins;
  const config::EncoderConfiguration &_configuration;
  volatile int32_t _leftTicks = 0;
  volatile int32_t _rightTicks = 0;
  int32_t _lastLeftTicks = 0;
  int32_t _lastRightTicks = 0;
  portMUX_TYPE _criticalMux = portMUX_INITIALIZER_UNLOCKED;
  bool _initialized = false;

  static EncoderDriver *_activeInstance;
};
} // namespace balance_car::drivers
