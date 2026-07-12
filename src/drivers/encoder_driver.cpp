#include "drivers/encoder_driver.h"

#include <math.h>

namespace balance_car::drivers
{
EncoderDriver *EncoderDriver::_activeInstance = nullptr;

EncoderDriver::EncoderDriver(const config::EncoderPins &pins, const config::EncoderConfiguration &configuration)
    : _pins(pins), _configuration(configuration)
{
}

bool EncoderDriver::begin()
{
  const uint8_t inputMode = _configuration.useInternalPullups ? INPUT_PULLUP : INPUT;
  pinMode(_pins.leftPhaseA, inputMode);
  pinMode(_pins.leftPhaseB, inputMode);
  pinMode(_pins.rightPhaseA, inputMode);
  pinMode(_pins.rightPhaseB, inputMode);

  if (digitalPinToInterrupt(_pins.leftPhaseA) == NOT_AN_INTERRUPT ||
      digitalPinToInterrupt(_pins.rightPhaseA) == NOT_AN_INTERRUPT)
  {
    return false;
  }

  _activeInstance = this;
  attachInterrupt(digitalPinToInterrupt(_pins.leftPhaseA), onLeftInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pins.rightPhaseA), onRightInterrupt, CHANGE);
  reset();
  _initialized = true;
  return true;
}

bool EncoderDriver::isInitialized() const
{
  return _initialized;
}

void EncoderDriver::reset()
{
  portENTER_CRITICAL(&_criticalMux);
  _leftTicks = 0;
  _rightTicks = 0;
  _lastLeftTicks = 0;
  _lastRightTicks = 0;
  portEXIT_CRITICAL(&_criticalMux);
}

int32_t EncoderDriver::readTicks(Wheel wheel) const
{
  portENTER_CRITICAL(const_cast<portMUX_TYPE *>(&_criticalMux));
  const int32_t ticks = readTicksUnsafe(wheel);
  portEXIT_CRITICAL(const_cast<portMUX_TYPE *>(&_criticalMux));
  return ticks;
}

WheelSpeed EncoderDriver::sample(float deltaSeconds)
{
  WheelSpeed speed = {};
  if (!_initialized || deltaSeconds <= 0.0F)
  {
    return speed;
  }

  const int32_t leftTicks = readTicks(Wheel::Left);
  const int32_t rightTicks = readTicks(Wheel::Right);
  speed.leftTicks = leftTicks;
  speed.rightTicks = rightTicks;
  speed.leftTickDelta = leftTicks - _lastLeftTicks;
  speed.rightTickDelta = rightTicks - _lastRightTicks;
  speed.leftTicksPerSecond = static_cast<float>(speed.leftTickDelta) / deltaSeconds;
  speed.rightTicksPerSecond = static_cast<float>(speed.rightTickDelta) / deltaSeconds;

  const float wheelCircumferenceMeters = PI * _configuration.wheelDiameterMeters;
  const float metersPerTick = wheelCircumferenceMeters / _configuration.countsPerWheelRevolution;
  speed.leftMetersPerSecond = speed.leftTicksPerSecond * metersPerTick;
  speed.rightMetersPerSecond = speed.rightTicksPerSecond * metersPerTick;

  _lastLeftTicks = leftTicks;
  _lastRightTicks = rightTicks;
  return speed;
}

void IRAM_ATTR EncoderDriver::onLeftInterrupt()
{
  if (_activeInstance != nullptr)
  {
    _activeInstance->handleLeftEdge();
  }
}

void IRAM_ATTR EncoderDriver::onRightInterrupt()
{
  if (_activeInstance != nullptr)
  {
    _activeInstance->handleRightEdge();
  }
}

void IRAM_ATTR EncoderDriver::handleLeftEdge()
{
  const bool phaseA = digitalRead(_pins.leftPhaseA);
  const bool phaseB = digitalRead(_pins.leftPhaseB);
  int32_t increment = phaseA == phaseB ? 1 : -1;
  if (_configuration.leftDirectionInverted)
  {
    increment = -increment;
  }

  portENTER_CRITICAL_ISR(&_criticalMux);
  _leftTicks += increment;
  portEXIT_CRITICAL_ISR(&_criticalMux);
}

void IRAM_ATTR EncoderDriver::handleRightEdge()
{
  const bool phaseA = digitalRead(_pins.rightPhaseA);
  const bool phaseB = digitalRead(_pins.rightPhaseB);
  int32_t increment = phaseA == phaseB ? 1 : -1;
  if (_configuration.rightDirectionInverted)
  {
    increment = -increment;
  }

  portENTER_CRITICAL_ISR(&_criticalMux);
  _rightTicks += increment;
  portEXIT_CRITICAL_ISR(&_criticalMux);
}

int32_t EncoderDriver::readTicksUnsafe(Wheel wheel) const
{
  return wheel == Wheel::Left ? _leftTicks : _rightTicks;
}
} // namespace balance_car::drivers
