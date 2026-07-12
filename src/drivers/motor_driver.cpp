#include "drivers/motor_driver.h"

#include <math.h>

namespace balance_car::drivers
{
MotorDriver::MotorDriver(const config::MotorPins &pins, const config::MotorConfiguration &configuration)
    : _pins(pins), _configuration(configuration)
{
}

bool MotorDriver::begin()
{
  pinMode(_pins.standby, OUTPUT);
  pinMode(_pins.leftIn1, OUTPUT);
  pinMode(_pins.leftIn2, OUTPUT);
  pinMode(_pins.rightIn1, OUTPUT);
  pinMode(_pins.rightIn2, OUTPUT);

  digitalWrite(_pins.standby, LOW);
  digitalWrite(_pins.leftIn1, LOW);
  digitalWrite(_pins.leftIn2, LOW);
  digitalWrite(_pins.rightIn1, LOW);
  digitalWrite(_pins.rightIn2, LOW);

  const bool leftReady = _leftPwm.begin(
      _pins.leftPwm, _configuration.pwmFrequencyHz, _configuration.pwmResolutionBits);
  const bool rightReady = _rightPwm.begin(
      _pins.rightPwm, _configuration.pwmFrequencyHz, _configuration.pwmResolutionBits);

  _initialized = leftReady && rightReady;
  stop();
  return _initialized;
}

void MotorDriver::setEnabled(bool enabled)
{
  if (!_initialized || !enabled)
  {
    stop();
    digitalWrite(_pins.standby, LOW);
    _enabled = false;
    return;
  }

  digitalWrite(_pins.standby, HIGH);
  _enabled = true;
}

bool MotorDriver::isEnabled() const
{
  return _enabled;
}

void MotorDriver::setNormalized(float leftCommand, float rightCommand)
{
  if (!_enabled)
  {
    stop();
    return;
  }

  applyWheel(clampNormalized(leftCommand), _pins.leftIn1, _pins.leftIn2,
             _configuration.leftDirectionInverted, _leftPwm, _leftWheelState);
  applyWheel(clampNormalized(rightCommand), _pins.rightIn1, _pins.rightIn2,
             _configuration.rightDirectionInverted, _rightPwm, _rightWheelState);
}

void MotorDriver::stop()
{
  _leftPwm.stop();
  _rightPwm.stop();
  digitalWrite(_pins.leftIn1, LOW);
  digitalWrite(_pins.leftIn2, LOW);
  digitalWrite(_pins.rightIn1, LOW);
  digitalWrite(_pins.rightIn2, LOW);
  _leftWheelState = {};
  _rightWheelState = {};
}

void MotorDriver::applyWheel(float command, uint8_t in1Pin, uint8_t in2Pin, bool directionInverted,
                             hal::PwmChannel &pwmChannel, WheelState &wheelState)
{
  const float magnitude = fabsf(command);
  if (magnitude < 0.001F)
  {
    pwmChannel.stop();
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    wheelState = {};
    return;
  }

  const bool requestedDirectionIn1High = (command > 0.0F) != directionInverted;
  if (!wheelState.isDriving || wheelState.directionIn1High != requestedDirectionIn1High)
  {
    pwmChannel.stop();
    digitalWrite(in1Pin, requestedDirectionIn1High ? HIGH : LOW);
    digitalWrite(in2Pin, requestedDirectionIn1High ? LOW : HIGH);
    wheelState.isDriving = true;
    wheelState.directionIn1High = requestedDirectionIn1High;
  }

  const uint32_t duty = static_cast<uint32_t>(magnitude * pwmChannel.maximumDuty() + 0.5F);
  pwmChannel.writeDuty(duty);
}

float MotorDriver::clampNormalized(float value)
{
  if (value > 1.0F)
  {
    return 1.0F;
  }
  if (value < -1.0F)
  {
    return -1.0F;
  }
  return value;
}
} // namespace balance_car::drivers
