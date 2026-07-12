#pragma once

#include "config/board_pins.h"
#include "config/vehicle_config.h"
#include "hal/pwm_channel.h"

namespace balance_car::drivers
{
class MotorDriver
{
public:
  MotorDriver(const config::MotorPins &pins, const config::MotorConfiguration &configuration);

  bool begin();
  void setEnabled(bool enabled);
  bool isEnabled() const;
  void setNormalized(float leftCommand, float rightCommand);
  void stop();

private:
  struct WheelState
  {
    bool isDriving = false;
    bool directionIn1High = false;
  };

  void applyWheel(float command, uint8_t in1Pin, uint8_t in2Pin, bool directionInverted,
                  hal::PwmChannel &pwmChannel, WheelState &wheelState);
  static float clampNormalized(float value);

  const config::MotorPins &_pins;
  const config::MotorConfiguration &_configuration;
  hal::PwmChannel _leftPwm{0};
  hal::PwmChannel _rightPwm{1};
  WheelState _leftWheelState;
  WheelState _rightWheelState;
  bool _initialized = false;
  bool _enabled = false;
};
} // namespace balance_car::drivers
