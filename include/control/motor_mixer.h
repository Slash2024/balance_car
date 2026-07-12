#pragma once

namespace balance_car::control
{
struct MixedMotorCommand
{
  float left = 0.0F;
  float right = 0.0F;
};

class MotorMixer
{
public:
  static MixedMotorCommand mix(float balanceCommand, float turnCommand);
};
} // namespace balance_car::control
