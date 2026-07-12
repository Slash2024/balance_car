#include "control/motor_mixer.h"

#include <math.h>

namespace balance_car::control
{
MixedMotorCommand MotorMixer::mix(float balanceCommand, float turnCommand)
{
  MixedMotorCommand mixedCommand;
  mixedCommand.left = balanceCommand - turnCommand;
  mixedCommand.right = balanceCommand + turnCommand;
  const float maximumMagnitude = fmaxf(fabsf(mixedCommand.left), fabsf(mixedCommand.right));
  if (maximumMagnitude > 1.0F)
  {
    mixedCommand.left /= maximumMagnitude;
    mixedCommand.right /= maximumMagnitude;
  }
  return mixedCommand;
}
} // namespace balance_car::control
