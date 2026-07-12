#include "hal/pwm_channel.h"

namespace balance_car::hal
{
PwmChannel::PwmChannel(uint8_t channel) : _channel(channel)
{
}

bool PwmChannel::begin(uint8_t pin, uint32_t frequencyHz, uint8_t resolutionBits)
{
  if (resolutionBits == 0 || resolutionBits > 16)
  {
    return false;
  }

  if (ledcSetup(_channel, frequencyHz, resolutionBits) == 0.0)
  {
    return false;
  }

  ledcAttachPin(pin, _channel);

  _pin = pin;
  _maximumDuty = (1UL << resolutionBits) - 1UL;
  _initialized = true;
  stop();
  return true;
}

void PwmChannel::writeDuty(uint32_t duty)
{
  if (!_initialized)
  {
    return;
  }

  ledcWrite(_channel, duty > _maximumDuty ? _maximumDuty : duty);
}

void PwmChannel::stop()
{
  writeDuty(0);
}

uint32_t PwmChannel::maximumDuty() const
{
  return _maximumDuty;
}
} // namespace balance_car::hal
