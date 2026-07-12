#include "hal/i2c_bus.h"

namespace balance_car::hal
{
I2cBus::I2cBus(TwoWire &wire) : _wire(wire)
{
}

bool I2cBus::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t frequencyHz)
{
  return _wire.begin(sdaPin, sclPin, frequencyHz);
}

bool I2cBus::probe(uint8_t deviceAddress)
{
  _wire.beginTransmission(deviceAddress);
  return _wire.endTransmission() == 0;
}

bool I2cBus::writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value)
{
  _wire.beginTransmission(deviceAddress);
  _wire.write(registerAddress);
  _wire.write(value);
  return _wire.endTransmission() == 0;
}

bool I2cBus::readRegisters(uint8_t deviceAddress, uint8_t startRegister, uint8_t *buffer, size_t length)
{
  if (buffer == nullptr || length == 0)
  {
    return false;
  }

  _wire.beginTransmission(deviceAddress);
  _wire.write(startRegister);
  if (_wire.endTransmission(false) != 0)
  {
    return false;
  }

  const size_t received = _wire.requestFrom(deviceAddress, length, true);
  if (received != length)
  {
    while (_wire.available())
    {
      _wire.read();
    }
    return false;
  }

  for (size_t index = 0; index < length; ++index)
  {
    buffer[index] = static_cast<uint8_t>(_wire.read());
  }
  return true;
}
} // namespace balance_car::hal
