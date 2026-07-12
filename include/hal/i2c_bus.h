#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace balance_car::hal
{
class I2cBus
{
public:
  explicit I2cBus(TwoWire &wire);

  bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t frequencyHz);
  bool probe(uint8_t deviceAddress);
  bool writeRegister(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value);
  bool readRegisters(uint8_t deviceAddress, uint8_t startRegister, uint8_t *buffer, size_t length);

private:
  TwoWire &_wire;
};
} // namespace balance_car::hal
