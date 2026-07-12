#include "drivers/imu_driver.h"

#include <math.h>

namespace balance_car::drivers
{
namespace
{
constexpr uint8_t kMpu6050AddressLow = 0x68;
constexpr uint8_t kMpu6050AddressHigh = 0x69;
constexpr uint8_t kWhoAmIRegister = 0x75;
constexpr uint8_t kSampleRateDividerRegister = 0x19;
constexpr uint8_t kConfigurationRegister = 0x1A;
constexpr uint8_t kGyroConfigurationRegister = 0x1B;
constexpr uint8_t kAccelerometerConfigurationRegister = 0x1C;
constexpr uint8_t kPowerManagementRegister = 0x6B;
constexpr uint8_t kAccelerometerDataRegister = 0x3B;
constexpr float kAccelerometerScaleLsbPerG = 8192.0F;
constexpr float kGyroscopeScaleLsbPerDps = 65.5F;
}

ImuDriver::ImuDriver(hal::I2cBus &bus, const config::ImuPins &pins,
                     const config::ImuConfiguration &configuration)
    : _bus(bus), _pins(pins), _configuration(configuration)
{
}

bool ImuDriver::begin()
{
  _initialized = false;
  _calibrated = false;
  _consecutiveFailures = 0;
  if (!_bus.begin(_pins.sda, _pins.scl, _configuration.i2cFrequencyHz))
  {
    return false;
  }

  const uint8_t candidateAddresses[] = {kMpu6050AddressLow, kMpu6050AddressHigh};
  for (uint8_t candidateAddress : candidateAddresses)
  {
    uint8_t whoAmI = 0;
    if (!_bus.probe(candidateAddress) ||
        !_bus.readRegisters(candidateAddress, kWhoAmIRegister, &whoAmI, 1))
    {
      continue;
    }

    const ImuModel detectedModel = identifyModel(whoAmI);
    if (detectedModel == ImuModel::Unknown)
    {
      continue;
    }

    _address = candidateAddress;
    _model = detectedModel;
    _initialized = configureSensor();
    return _initialized;
  }

  return false;
}

bool ImuDriver::calibrateGyroscope()
{
  if (!_initialized)
  {
    return false;
  }

  float gyroXSum = 0.0F;
  float gyroYSum = 0.0F;
  float gyroZSum = 0.0F;
  float gyroXSquareSum = 0.0F;
  float gyroYSquareSum = 0.0F;
  float gyroZSquareSum = 0.0F;

  for (uint16_t index = 0; index < _configuration.calibrationSamples; ++index)
  {
    RawImuData rawData = {};
    if (!readRaw(rawData))
    {
      _calibrated = false;
      return false;
    }

    const float gyroXDps = static_cast<float>(rawData.gyroX) / kGyroscopeScaleLsbPerDps;
    const float gyroYDps = static_cast<float>(rawData.gyroY) / kGyroscopeScaleLsbPerDps;
    const float gyroZDps = static_cast<float>(rawData.gyroZ) / kGyroscopeScaleLsbPerDps;
    gyroXSum += gyroXDps;
    gyroYSum += gyroYDps;
    gyroZSum += gyroZDps;
    gyroXSquareSum += gyroXDps * gyroXDps;
    gyroYSquareSum += gyroYDps * gyroYDps;
    gyroZSquareSum += gyroZDps * gyroZDps;
    delay(_configuration.calibrationIntervalMs);
  }

  const float sampleCount = static_cast<float>(_configuration.calibrationSamples);
  _gyroXOffsetDps = gyroXSum / sampleCount;
  _gyroYOffsetDps = gyroYSum / sampleCount;
  _gyroZOffsetDps = gyroZSum / sampleCount;

  const float gyroXVariance = fmaxf(0.0F, gyroXSquareSum / sampleCount - _gyroXOffsetDps * _gyroXOffsetDps);
  const float gyroYVariance = fmaxf(0.0F, gyroYSquareSum / sampleCount - _gyroYOffsetDps * _gyroYOffsetDps);
  const float gyroZVariance = fmaxf(0.0F, gyroZSquareSum / sampleCount - _gyroZOffsetDps * _gyroZOffsetDps);
  const float maximumStdDev = fmaxf(sqrtf(gyroXVariance), fmaxf(sqrtf(gyroYVariance), sqrtf(gyroZVariance)));
  _calibrated = maximumStdDev <= _configuration.maximumStationaryGyroStdDevDps;
  return _calibrated;
}

ImuSample ImuDriver::read()
{
  ImuSample sample = {};
  if (!_initialized)
  {
    return sample;
  }

  RawImuData rawData = {};
  if (!readRaw(rawData))
  {
    ++_consecutiveFailures;
    return sample;
  }

  _consecutiveFailures = 0;
  _lastSampleMs = millis();
  sample.accelXG = static_cast<float>(rawData.accelX) / kAccelerometerScaleLsbPerG;
  sample.accelYG = static_cast<float>(rawData.accelY) / kAccelerometerScaleLsbPerG;
  sample.accelZG = static_cast<float>(rawData.accelZ) / kAccelerometerScaleLsbPerG;
  sample.gyroXDps = static_cast<float>(rawData.gyroX) / kGyroscopeScaleLsbPerDps - _gyroXOffsetDps;
  sample.gyroYDps = static_cast<float>(rawData.gyroY) / kGyroscopeScaleLsbPerDps - _gyroYOffsetDps;
  sample.gyroZDps = static_cast<float>(rawData.gyroZ) / kGyroscopeScaleLsbPerDps - _gyroZOffsetDps;
  sample.timestampMs = _lastSampleMs;
  sample.valid = true;
  return sample;
}

bool ImuDriver::isInitialized() const
{
  return _initialized;
}

bool ImuDriver::isCalibrated() const
{
  return _calibrated;
}

bool ImuDriver::isHealthy() const
{
  return _initialized && _consecutiveFailures < 3 &&
         millis() - _lastSampleMs <= _configuration.maximumSampleAgeMs;
}

ImuModel ImuDriver::model() const
{
  return _model;
}

uint8_t ImuDriver::address() const
{
  return _address;
}

const char *ImuDriver::modelName() const
{
  switch (_model)
  {
  case ImuModel::Mpu6050:
    return "MPU6050";
  case ImuModel::Mpu6500:
    return "MPU6500";
  default:
    return "UNKNOWN";
  }
}

bool ImuDriver::configureSensor()
{
  return _bus.writeRegister(_address, kPowerManagementRegister, 0x00) &&
         _bus.writeRegister(_address, kConfigurationRegister, 0x03) &&
         _bus.writeRegister(_address, kSampleRateDividerRegister, 0x01) &&
         _bus.writeRegister(_address, kGyroConfigurationRegister, 0x08) &&
         _bus.writeRegister(_address, kAccelerometerConfigurationRegister, 0x08);
}

bool ImuDriver::readRaw(RawImuData &rawData)
{
  uint8_t buffer[14] = {};
  if (!_bus.readRegisters(_address, kAccelerometerDataRegister, buffer, sizeof(buffer)))
  {
    return false;
  }

  rawData.accelX = combineBytes(buffer[0], buffer[1]);
  rawData.accelY = combineBytes(buffer[2], buffer[3]);
  rawData.accelZ = combineBytes(buffer[4], buffer[5]);
  rawData.gyroX = combineBytes(buffer[8], buffer[9]);
  rawData.gyroY = combineBytes(buffer[10], buffer[11]);
  rawData.gyroZ = combineBytes(buffer[12], buffer[13]);
  return true;
}

int16_t ImuDriver::combineBytes(uint8_t highByte, uint8_t lowByte)
{
  return static_cast<int16_t>((static_cast<uint16_t>(highByte) << 8U) | lowByte);
}

ImuModel ImuDriver::identifyModel(uint8_t whoAmI)
{
  if (whoAmI == 0x68 || whoAmI == 0x69)
  {
    return ImuModel::Mpu6050;
  }
  if (whoAmI == 0x70 || whoAmI == 0x71)
  {
    return ImuModel::Mpu6500;
  }
  return ImuModel::Unknown;
}
} // namespace balance_car::drivers
