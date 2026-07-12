#pragma once

#include "config/board_pins.h"
#include "config/vehicle_config.h"
#include "hal/i2c_bus.h"

namespace balance_car::drivers
{
enum class ImuModel
{
  Unknown,
  Mpu6050,
  Mpu6500,
};

struct ImuSample
{
  float accelXG;
  float accelYG;
  float accelZG;
  float gyroXDps;
  float gyroYDps;
  float gyroZDps;
  uint32_t timestampMs;
  bool valid;
};

class ImuDriver
{
public:
  ImuDriver(hal::I2cBus &bus, const config::ImuPins &pins, const config::ImuConfiguration &configuration);

  bool begin();
  bool calibrateGyroscope();
  ImuSample read();
  bool isInitialized() const;
  bool isCalibrated() const;
  bool isHealthy() const;
  ImuModel model() const;
  uint8_t address() const;
  const char *modelName() const;

private:
  struct RawImuData
  {
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
  };

  bool configureSensor();
  bool readRaw(RawImuData &rawData);
  static int16_t combineBytes(uint8_t highByte, uint8_t lowByte);
  static ImuModel identifyModel(uint8_t whoAmI);

  hal::I2cBus &_bus;
  const config::ImuPins &_pins;
  const config::ImuConfiguration &_configuration;
  ImuModel _model = ImuModel::Unknown;
  uint8_t _address = 0;
  float _gyroXOffsetDps = 0.0F;
  float _gyroYOffsetDps = 0.0F;
  float _gyroZOffsetDps = 0.0F;
  uint32_t _lastSampleMs = 0;
  uint8_t _consecutiveFailures = 0;
  bool _initialized = false;
  bool _calibrated = false;
};
} // namespace balance_car::drivers
