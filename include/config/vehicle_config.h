#pragma once

#include <Arduino.h>

namespace balance_car::config
{
struct MotorConfiguration
{
  uint32_t pwmFrequencyHz;
  uint8_t pwmResolutionBits;
  bool leftDirectionInverted;
  bool rightDirectionInverted;
};

struct EncoderConfiguration
{
  float countsPerWheelRevolution;
  float wheelDiameterMeters;
  bool useInternalPullups;
  bool leftDirectionInverted;
  bool rightDirectionInverted;
};

struct ImuConfiguration
{
  uint32_t i2cFrequencyHz;
  uint16_t calibrationSamples;
  uint16_t calibrationIntervalMs;
  float maximumStationaryGyroStdDevDps;
  uint16_t maximumSampleAgeMs;
};

struct SafetyConfiguration
{
  uint16_t manualTestDurationMs;
  float manualTestPower;
};

constexpr MotorConfiguration kMotorConfiguration = {
    .pwmFrequencyHz = 20000,
    .pwmResolutionBits = 10,
    .leftDirectionInverted = false,
    .rightDirectionInverted = false,
};

constexpr EncoderConfiguration kEncoderConfiguration = {
    .countsPerWheelRevolution = 780.0F,
    .wheelDiameterMeters = 0.064F,
    .useInternalPullups = true,
    .leftDirectionInverted = false,
    .rightDirectionInverted = false,
};

constexpr ImuConfiguration kImuConfiguration = {
    .i2cFrequencyHz = 400000,
    .calibrationSamples = 500,
    .calibrationIntervalMs = 2,
    .maximumStationaryGyroStdDevDps = 3.0F,
    .maximumSampleAgeMs = 40,
};

constexpr SafetyConfiguration kSafetyConfiguration = {
    .manualTestDurationMs = 1000,
    .manualTestPower = 0.15F,
};
} // namespace balance_car::config
