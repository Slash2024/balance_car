#pragma once

#include <Arduino.h>

namespace balance_car::config
{
struct MotorPins
{
  uint8_t standby;
  uint8_t leftPwm;
  uint8_t leftIn1;
  uint8_t leftIn2;
  uint8_t rightPwm;
  uint8_t rightIn1;
  uint8_t rightIn2;
};

struct EncoderPins
{
  uint8_t leftPhaseA;
  uint8_t leftPhaseB;
  uint8_t rightPhaseA;
  uint8_t rightPhaseB;
};

struct ImuPins
{
  uint8_t sda;
  uint8_t scl;
};

constexpr MotorPins kMotorPins = {
    .standby = 4,
    .leftPwm = 5,
    .leftIn1 = 6,
    .leftIn2 = 7,
    .rightPwm = 15,
    .rightIn1 = 16,
    .rightIn2 = 17,
};

constexpr EncoderPins kEncoderPins = {
    .leftPhaseA = 11,
    .leftPhaseB = 12,
    .rightPhaseA = 13,
    .rightPhaseB = 14,
};

constexpr ImuPins kImuPins = {
    .sda = 8,
    .scl = 10,
};
} // namespace balance_car::config
