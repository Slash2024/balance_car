#include "app/safety_manager.h"

namespace balance_car::app
{
SafetyManager::SafetyManager(drivers::MotorDriver &motorDriver, const config::SafetyConfiguration &configuration)
    : _motorDriver(motorDriver), _configuration(configuration)
{
}

void SafetyManager::begin()
{
  _motorDriver.setEnabled(false);
  _state = SafetyState::SelfTesting;
  _faultCode = FaultCode::None;
}

void SafetyManager::completeSelfTest(const SelfTestReport &report)
{
  if (!report.passed)
  {
    reportFault(FaultCode::SelfTestFailed);
    return;
  }

  _motorDriver.setEnabled(false);
  _state = SafetyState::Standby;
  _faultCode = FaultCode::None;
}

bool SafetyManager::requestManualMotorTest(float leftPower, float rightPower, uint32_t nowMs)
{
  if (_state != SafetyState::Standby && _state != SafetyState::ManualTest)
  {
    return false;
  }

  _motorDriver.setEnabled(true);
  _motorDriver.setNormalized(leftPower, rightPower);
  _manualTestExpiresAtMs = nowMs + _configuration.manualTestDurationMs;
  _state = SafetyState::ManualTest;
  return true;
}

void SafetyManager::disarm()
{
  _motorDriver.setEnabled(false);
  _manualTestExpiresAtMs = 0;
  if (_state != SafetyState::Fault)
  {
    _state = SafetyState::Standby;
  }
}

void SafetyManager::reportFault(FaultCode faultCode)
{
  _motorDriver.setEnabled(false);
  _manualTestExpiresAtMs = 0;
  _faultCode = faultCode;
  _state = SafetyState::Fault;
}

void SafetyManager::update(uint32_t nowMs)
{
  if (_state == SafetyState::ManualTest && static_cast<int32_t>(nowMs - _manualTestExpiresAtMs) >= 0)
  {
    disarm();
  }
}

SafetyState SafetyManager::state() const
{
  return _state;
}

FaultCode SafetyManager::faultCode() const
{
  return _faultCode;
}

const char *SafetyManager::stateName(SafetyState state)
{
  switch (state)
  {
  case SafetyState::Boot:
    return "BOOT";
  case SafetyState::SelfTesting:
    return "SELF_TESTING";
  case SafetyState::Standby:
    return "STANDBY";
  case SafetyState::ManualTest:
    return "MANUAL_TEST";
  case SafetyState::Fault:
    return "FAULT";
  }
  return "UNKNOWN";
}

const char *SafetyManager::faultName(FaultCode faultCode)
{
  switch (faultCode)
  {
  case FaultCode::None:
    return "NONE";
  case FaultCode::SelfTestFailed:
    return "SELF_TEST_FAILED";
  case FaultCode::ImuUnhealthy:
    return "IMU_UNHEALTHY";
  }
  return "UNKNOWN";
}
} // namespace balance_car::app
