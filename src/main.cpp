#include <Arduino.h>

#include "app/safety_manager.h"
#include "app/self_test.h"
#include "app/vision_debug.h"
#include "config/board_pins.h"
#include "config/vehicle_config.h"
#include "drivers/encoder_driver.h"
#include "drivers/imu_driver.h"
#include "drivers/motor_driver.h"
#include "drivers/vision_i2c_client.h"
#include "hal/i2c_bus.h"

namespace
{
constexpr uint32_t kImuReadPeriodMs = 20;
constexpr uint32_t kEncoderSamplePeriodMs = 100;
constexpr uint32_t kTelemetryPeriodMs = 500;

balance_car::hal::I2cBus i2cBus(Wire); // IMU 总线：GPIO8/GPIO10。
TwoWire visionWire(1);                 // 相机视觉总线：GPIO1/GPIO2。
balance_car::drivers::MotorDriver motorDriver(
    balance_car::config::kMotorPins, balance_car::config::kMotorConfiguration);
balance_car::drivers::EncoderDriver encoderDriver(
    balance_car::config::kEncoderPins, balance_car::config::kEncoderConfiguration);
balance_car::drivers::ImuDriver imuDriver(
    i2cBus, balance_car::config::kImuPins, balance_car::config::kImuConfiguration);
balance_car::drivers::VisionI2cClient visionI2cClient(
    visionWire, balance_car::config::kVisionI2cPins);
balance_car::app::VisionDebug visionDebug(visionI2cClient);
balance_car::app::SelfTest selfTest(motorDriver, encoderDriver, imuDriver);
balance_car::app::SafetyManager safetyManager(
    motorDriver, balance_car::config::kSafetyConfiguration);

balance_car::drivers::ImuSample latestImuSample = {};
balance_car::drivers::WheelSpeed latestWheelSpeed = {};
uint32_t lastImuReadMs = 0;
uint32_t lastEncoderSampleMs = 0;
uint32_t lastTelemetryMs = 0;

void printHelp()
{
  Serial.println("[DIAG] h=help s=status x=stop 1=left+ 2=left- 3=right+ 4=right- 5=both+ 6=both-");
  Serial.println("[DIAG] Motor tests use low power and stop automatically after one second.");
}

void printStatus()
{
  Serial.print("[STATUS] STATE=");
  Serial.print(balance_car::app::SafetyManager::stateName(safetyManager.state()));
  Serial.print(" FAULT=");
  Serial.println(balance_car::app::SafetyManager::faultName(safetyManager.faultCode()));
}

void requestMotorTest(float leftPower, float rightPower)
{
  if (safetyManager.requestManualMotorTest(leftPower, rightPower, millis()))
  {
    Serial.println("[DIAG] MOTOR_TEST=ACTIVE");
    return;
  }
  Serial.println("[DIAG] MOTOR_TEST=REJECTED");
}

void processSerialCommand()
{
  while (Serial.available() > 0)
  {
    const char command = static_cast<char>(Serial.read());
    switch (command)
    {
    case 'h':
      printHelp();
      break;
    case 's':
      printStatus();
      break;
    case 'x':
    case '0':
      safetyManager.disarm();
      Serial.println("[DIAG] MOTOR_TEST=STOPPED");
      break;
    case '1':
      requestMotorTest(balance_car::config::kSafetyConfiguration.manualTestPower, 0.0F);
      break;
    case '2':
      requestMotorTest(-balance_car::config::kSafetyConfiguration.manualTestPower, 0.0F);
      break;
    case '3':
      requestMotorTest(0.0F, balance_car::config::kSafetyConfiguration.manualTestPower);
      break;
    case '4':
      requestMotorTest(0.0F, -balance_car::config::kSafetyConfiguration.manualTestPower);
      break;
    case '5':
      requestMotorTest(balance_car::config::kSafetyConfiguration.manualTestPower,
                       balance_car::config::kSafetyConfiguration.manualTestPower);
      break;
    case '6':
      requestMotorTest(-balance_car::config::kSafetyConfiguration.manualTestPower,
                       -balance_car::config::kSafetyConfiguration.manualTestPower);
      break;
    default:
      break;
    }
  }
}

void updateSensors(uint32_t nowMs)
{
  if (nowMs - lastImuReadMs >= kImuReadPeriodMs)
  {
    lastImuReadMs = nowMs;
    latestImuSample = imuDriver.read();
    if (!imuDriver.isHealthy())
    {
      safetyManager.reportFault(balance_car::app::FaultCode::ImuUnhealthy);
    }
  }

  if (nowMs - lastEncoderSampleMs >= kEncoderSamplePeriodMs)
  {
    const uint32_t elapsedMs = nowMs - lastEncoderSampleMs;
    lastEncoderSampleMs = nowMs;
    latestWheelSpeed = encoderDriver.sample(static_cast<float>(elapsedMs) / 1000.0F);
  }
}

void printTelemetry(uint32_t nowMs)
{
  if (nowMs - lastTelemetryMs < kTelemetryPeriodMs)
  {
    return;
  }

  lastTelemetryMs = nowMs;
  Serial.print("[IMU] accel_g=");
  Serial.print(latestImuSample.accelXG, 3);
  Serial.print(',');
  Serial.print(latestImuSample.accelYG, 3);
  Serial.print(',');
  Serial.print(latestImuSample.accelZG, 3);
  Serial.print(" gyro_dps=");
  Serial.print(latestImuSample.gyroXDps, 2);
  Serial.print(',');
  Serial.print(latestImuSample.gyroYDps, 2);
  Serial.print(',');
  Serial.print(latestImuSample.gyroZDps, 2);
  Serial.print(" ticks=");
  Serial.print(latestWheelSpeed.leftTicks);
  Serial.print(',');
  Serial.print(latestWheelSpeed.rightTicks);
  Serial.print(" speed_mps=");
  Serial.print(latestWheelSpeed.leftMetersPerSecond, 3);
  Serial.print(',');
  Serial.println(latestWheelSpeed.rightMetersPerSecond, 3);
}
} // namespace

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("===== Balance Car Hardware Self-Test =====");
  safetyManager.begin();
  const balance_car::app::SelfTestReport report = selfTest.run();
  balance_car::app::SelfTest::printReport(Serial, report);
  safetyManager.completeSelfTest(report);
  visionDebug.begin();
  printStatus();
  printHelp();
}

void loop()
{
  const uint32_t nowMs = millis();
  processSerialCommand();
  safetyManager.update(nowMs);
  updateSensors(nowMs);
  visionDebug.update(nowMs);
  printTelemetry(nowMs);
  delay(1);
}
