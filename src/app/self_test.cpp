#include "app/self_test.h"

namespace balance_car::app
{
SelfTest::SelfTest(drivers::MotorDriver &motorDriver, drivers::EncoderDriver &encoderDriver,
                   drivers::ImuDriver &imuDriver)
    : _motorDriver(motorDriver), _encoderDriver(encoderDriver), _imuDriver(imuDriver)
{
}

SelfTestReport SelfTest::run()
{
  SelfTestReport report = {};
  report.motorDriverReady = _motorDriver.begin();
  _motorDriver.setEnabled(false);
  report.encodersReady = _encoderDriver.begin();
  report.imuReady = _imuDriver.begin();
  if (report.imuReady)
  {
    report.imuCalibrated = _imuDriver.calibrateGyroscope();
    report.imuModel = _imuDriver.model();
    report.imuAddress = _imuDriver.address();
  }

  report.passed = report.motorDriverReady && report.encodersReady && report.imuReady && report.imuCalibrated;
  return report;
}

void SelfTest::printReport(Stream &output, const SelfTestReport &report)
{
  output.print("[SELFTEST] MOTOR=");
  output.println(report.motorDriverReady ? "READY" : "FAIL");
  output.print("[SELFTEST] ENCODERS=");
  output.println(report.encodersReady ? "READY" : "FAIL");
  output.print("[SELFTEST] IMU=");
  output.println(report.imuReady ? "READY" : "FAIL");
  if (report.imuReady)
  {
    output.print("[SELFTEST] IMU_MODEL=");
    switch (report.imuModel)
    {
    case drivers::ImuModel::Mpu6050:
      output.println("MPU6050");
      break;
    case drivers::ImuModel::Mpu6500:
      output.println("MPU6500");
      break;
    default:
      output.println("UNKNOWN");
      break;
    }
    output.print("[SELFTEST] IMU_ADDRESS=0x");
    output.println(report.imuAddress, HEX);
  }
  output.print("[SELFTEST] GYRO_CALIBRATION=");
  output.println(report.imuCalibrated ? "PASS" : "FAIL");
  output.print("[SELFTEST] RESULT=");
  output.println(report.passed ? "PASS" : "FAIL");
}
} // namespace balance_car::app
