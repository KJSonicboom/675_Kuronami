#pragma once
#include "lib/TaskWrapper.hpp"
#include "pros/motors.h"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include <memory>
namespace lib {

class Chassis : public ryan::TaskWrapper{

private:
  enum class DriveState { IDLE, MOVING };
  std::shared_ptr<pros::MotorGroup> leftMotors;
  std::shared_ptr<pros::MotorGroup> rightMotors;

  DriveState state;

  const int rpm;
  const double wheel;
  static constexpr float MAX_DRIVE_SPEED = 64.8;

  double angleWrap(double angle) {
    while (angle > 360) {
      angle -= 360;
    }
    while (angle < -360) {
      angle += 360;
    }
    return angle;
  }

public:

  DriveState getState() { return state; }

  // constructor
  Chassis(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, int rpm, double wheel)
  : leftMotors(leftMotors), rightMotors(rightMotors), rpm(rpm), wheel(wheel) 
  {
    leftMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    rightMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    state = DriveState::IDLE;
  }

  double sinc(double x);

  void loop() override;

  // driver functions
  double inputCurve(int input, double t = 1);
  void arcadeMod(double forward, double turn, std::vector<double> curves, int speedThreshold);

  void waitUntilFinished();

};
} // namespace lib