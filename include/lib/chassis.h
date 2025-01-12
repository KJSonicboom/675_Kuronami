#pragma once
#include "lib/TaskWrapper.hpp"
#include "lib/point.hpp"
#include "lib/pid.h"
#include "lib/trackwheel.h"
#include "pros/motors.h"
#include <memory>
#include "trajectory.hpp"
namespace lib {

class Chassis : public ryan::TaskWrapper{

private:
  enum class DriveState { IDLE, MOVING };
  std::shared_ptr<pros::MotorGroup> leftMotors;
  std::shared_ptr<pros::MotorGroup> rightMotors;
  std::shared_ptr<pros::Imu> imu;
  std::shared_ptr<lib::TrackingWheel> track;

  Point currentPose = Point(0, 0, 0);

  DriveState state;
  float headingTarget;

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
  int team = 0;

  DriveState getState() { return state; }

  // constructor
  Chassis(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, pros::Imu *imu, lib::TrackingWheel *track, int rpm, double wheel)
  : leftMotors(leftMotors), rightMotors(rightMotors), imu(imu), track(track), rpm(rpm), wheel(wheel) 
  {
    leftMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    rightMotors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    state = DriveState::IDLE;
  }

  double sinc(double x);

  
  // tracking
  void loop() override;
  Point getPose(bool radians = false);
  void setPose(Point newPose, bool radians = false);

  // driver functions
  double inputCurve(int input, double t = 1);
  void arcadeMod(double forward, double turn, std::vector<double> curves, int speedThreshold);
  void arcade(double forward, double turn, std::vector<double> curves);
  void arcade(double forward, double turn);
  void tank(double left, double right, std::vector<double> curves = {0, 0});



  void waitUntilFinished();



  // relative pid movements
  void move(float target, PID linearPid, PID headingPid, float maxSpeed = 110, bool async = false, bool fast = false);

  void turn(double target, PID turningPid, float maxSpeed = 127, bool async = false, bool reflectManually = false, bool fast = false);

  void swing(double target, bool side, float multiplier, PID turningPid, float maxSpeed = 127, bool async = false, bool reflectManually = false, bool fast = false);
 
  // odom movements
  void moveToPoint(float x, float y, PID linearPid, PID headingPid, bool backwards = false, float maxSpeed = 127, bool async = false, bool fast = false);
  void followTrajectory(const std::vector<ProfilePoint>& trajectory, bool backwards = false, double max_speed = MAX_DRIVE_SPEED, double b = 1.5, double zeta = 0.8, bool async = false);
  void moveToPose(Point target, PID linearPid, PID angularPid, bool backwards = false, int maxSpeed = 127, bool async = false);
};
} // namespace lib