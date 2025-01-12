#include "lib/chassis.h"
#include "pros/rtos.hpp"
#include "robodash/views/console.hpp"
#include "robotconfig.h"
#include "util.h"
#include <cstdint>
#include <string>

using namespace lib;

void Chassis::move(float target, PID linearPid, PID headingPid, float maxSpeed,
                   bool async, bool fast) {

  if (async) {
    while (this->getState() == DriveState::MOVING) {
      pros::delay(10);
    }
    pros::Task task([&]() { move(target, linearPid, headingPid, maxSpeed); });
  }

  float startPos = track->getDistance();
  float startTime = pros::millis();
  double startHeading = headingTarget;

  linearPid.target_set(target);
  headingPid.target_set(startHeading);

  state = DriveState::MOVING;

  while (linearPid.exit_condition() == exit_output::RUNNING && true) {

    double distance = track->getDistance();
    double heading = imu->get_rotation();

    double linearError = startPos + target - distance;
    double headingError = constrain180(startHeading - heading);

    float linearOutput =
        linearPid.compute_error(linearError, distance - startPos);
    float headingOutput =
        headingPid.compute_error(headingError, constrain180(heading));

    // if linearoutput + abs(headingOutput) > maxSpeed or 127, scale down
    // linearOutput
    if (std::abs(linearOutput) + std::abs(headingOutput) > maxSpeed) {
      linearOutput =
          (linearOutput > 0)
              ? fmin(linearOutput, maxSpeed - std::abs(headingOutput))
              : fmax(linearOutput, -maxSpeed + std::abs(headingOutput));
    }

    arcade(linearOutput, headingOutput);

    // if we went past the target
    if (fast) {
      if (fabs(distance - startPos) > fabs(target)) {
        break;
      }
    }

    pros::delay(10);
  }

  // reset pid
  std::string str = std::to_string(chassis.getPose().x) + " " +
                    std::to_string(chassis.getPose().y) + " " +
                    std::to_string(chassis.getPose().theta) + "\n";
  // console.println(str);
  linearPid.variables_reset();
  headingPid.variables_reset();
  leftMotors->move(0);
  rightMotors->move(0);
  leftMotors->brake();
  rightMotors->brake();
  state = DriveState::IDLE;
}



void Chassis::turn(double target, PID turningPid, float maxSpeed, bool async, bool reflectManually, bool fast) {

  if (team == 2 and reflectManually) {
    target = -target;
  }

  headingTarget = target;

  if (async) {
    while (this->getState() == DriveState::MOVING) {
      pros::delay(20);
    }
    pros::Task task([&]() { turn(target, turningPid, maxSpeed); });
  }

  turningPid.target_set(target);

  state = DriveState::MOVING;

  while (turningPid.exit_condition() == exit_output::RUNNING) {

    double headingError = constrain180(target - imu->get_rotation());

    float output = turningPid.compute_error(headingError,
                                            constrain180(imu->get_rotation()));
    output = (output > 0) ? fmin(output, maxSpeed) : fmax(output, -maxSpeed);

    leftMotors->move(output);
    rightMotors->move(-output);

    // if we went past the target
    //if (fast) {
    //  if (fabs(constrain180(imu->get_rotation())) > fabs(target)) {
    //    break;
    //  }
    //}

    pros::delay(10);
  }

  // reset pid
  std::string str = std::to_string(chassis.getPose().x) + " " +
                    std::to_string(chassis.getPose().y) + " " +
                    std::to_string(chassis.getPose().theta) + "\n";
  // console.println(str);
  leftMotors->brake();
  rightMotors->brake();
  turningPid.variables_reset();
  turningPid.variables_reset();
  leftMotors->move(0);
  rightMotors->move(0);

  state = DriveState::IDLE;
}



void Chassis::swing(double target, bool side, float multiplier, PID turningPid, float maxSpeed, bool async, bool reflectManually, bool fast) {

  if (team == 2) {
    target = -target;
    side = !side;
  }

  headingTarget = target;

  if (async) {
    while (this->getState() == DriveState::MOVING) {
      pros::delay(20);
    }
    pros::Task task([&]() { turn(target, turningPid, maxSpeed); });
  }

  turningPid.target_set(target);

  state = DriveState::MOVING;

  while (turningPid.exit_condition() == exit_output::RUNNING) {

    double headingError = constrain180(target - imu->get_rotation());

    float output = turningPid.compute_error(headingError,
                                            constrain180(imu->get_rotation()));
    output = (output > 0) ? fmin(output, maxSpeed) : fmax(output, -maxSpeed);

    if (side) {
      leftMotors->move(output * multiplier);
      rightMotors->move(-output);
    } else {
      leftMotors->move(output);
      rightMotors->move(-output * multiplier);
    }

    //if (fast) {
    //  if (fabs(constrain180(imu->get_rotation())) > fabs(target)) {
    //    break;
    //  }
    //}

    pros::delay(10);
  }

  // reset pid
  std::string str = std::to_string(chassis.getPose().x) + " " +
                    std::to_string(chassis.getPose().y) + " " +
                    std::to_string(chassis.getPose().theta) + "\n";
  // console.println(str);
  leftMotors->brake();
  rightMotors->brake();
  turningPid.variables_reset();
  turningPid.variables_reset();
  leftMotors->move(0);
  rightMotors->move(0);

  state = DriveState::IDLE;
}

void Chassis::waitUntilFinished() {
  while (state == DriveState::MOVING) {
    pros::delay(10);
  }
}