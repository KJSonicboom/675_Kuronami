#include "lib/chassis.h"
#include "point.hpp"
#include "pros/rtos.hpp"
#include "robodash/views/console.hpp"
#include "robotconfig.h"
#include "util.h"
#include <string>
#include <utility>

using namespace lib;

void Chassis::moveToPoint(float x, float y, PID linearPid, PID turningPid, bool backwards, float maxSpeed, bool async, bool fast) {

  if (async) {
    while (this->getState() == DriveState::MOVING) {
      pros::delay(20);
    }
    pros::Task task([&]() {
      moveToPoint(x, y, linearPid, turningPid, backwards, maxSpeed, false, fast);
    });
  }

  if (team == 2) {
    x = -x;
  }

  state = DriveState::MOVING;

  // turn to the point
  double angle = fmod(
      radiansToDegrees(M_PI_2 - atan2(y - getPose().y, x - getPose().x)), 360);
  std::cout << angle << "\n";

  if (backwards) {
    angle += 180;
  }
  turn(angle, turningPid, maxSpeed, false, false, fast);

  // move to the point
  float distance = sqrt(pow(x - getPose().x, 2) + pow(y - getPose().y, 2));
  std::cout << distance << "\n";

  if (backwards) {
    distance = -distance;
  }
  move(distance, linearPid, turningPid, maxSpeed, false, fast);

  state = DriveState::IDLE;
}

void Chassis::followTrajectory(const std::vector<ProfilePoint> &trajectory,
                               bool backwards, double maxSpeed, double b,
                               double zeta, bool async) {
  state = DriveState::MOVING;
  std::cout << "Starting trajectory following (backwards = "
            << (backwards ? "true" : "false") << ")\n";

  double startDistance = track->getDistance();
  size_t trajectoryIndex = 1;

  while (trajectoryIndex < trajectory.size()) {
    Point currentPose = getPose();
    double distanceTraveled = std::abs(track->getDistance() - startDistance);

    std::cout << "Current pose: x=" << currentPose.x << " y=" << currentPose.y
              << " theta=" << currentPose.theta << "\n";

    while (distanceTraveled > trajectory[trajectoryIndex].distance) {
      trajectoryIndex++;
      if (trajectoryIndex >= trajectory.size())
        break;
    }

    const ProfilePoint &desired = trajectory[trajectoryIndex];

    std::cout << "Profile velocities: left=" << desired.leftVelocity
              << " right=" << desired.rightVelocity << "\n";

    double leftVel = desired.leftVelocity;
    double rightVel = desired.rightVelocity;

    if (backwards) {
      leftVel = -leftVel;
      rightVel = -rightVel;
    }

    int leftMotorVel = static_cast<int>((leftVel / maxSpeed) * 600);
    int rightMotorVel = static_cast<int>((rightVel / maxSpeed) * 600);

    std::cout << "Motor velocities: left=" << leftMotorVel
              << " right=" << rightMotorVel << "\n\n";

    leftMotors->move_velocity(leftMotorVel);
    rightMotors->move_velocity(rightMotorVel);
    pros::delay(25);
  }

  leftMotors->move(0);
  rightMotors->move(0);
  leftMotors->brake();
  rightMotors->brake();
  state = DriveState::IDLE;
}

void Chassis::moveToPose(Point target, PID linearPid, PID angularPid, bool backwards, int maxSpeed, bool async) {
  if (async) {
    while (this->getState() == DriveState::MOVING) {
      pros::delay(20);
    }
    pros::Task task(
        [&]() { moveToPose(target, linearPid, angularPid, maxSpeed, false); });
    return;
  }


  Point targetPoint = target;
  if (team == 2) {
    target.x = -target.x;
  }
  double startDistance = track->getDistance();
  double prevLateralError = 0;
  state = DriveState::MOVING;


  while (linearPid.exit_condition() == exit_output::RUNNING) {
    Point currentPosition = getPose();


    // update error
    float deltaX = targetPoint.x - currentPosition.x;
    float deltaY = targetPoint.y - currentPosition.y;
    float targetTheta = fmod(radiansToDegrees(M_PI_2 - atan2(deltaY, deltaX)), 360);
    float hypot = std::hypot(deltaX, deltaY);
    float diffTheta1 = angleError(currentPosition.theta, targetTheta);
    float diffTheta2 = angleError(currentPosition.theta, targetTheta + 180);
    float angularError = (std::fabs(diffTheta1) < std::fabs(diffTheta2))
                             ? diffTheta1
                             : diffTheta2;
    float lateralError = hypot * cos(degreesToRadians(std::fabs(diffTheta1)));


    double linearOut = linearPid.compute_error(lateralError, track->getDistance() - startDistance);
    double angularOut = angularPid.compute_error(angularError, constrain180(currentPosition.theta));

    if (backwards) {linearOut = -linearOut;}

    if (lateralError <= 7) {angularOut = 0;}

    // Calculate motor powers
    float leftPower = linearOut + angularOut;
    float rightPower = linearOut - angularOut;

     // ratio the speeds to respect the max speed
        leftPower = linearOut + angularOut;
        rightPower = linearOut - angularOut;
        const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / 127;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

    // Move motors
    leftMotors->move(leftPower);
    rightMotors->move(rightPower);

    //if(lateralError < 7 && std::abs(prevLateralError) - std::abs(lateralError) < 0){break;}

    prevLateralError = lateralError;

    pros::delay(15);
  }

  // Stop and brake
  leftMotors->move(0);
  rightMotors->move(0);
  leftMotors->brake();
  rightMotors->brake();
  state = DriveState::IDLE;
}