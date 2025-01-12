#include "lib/chassis.h"



using namespace lib;



//Chassis with only vertical tracking wheel and imu
void Chassis::loop() {

  double lastAngle = imu->get_rotation() * M_PI / 180.0;
  double lastPosition = track->getDistance();

  while (true) {

    double rawAngle = imu->get_rotation() * M_PI / 180.0;
    double rawPosition = track->getDistance();

    // Calculate the change in sensor values
    float deltaVertical = rawPosition - lastPosition;
    float deltaImu = rawAngle - lastAngle;

    // Update the previous sensor values
    lastPosition = rawPosition;
    lastAngle = rawAngle;

    // Calculate global x and y
    currentPose.x += deltaVertical * sin(rawAngle);
    currentPose.y += deltaVertical * cos(rawAngle);
    currentPose.theta = rawAngle;

    //delay
    pros::delay(10);
  }
}

Point Chassis::getPose(bool radians) { 
  if (radians) {return currentPose;}
  else {return Point(currentPose.x, currentPose.y, currentPose.theta * 180.0 / M_PI); }
}



void Chassis::setPose(Point newPose, bool radians) {
  if (team == 2) {
    newPose.x = -newPose.x;
    newPose.theta = -newPose.theta;
  }
  currentPose = newPose;
  if (!radians) {currentPose.theta *= M_PI / 180.0;}
  headingTarget = newPose.theta;
}