#include "lib/lift.hpp"
#include "pros/abstract_motor.hpp"
#include "robotconfig.h"
#include <cstdint>
#include <iostream>
#include <string>

using namespace lib;

void Lift::loop() {

  uint32_t now = pros::millis();
  
  while (true) {
    switch (getState()) {

    case LiftState::Disabled:

      motor->move_absolute(ZERO, 50);  

      break;

      motor->move_absolute(UP_ANGLE, 100);

      master.set_text(0, 0, std::to_string(motor->get_position()));

      break;
    }
  }
}

lemlib::PID armPID(4, 0, 20);

void Lift::moveArm(int targetDegree, int timeout){
  error = 100;
  lemlib::Timer armTimer(timeout);
  while(abs(error) > 10 && !armTimer.isDone()){
      currentPosition = motor->get_position();
      error = targetDegree - currentPosition; 
      motor->move(armPID.update(error));
  }

  armPID.reset();

}

void Lift::moveVelocity(int velocity){
  motor->move_velocity(velocity);
}

float Lift::getAngle() { return target; }