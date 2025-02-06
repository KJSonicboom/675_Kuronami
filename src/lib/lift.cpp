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

      motor->move_absolute(ZERO, 35);  

      break;
    
    case LiftState::Scoring:

      motor->move_absolute(UP_ANGLE, 110);

      break;
    

    case LiftState::AllianceStake:

      motor->move_absolute(ALLIANCE_ANGLE, 110);

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
float Lift::getAngle() { return target; }