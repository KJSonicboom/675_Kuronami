#include "lib/lift.hpp"
#include "pros/abstract_motor.hpp"
#include "robotconfig.h"
#include <cstdint>
#include <iostream>

using namespace lib;

void Lift::loop() {

  uint32_t now = pros::millis();
  motor->tare_position();
  while (true) {
    switch (getState()) {

    case LiftState::Disabled:

      target = DOWN_ANGLE;      

      break;

    case LiftState::Scoring:

      target = UP_ANGLE;

      break;
    }

    // float error = abs(target - motor->get_position());
    // target == UP_ANGLE ? motor->move(error * p + (error - prev_error) * d) : motor->move(-error * 1 + (error - prev_error) * d); //may need to force this into a negative position
    // prev_error = error;

    

    pros::Task::delay_until(&now, 15); //no understand
  }
}

float Lift::getAngle() { return target; }