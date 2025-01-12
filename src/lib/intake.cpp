#include "lib/intake.hpp"
#include "pros/rtos.hpp"
#include "robotconfig.h"


using namespace lib;

void Intake::loop() {

  int jamTimer = 0;
  uint32_t jamStartTime = 0;

  while (true) {
    // Intake jam logic
    if (std::abs(motor->get_actual_velocity()) < 5 && getState() != IntakeState::Idle) {
      if (jamStartTime == 0) {
        // First time detecting slow velocity
        jamStartTime = pros::millis();
      }

      // Calculate how long we've been in slow velocity state
      uint32_t jamTimer = pros::millis() - jamStartTime;

      if (armLoading) {
        if (jamTimer > 1000) {
          setState(IntakeState::Jam);
        }
      } else {
        if (jamTimer > 75) {
          setState(IntakeState::Jam);
        }
      }
    } else {
      // Reset the start time when velocity is normal
      jamStartTime = 0;
      jamTimer = 0;
    }

    switch (getState()) {

    case IntakeState::Idle:

      color->set_led_pwm(0);

      motor->move(0);

      break;

    case IntakeState::Eat:

      color->set_led_pwm(0);

      motor->move(127);

      break;

    case IntakeState::Reload:

      color->set_led_pwm(100);

      motor->move(100);

      if(detectRingColor() == teamColor){
        motor->tare_position();
        while(220 > abs(motor->get_position())){
          motor->move(-72);
        }
        motor->move_absolute(110, 72);
      }

      break;

    case IntakeState::Picky:

      color->set_led_pwm(100);

      motor->move(127);

      if(sort_override != false && detectRingColor() != teamColor){
        motor->tare_position();
        while(600 > abs(motor->get_position())){
          motor->move(127);
        }
        motor->move(0);
        delay(250);

        setState(IntakeState::Picky);
        
      }

      break;

    case IntakeState::Out:

      color->set_led_pwm(0);

      motor->move(-127);
      break;

    case IntakeState::Jam:

      color->set_led_pwm(0);

      motor->move(-127);
      if (armLoading) {
        setState(IntakeState::Idle);
        break;
      } else {
        motor->move(-100);
        pros::delay(250);
      }
      setState(IntakeState::Eat);
      break;
    }

    pros::delay(10);
  }
}

