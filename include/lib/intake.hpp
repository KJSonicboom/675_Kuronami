#pragma once
#include "lib/TaskWrapper.hpp"
#include "lib/StateMachine.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/optical.hpp"

namespace lib {

enum class IntakeState {Eat, Picky, Reload, Out, Jam, Idle};

enum class ringColor {RED, BLUE, NOTHING};

class Intake : public StateMachine<IntakeState, IntakeState::Idle>, public ryan::TaskWrapper {

private:

  std::shared_ptr<pros::Motor> motor;
  std::shared_ptr<pros::Optical> color;
  int sort_time;


public:

  bool sort_override = false;
  ringColor teamColor = lib::ringColor::RED;

  Intake(pros::Motor *motor, pros::Optical *color) : motor(motor), color(color) {
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }

  ///@brief returns ring color 
  ringColor detectRingColor(){

      color->set_led_pwm(100); //turns the LED light on to show sorting is enabled

      if(color->get_hue() < 22 && color->get_proximity() > 210) return ringColor::RED; 
      else if(color->get_hue() > 100 && color->get_proximity() > 210) return ringColor::BLUE;
      else return ringColor::NOTHING;

  }

  void loop() override;

  };
}