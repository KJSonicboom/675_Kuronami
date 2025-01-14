#pragma once
#include "StateMachine.hpp"
#include "lib/TaskWrapper.hpp"
#include "pid.h"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include <memory>

namespace lib {

enum class LiftState {Disabled, Scoring};

class Lift : public StateMachine<LiftState, LiftState::Scoring>, public ryan::TaskWrapper {

private:
  std::shared_ptr<pros::Motor> motor;
  const float DOWN_ANGLE = 5;
  const float UP_ANGLE = -710 ;

  const float gearRatio;

  double target;

  float p = 5;

  float d = 20;

  float prev_error = 0;


public:
  Lift(pros::Motor *motor, double gearRatio) : motor(motor), gearRatio(gearRatio) {
    motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  void loop() override;

  float getAngle();
  
};

} // namespace lib