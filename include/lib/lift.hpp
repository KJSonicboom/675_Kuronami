#pragma once
#include "StateMachine.hpp"
#include "lib/TaskWrapper.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/timer.hpp"
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

  int currentPosition;

  int error = 100;

public:
  Lift(pros::Motor *motor, double gearRatio) : motor(motor), gearRatio(gearRatio) {
    motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
  void loop() override;

  /// @brief Moves the arm using a PID controller
  /// @param targetDegree Target degree of the arm
  /// @param timeout Timeout so the PID doesn't run forever
  void moveArm(int targetDegree, int timeout);


  float getAngle();
  
};

} // namespace lib