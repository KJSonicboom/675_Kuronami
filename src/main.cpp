#include "auton.hpp"
#include "lib/lift.hpp"
#include "lib/trajectory.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "robodash/api.h"
#include "robotconfig.h"
#include <string>

// ================================= Views ================================= //

rd::Selector selector({{"Skills", skills},
                       {"Red Goal Side", red_goal_side},
                       {"Red Stack Side", red_stack_side},
                       {"Blue Goal Side", blue_goal_side},
                       {"Blue Stack Side", blue_stack_side}});

// ========================= Competition Functions ========================= //

void initialize() {


  imu.reset();

  while (imu.is_calibrating()) {
    pros::delay(10);
  }

  odomH.reset();
  odomV.reset();
  pros::delay(500);

  chassis.startTask();
  direct.startTask();
  intake.startTask();
}

void disabled() {
  
}

void competition_initialize() {}

void autonomous() {
  // selector.focus();
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


  // selector.run_auton();
}

void opcontrol() {

  float directTarget = -1;

  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  while (true) {

    chassis.arcadeMod(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
                          controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), {10 , 10}, 3); //needs tuning

        //intake control
    if(controller.get_digital(E_CONTROLLER_DIGITAL_R1)){
        if(!intake.sort_override) intake.setState(lib::IntakeState::Eat);
        else intake.setState(lib::IntakeState::Picky);
    }
    else if(controller.get_digital(E_CONTROLLER_DIGITAL_R2)) intake.setState(lib::IntakeState::Out);
    else if(controller.get_digital(E_CONTROLLER_DIGITAL_L2)) intake.setState(lib::IntakeState::Reload);
    else intake.setState(lib::IntakeState::Idle);

    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) intake.sort_override = !intake.sort_override;

    //clamp control
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) mogoClamp.toggle();

        //doinker control
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) doinker.toggle();

    //direct mech control
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) direct.setState(lib::LiftState::Scoring);
    else direct.setState(lib::LiftState::Disabled);


    pros::delay(15);
  }
}
