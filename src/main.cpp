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
                       {"Blue Goal Side", blue_goal_side}
});

// ========================= Competition Functions ========================= //

// create the chassis
inline lib::Chassis chassis = lib::Chassis(&leftMotors, &rightMotors, &imu, &Vtrack, 480, 2.75);

//======================EZ & Lemlib===========================

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

  selector.focus();

  init();
}

void disabled() {
  
}

void competition_initialize() {}

void autonomous() {
  
  red_goal_side();

}

void opcontrol() {

  float directTarget = -1;

  bool isUp = false;
  bool trapDirect = false;

  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  while (true) {

    chassis.arcadeMod(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
                          controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), {10 , 10}, 3); //needs tuning

    //intake control
    if(controller.get_digital(E_CONTROLLER_DIGITAL_R1)){
      // direct.setState(lib::LiftState::Down);
      if(!intake.sort_override) intake.setState(lib::IntakeState::Eat);
      else intake.setState(lib::IntakeState::Picky);
    }
    else if(controller.get_digital(E_CONTROLLER_DIGITAL_R2)) intake.setState(lib::IntakeState::Out);
    else if(controller.get_digital(E_CONTROLLER_DIGITAL_L2)) intake.setState(lib::IntakeState::Reload);
    else if(!isUp){
      direct.setState(lib::LiftState::Disabled);
      intake.setState(lib::IntakeState::Idle);
    } 
    else{
      intake.setState(lib::IntakeState::Idle);
    }

    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) direct.setState(lib::LiftState::Disabled);;

    //clamp control
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) mogoClamp.toggle();

    //doinker control
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) doinker.toggle();

    //direct mech control
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
      if(!isUp){
        direct.setState(lib::LiftState::Scoring);
        isUp = !isUp;
      }

      else{
       direct.setState(lib::LiftState::Disabled);
       isUp = !isUp;
      }
    } 

    //push arm down
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
      if(trapDirect) {
        direct.moveVelocity(-50);
        trapDirect = !trapDirect;
      }
      else{
        direct.moveVelocity(0);
        trapDirect = !trapDirect;
      }
    }

    pros::delay(15);
  }
}
