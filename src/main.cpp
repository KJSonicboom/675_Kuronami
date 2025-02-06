#include "auton.hpp"
#include "lib/lift.hpp"
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

//======================EZ & Lemlib===========================

void initialize() {


  chassis.calibrate();

  while (imu.is_calibrating()) {
    pros::delay(10);
  }

  odomH.reset();
  odomV.reset();
  pros::delay(500);

  drivetrain.startTask();
  direct.startTask();
  intake.startTask();

  pros::lcd::initialize();

  pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            controller.set_text(0, 0, "X: " + std::to_string((int)chassis.getPose().x)); // x
            controller.set_text(0, 2, "X: " + std::to_string((int)chassis.getPose().x)); // x
            controller.set_text(0, 4, "X: " + std::to_string((int)chassis.getPose().x)); // x
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });



  // selector.focus();



}

void disabled() {
  
}

void competition_initialize() {}

void autonomous() {
  
  skills();


}

void opcontrol() {

  bool isUp = false;
  bool trapDirect = false;

  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  while (true) {

    drivetrain.arcadeMod(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), 
                          controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), {10 , 10}, 3); //needs tuning

    //intake control
    if(controller.get_digital(E_CONTROLLER_DIGITAL_R1)){
      if(!intake.sort_override) intake.setState(lib::IntakeState::Eat);
      else intake.setState(lib::IntakeState::Picky);
    }
    else if(controller.get_digital(E_CONTROLLER_DIGITAL_R2)) intake.setState(lib::IntakeState::Out);
    else if(controller.get_digital(E_CONTROLLER_DIGITAL_L2)) intake.setState(lib::IntakeState::Reload);
    else{
      intake.setState(lib::IntakeState::Idle);
    }

    //intake sort override
    if(controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) intake.sort_override = !intake.sort_override;

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

    pros::delay(15);
  }
}
