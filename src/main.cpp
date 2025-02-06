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

// rd::Selector selector({{"Skills", skills},
//                        {"Red Goal Side", red_goal_side},
//                        {"Blue Goal Side", blue_goal_side}
// });

// horizontal tracking wheel
lemlib::TrackingWheel lemlib_horizontal_tracking_wheel(&odomH, lemlib::Omniwheel::NEW_275_HALF, -1.5);
lemlib::TrackingWheel lemlib_vertical_tracking_wheel(&odomV, lemlib::Omniwheel::NEW_275_HALF, 1.25);

lemlib::Drivetrain lemlibDrivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              480, // drivetrain rpm is 480
                              2 // horizontal drift is 2 (idk what this means)
);

// odometry settings
lemlib::OdomSensors sensors(&lemlib_vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr,
                            &lemlib_horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              25, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0  // maximum acceleration (slew): use if acceleration is too high
);                                       

// angular PID controller
lemlib::ControllerSettings angular_controller(5, // proportional gain (kP) 
                                              0, // integral gain (kI) 
                                              48, // dervative gain (kD) 
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(lemlibDrivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

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
  
  // skills();

  chassis.setPose(-62, 0, 270);

  chassis.turnToHeading(0, 1000);


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
