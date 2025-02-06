#include "main.h"
#include "pros/adi.h"
#include "lemlib/api.hpp"
#include "robotconfig.h"
#include "EZ-Template/api.hpp"
#include <string>
#include <future>         

//Lemlib stuff

// Imu imu(20); 

// // horizontal tracking wheel
// lemlib::TrackingWheel lemlib_horizontal_tracking_wheel(&odomH, lemlib::Omniwheel::NEW_275_HALF, -1.5);
// lemlib::TrackingWheel lemlib_vertical_tracking_wheel(&odomV, lemlib::Omniwheel::NEW_275_HALF, 1.25);

// lemlib::Drivetrain lemlibDrivetrain(&leftMotors, // left motor group
//                               &rightMotors, // right motor group
//                               10, // 10 inch track width
//                               lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
//                               480, // drivetrain rpm is 480
//                               2 // horizontal drift is 2 (idk what this means)
// );

// // odometry settings
// lemlib::OdomSensors sensors(&lemlib_vertical_tracking_wheel, // vertical tracking wheel 1
//                             nullptr,
//                             &lemlib_horizontal_tracking_wheel, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2
//                             &imu // inertial sensor
// );

// // lateral PID controller
// lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               25, // derivative gain (kD)
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               2, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0  // maximum acceleration (slew): use if acceleration is too high
// );                                       

// // angular PID controller
// lemlib::ControllerSettings angular_controller(5, // proportional gain (kP) 
//                                               0, // integral gain (kI) 
//                                               48, // dervative gain (kD) 
//                                               0, // anti windup
//                                               1, // small error range, in inches
//                                               100, // small error range timeout, in milliseconds
//                                               3, // large error range, in inches
//                                               500, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// // create the chassis
// lemlib::Chassis chassis(lemlibDrivetrain, // drivetrain settings
//                         lateral_controller, // lateral PID settings
//                         angular_controller, // angular PID settings
//                         sensors // odometry sensors
// );



//https://www.youtube.com/watch?v=u-WkupiXnWQ <-- route
void skills()
{ 

  intake.teamColor = lib::ringColor::RED;
  // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

  // chassis.setPose(-62, 0, 270);

  //score on alliance and move away a lil bit
  // direct.setState(lib::LiftState::AllianceStake);
  // delay(250);
  // chassis.moveToPoint(-64, 0, 750);
  // chassis.waitUntilDone();
  // direct.setState(lib::LiftState::Disabled);
  // delay(250);
  // chassis.moveToPoint(-60, 0, 500, {.forwards = false});

  //get mogo
  // chassis.turnToPoint(-47, -23, 1500, {.forwards = false});

  // chassis.moveToPoint(-47, -23, 1500, {.forwards = false});
  // chassis.waitUntilDone();
  // delay(250);
  // mogoClamp.set_value(true);
  // delay(150);

  //turn and get first two rings
  // intake.setState(lib::IntakeState::Eat);
  // chassis.turnToPoint(-22, -24, 1000);
  // chassis.moveToPoint(-22, -24, 1500, {.minSpeed = 30, .earlyExitRange = 3});
  // intake.setState(lib::IntakeState::Hold);
  // chassis.moveToPoint(24, -47, 1500);

  // //go back and score on wall stake
  // chassis.moveToPoint(0, -47, 3000, {.forwards = false});
  // direct.setState(lib::LiftState::Scoring);
  // chassis.turnToHeading(180, 1200, {.maxSpeed = 60});
  // intake.setState(lib::IntakeState::Eat);
  // chassis.moveToPoint(0, -65, 1000);
  // delay(250);
  // chassis.moveToPoint(0, -47, 1500, {.forwards = false});

  //fill goal and place in corner
  // chassis.moveToPoint()
  

  //score on first mogo



}


void blue_goal_side(){


}

void blue_stack_side(){

    
}

void red_goal_side(){

    


}

void red_stack_side(){

    
}

// void blue_elims_goal_side(){

// isRedTeam = false;

// }

// void red_elims_goal_side(){
    
// isRedTeam = true;

// }