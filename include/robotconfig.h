#pragma once

#include "lib/lift.hpp"
#include "lib/intake.hpp"
#include "lib/chassis.h"

#include "EZ-Template/api.hpp"
#include "lemlib/api.hpp"

#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

using namespace pros;

//====================GLOBAL VARIABLES=======================

//manages arm state
inline bool armLoading = false;

//=======================CHASSIS==============================

inline Motor left_front_motor(9, pros::v5::MotorGears::blue);
inline Motor left_middle_motor(16, pros::v5::MotorGears::blue);
inline Motor left_back_motor(21, pros::v5::MotorGears::blue);

inline MotorGroup leftMotors({-9, -16, -21}, pros::MotorGearset::blue);

inline Motor right_front_motor(3, pros::v5::MotorGears::blue);
inline Motor right_middle_motor(14, pros::v5::MotorGears::blue); 
inline Motor right_back_motor(18, pros::v5::MotorGears::blue);

inline MotorGroup rightMotors({3, 14, 18}, pros::MotorGearset::blue);

inline Imu imu (20); 

inline lib::Chassis drivetrain = lib::Chassis(&leftMotors, &rightMotors, 480, 2.75);

//=======================INTAKE=============================

inline Motor intaker(-4, pros::v5::MotorGears::blue);
inline Optical intakeSensor(2);
inline lib::Intake intake(&intaker, &intakeSensor);

//========================ARM=================================

inline Motor arm(19, pros::v5::MotorGears::red); 
inline lib::Lift direct(&arm, 0.5);

//========================MISC===============================

inline Controller controller (E_CONTROLLER_MASTER);

inline adi::Pneumatics mogoClamp('F', false);
inline adi::Pneumatics doinker('H', false);
// inline adi::Pneumatics PTO('F', true);

//======================ODOM STUFF===========================

//odom wheels
inline Rotation odomH(8); 
inline Rotation odomV(13);

// //Lemlib stuff

// // horizontal tracking wheel
// lemlib::TrackingWheel lemlib_horizontal_tracking_wheel(&odomH, lemlib::Omniwheel::NEW_275, 1.5);
// lemlib::TrackingWheel lemlib_vertical_tracking_wheel(&odomH, lemlib::Omniwheel::NEW_275, 1.25);

// inline lemlib::Drivetrain lemlibDrivetrain(&leftMotors, // left motor group
//                               &rightMotors, // right motor group
//                               10, // 10 inch track width
//                               lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
//                               480, // drivetrain rpm is 480
//                               2 // horizontal drift is 2 (idk what this means)
// );

// // odometry settings
// inline lemlib::OdomSensors sensors(&lemlib_vertical_tracking_wheel, // vertical tracking wheel 1
//                             nullptr,
//                             &lemlib_horizontal_tracking_wheel, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2
//                             &imu // inertial sensor
// );

// // lateral PID controller
// inline lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
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
// inline lemlib::ControllerSettings angular_controller(5, // proportional gain (kP) 
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
// inline lemlib::Chassis chassis(lemlibDrivetrain, // drivetrain settings
//                         lateral_controller, // lateral PID settings
//                         angular_controller, // angular PID settings
//                         sensors // odometry sensors
// );

