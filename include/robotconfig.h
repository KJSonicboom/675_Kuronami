#pragma once

#include "lib/lift.hpp"
#include "lib/intake.hpp"
#include "lib/chassis.h"

// #include "EZ-Template/api.hpp"
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

//=======================INTAKE=============================

inline Motor intaker(-4, pros::v5::MotorGears::blue);
inline Optical intakeSensor(2);
inline lib::Intake intake(&intaker, &intakeSensor);

//========================ARM=================================

inline Motor arm(19, pros::v5::MotorGears::red); 
inline lib::Lift direct(&arm, 0.5);

//========================MISC===============================

inline Controller controller (E_CONTROLLER_MASTER);
inline Imu imu (12); 

inline adi::Pneumatics mogoClamp('G', false);
inline adi::Pneumatics doinker('H', false);

//======================ODOM STUFF===========================

//odom wheels
inline Rotation odomH(8); 
inline Rotation odomV(13);

//lib odom wheels
inline lib::TrackingWheel Vtrack(odomV, 2.44);
inline lib::TrackingWheel Htrack(odomH, 2.44);

// create the chassis
inline lib::Chassis chassis = lib::Chassis(&leftMotors, &rightMotors, &imu, &Vtrack, 480, 2.75);

//======================EZ & Lemlib===========================

// ez::Drive ezChassis(
//     // These are your drive motors, the first motor is used for sensing!
//     {-9, -16, -21},     // Left Chassis Ports (negative port will reverse it!)
//     {3, 14, 18},  // Right Chassis Ports (negative port will reverse it!)

//     12,       // IMU Port
//     3.25,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
//     480);  // Wheel RPM = cartridge * (motor gear / wheel gear)

// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
