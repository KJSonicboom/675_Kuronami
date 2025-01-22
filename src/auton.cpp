#include "main.h"
#include "pros/adi.h"
#include "lemlib/api.hpp"
#include "robotconfig.h"
#include "EZ-Template/api.hpp"
#include <future>         

//for EZ
//chassis.pid_drive_set(24_in, 110); or chassis.pid_turn_set(45_deg, 90); or chassis.pid_turn_relative_set(45_deg, 90);
//chassis.odom_xyt_set(0_in, 0_in, 0_deg);
//chassis.pid_wait();

ez::Drive ezChassis(
    // These are your drive motors, the first motor is used for sensing!
    {-9, -16, -21},     // Left Chassis Ports (negative port will reverse it!)
    {3, 14, 18},  // Right Chassis Ports (negative port will reverse it!)

    12,       // IMU Port
    3.25,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    480);  // Wheel RPM = cartridge * (motor gear / wheel gear)

ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels



void init(){

  ezChassis.pid_drive_constants_set(7.0, 0.0, 25.0);         // Fwd/rev constants, used for odom and non odom motions
  ezChassis.pid_heading_constants_set(5.0, 0.0, 48.0);        // Holds the robot straight while going forward without odom
  ezChassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  ezChassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  ezChassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  ezChassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  ezChassis.odom_tracker_back_set(&horiz_tracker);
  leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

}

void skills()
{ 

}


void blue_goal_side(){

  // lib::Intake::toggleSide();

}

void blue_stack_side(){

    
}

void red_goal_side(){

  // lib::Intake::toggleSide();

    // ezChassis.pid_drive_set(-48_in, 110);
    // ezChassis.pid_wait();

    // mogoClamp.toggle();




}

void red_stack_side(){

    
}

// void blue_elims_goal_side(){

// isRedTeam = false;

// }

// void red_elims_goal_side(){
    
// isRedTeam = true;

// }