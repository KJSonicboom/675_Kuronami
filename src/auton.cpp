#include "main.h"
#include "pros/adi.h"
#include "lemlib/api.hpp"
#include "robotconfig.h"
#include "EZ-Template/api.hpp"
#include <string>
#include <future>         

//https://www.youtube.com/watch?v=u-WkupiXnWQ <-- route
void skills()
{ 

  //this sets the team color for color sorting
  intake.teamColor = lib::ringColor::RED;
  //this sets the drivetrain motors to hold, vital for auton
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

  //set original pose (copy paste from path jerry for precision)
  chassis.setPose(-63, 0, 90);

  //score on alliance and move away a lil bit
  intake.setState(lib::IntakeState::Eat); //"setState" is used for all the state machines
  delay(400); //you need to delay a lot
  intake.setState(lib::IntakeState::Idle);
  chassis.moveToPoint(-60, 0, 500, {.forwards = false}); //moveToPoint moves to the (x, y) with a timeout

  // get mogo
  chassis.turnToPoint(-47, -23, 1500, {.forwards = false}); //always use turnToPoint and moveToPoint in unison
  chassis.moveToPoint(-47, -23, 1000, {.forwards = false});
  chassis.waitUntilDone(); //this is pretty self explanatory
  delay(400);
  mogoClamp.set_value(true);
  delay(150);

  //turn and get first two rings
  intake.setState(lib::IntakeState::Eat);
  chassis.turnToPoint(-20, -24, 1000);
  chassis.moveToPoint(-20, -24, 1500, {.minSpeed = 30, .earlyExitRange = 3});
  intake.setState(lib::IntakeState::Hold);
  chassis.moveToPoint(22, -49, 1500);

  // //go back and score on wall stake
  chassis.moveToPoint(0, -49, 3000, {.forwards = false});
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