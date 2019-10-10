/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Kyle Pirce                                                */
/*    Created:      Tue Sep 17 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//#include "C:/Program Files (x86)/VEX
//Robotics/VEXcode/sdk/vexv5/include/v5_apiuser.h"
//#include "C:/Program Files (x86)/VEX Robotics/VEXcode/sdk/vexv5/include/vex_global.h"
//#include "C:/Program Files (x86)/VEX Robotics/VEXcode/sdk/vexv5/include/vex_controller.h"
//#include "C:/Program Files (x86)/VEX Robotics/VEXcode/sdk/vexv5/include/vex_units.h"
#include "vex.h"

using namespace vex;

competition Competition;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller Controller = controller();
controller Controller2 = controller();
motor FL = motor(PORT20, false);
motor FR = motor(PORT11, true);
motor BL = motor(PORT10, false);
motor BR = motor(PORT1, true);
// lift control
motor Llift = motor(PORT8, false);
motor Rlift = motor(PORT17, true);
motor Claw = motor(PORT12, false);
motor fourbar = motor(PORT5, true);

gyro myGyro = gyro(Brain.ThreeWirePort.A);

// define your global instances of motors and other devices here

// Custom Functions

void drive(int LeftF, int RightF, int LeftB, int RightB, int wt) {
  FL.spin(directionType::fwd, LeftF, velocityUnits::pct);
  FR.spin(directionType::fwd, RightF, velocityUnits::pct);
  BL.spin(directionType::fwd, LeftB, velocityUnits::pct);
  BR.spin(directionType::fwd, RightB, velocityUnits::pct);
  task::sleep(wt);
}
/////////////////////////////////////////////////////////////
//////////////////////CLAW OPEN//////////////////////////////
/////////////////////////////////////////////////////////////
void clawOpen(int clawSpeed, int wait) {
Claw.spin(directionType::fwd, clawSpeed, velocityUnits::pct);  
task::sleep(wait);
Claw.stop(hold);
}
/////////////////////////////////////////////////////////////
//////////////////////CLAW CLOSE/////////////////////////////
/////////////////////////////////////////////////////////////
void clawClose(int clawSpeed, int wait) {
Claw.spin(directionType::rev, clawSpeed, velocityUnits::pct);  
task::sleep(wait);
Claw.stop(hold);
}
/////////////////////////////////////////////////////////////
///////////////////////LIFT UP///////////////////////////////
/////////////////////////////////////////////////////////////
void liftUp(int liftSpeed, int wait){
Llift.spin(directionType::fwd, liftSpeed, velocityUnits::pct);
Rlift.spin(directionType::fwd, liftSpeed, velocityUnits::pct);
task::sleep(wait);
Llift.stop(hold);
Rlift.stop(hold);
}
/////////////////////////////////////////////////////////////
///////////////////////LIFT DOWN/////////////////////////////
/////////////////////////////////////////////////////////////
void liftDown(int liftSpeed, int wait){
Llift.spin(directionType::rev, liftSpeed, velocityUnits::pct);
Rlift.spin(directionType::rev, liftSpeed, velocityUnits::pct);
task::sleep(wait);
Llift.stop(hold);
Rlift.stop(hold);
}
/////////////////////////////////////////////////////////////
/////////////LIFT DOWN AND CLOSE CLAW////////////////////////
/////////////////////////////////////////////////////////////
void liftDownWithCloseClaw(int liftSpeed, int clawSpeed, int wait){
Llift.spin(directionType::rev, liftSpeed, velocityUnits::pct);
Rlift.spin(directionType::rev, liftSpeed, velocityUnits::pct);
Claw.spin(directionType::rev, clawSpeed, velocityUnits::pct);  
task::sleep(wait);
Rlift.stop(hold);
Llift.stop(hold);
Claw.stop(hold);
}
void holoDrive(int ch3, int ch4, int ch1, int wt) {
  FL.spin(directionType::fwd, ch3 + ch4 + ch1, velocityUnits::pct);
  FR.spin(directionType::fwd, ch3 - ch4 - ch1, velocityUnits::pct);
  BL.spin(directionType::fwd, ch3 - ch4 + ch1, velocityUnits::pct);
  BR.spin(directionType::fwd, ch3 + ch4 - ch1, velocityUnits::pct);
  task::sleep(wt);
}

//------//

void preAuton() {
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "Pre-Auton");
  myGyro.startCalibration();
}

void autonomous() {
/////////////////////////////////////////////////////////////
//auto plan (starting up top):///////////////////////////////
///////1) close intake to grab preload///////////////////////
///////2) drive forwards to pickup cube in front of bot//////
///////3) lower lift to pick up cube/////////////////////////
///////4) raise lift a little////////////////////////////////
//5) drive forwards while lifting to get 4 stack in front////
///////6) lift down to get 4 stack///////////////////////////
///////7) raise lift to get cubes above scoring zone/////////
///////8) strafe into the wall while turning/////////////////
///////9) drive forwards to scoring zone/////////////////////
//////10) lower lift/////////////////////////////////////////
//////11) open claw while driving backwards//////////////////
/////////////////////////////////////////////////////////////

  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "1 Point Auton");
  drive(100, 100, 100, 100, 1000);
  drive(-75, -75, -75, -75, 300);
  drive(0, 0, 0, 0, 0);
}

void userControl() {
  int ch3;
  int ch4;
  int ch1;
  int ch2;
  Brain.Screen.clearScreen();
  Brain.Screen.printAt(1, 40, "6007R Driver Control");
  //Brain.Screen.printAt(1, 80, "fourbar.velocity(velocityUnits rpm)");
  Brain.Screen.drawRectangle(1, 40, 50,20);
  while (true) {
    fourbar.spin(directionType::rev);
    ch3 = Controller.Axis3.position(percentUnits::pct);
    ch4 = Controller.Axis4.position(percentUnits::pct);
    ch1 = Controller.Axis1.position(percentUnits::pct);
    ch2 = Controller.Axis2.position(percentUnits::pct);

    //holoDrive(ch3, ch4, ch1, 10);
    FL.spin(directionType::fwd, ch3 + ch4, velocityUnits::pct);
    BL.spin(directionType::fwd, ch3 - ch4, velocityUnits::pct);
    FR.spin(directionType::fwd, ch2 - ch1, velocityUnits::pct);
    BR.spin(directionType::fwd, ch2 + ch1, velocityUnits::pct);
    if (Controller.ButtonL1.pressing()) {
      Llift.spin(directionType::rev, 100, velocityUnits::pct);
      Rlift.spin(directionType::rev, 100, velocityUnits::pct);
      //fourbar.setVelocity(50, velocityUnits::pct);
    } else if (Controller.ButtonL2.pressing()) {
      Llift.spin(directionType::fwd, 100, velocityUnits::pct);
      Rlift.spin(directionType::fwd, 100, velocityUnits::pct);
      //fourbar.setVelocity(0, velocityUnits::pct);
    } else {
      Llift.stop(brakeType::brake);
      Rlift.stop(brakeType::brake);
    }

    if (Controller.ButtonR2.pressing()) {
      Claw.spin(directionType::rev, 100, velocityUnits::pct);
      //fourbar.setVelocity(-100, velocityUnits::pct);
    } else if (Controller.ButtonR1.pressing()) {
      Claw.spin(directionType::fwd, 100, velocityUnits::pct);
    } else {
      Claw.stop(brakeType::brake);
    }
    /*
    if (Controller.ButtonY.pressing()) {
      //fourbar.setVelocity(100, velocityUnits::pct);
    } else if (Controller2.ButtonX.pressing()) {
      //fourbar.setVelocity(0, velocityUnits::pct);
    }
    if (Controller.ButtonB.pressing()) {
      //fourbar.setVelocity(-100, velocityUnits::pct);
    }
    */
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  preAuton();
  while (true) {
    task::sleep(500);
  }
}
