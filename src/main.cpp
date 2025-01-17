/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*--------------
--------------------------------------------------------------*/


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                  
// Drivetrain           drivetrain    1, 10, D      `
// ClawMotor            motor         3             
// ArmMotor             motor         8             
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"
#include "functions.hpp"
#include <string>

using namespace std::chrono;
using namespace vex;

// A global instance of competition
competition Competition;

bool clamptrue = false;
bool prevclamp = false;

bool doinkertrue = false;
bool prevdoinker = false;

steady_clock::time_point lastClamp;
steady_clock::time_point lastToggle;
steady_clock::time_point lastDoinker;
// define your global instances of motors and other devices here


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


void pre_auton(void) {
//  inertialSensor.calibrate();
//  while(inertialSensor.isCalibrating()) {
//   wait(20, msec);
//  }
// Initializing Robot Configuration. DO NOT REMOVE!
vexcodeInit();
inertialSensor.calibrate();
wait(5, msec);
waitUntil(!inertialSensor.isCalibrating());
}
 // All activities that occur before the competition starts
// Example: clearing encoders, setting servo positions, ...


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*----------------------------- ----------------------------------------------*/

// PID
double kp = 0.175;
double ki = 0;
double kd = 0;

void pid(double targetDistance) {
 double error = targetDistance;
 double integral = 0;
 double lastError =  targetDistance;
 double prevDistanceError = fl.position(degrees);
 fl.setPosition(0, degrees);
 ml.setPosition(0, degrees);
 bl.setPosition(0, degrees);
 fr.setPosition(0, degrees);
 mr.setPosition(0, degrees);
 br.setPosition(0, degrees);
 while (true) {
   double measureDistance = (fl.position(degrees) + fr.position(degrees))/2;
   error = targetDistance - measureDistance;
   prevDistanceError = measureDistance;
   if (fabs(error)<30) {
     fl.stop(brake);
     ml.stop(brake);
     bl.stop(brake);

     fr.stop(brake);
     mr.stop(brake);
     br.stop(brake);
     return;
   }
  
   double speed = error * kp + integral * ki + (error - lastError) * kd;
   fl.spin(fwd, speed, percent);
   ml.spin(fwd, speed, percent);
   bl.spin(fwd, speed, percent);

   fr.spin(fwd, speed, percent);
   mr.spin(fwd, speed, percent);
   br.spin(fwd, speed, percent);

   lastError = error;
   wait(15, msec);
 }
}

// PID to inches
#define INCHES_TO_DEGREES 90/5
void pid_inches (double DistanceInInches) {
 double degrees = DistanceInInches * INCHES_TO_DEGREES;
 pid(degrees);
}
//stop moving (good)
void stopWheels() {
 // stop all motors in brake
 fl.stop(brake);
 ml.stop(brake);
 bl.stop(brake);

 fr.stop(brake);
 mr.stop(brake);
 br.stop(brake);
}

//clamp
void clamp() {
  mogo.set(true);
  mogo2.set(true);
}

//unclamp
void unclamp() {
  mogo.set(false);
  mogo2.set(false);
}

//use mogo mech
void mogoControl() {
 if (clamptrue) {
  mogo.set(true);
  mogo2.set(true);
 } else {
  mogo.set(false);
  mogo2.set(false);
 }
}

// clamp using one button
void clamping() {
 auto now = steady_clock::now();
 auto durLastClamp = duration_cast<milliseconds>(now-lastClamp).count();
 if (durLastClamp > 200) {
   mogoControl();
   clamptrue = !clamptrue;
   lastClamp = now;
 }
}

// Doinker up
void doinkerUp() {
  doinker.set(false);
}

//doinker down
void doinkerDown() {
  doinker.set(true);
}

//use the doinker
void doinkerControl() {
 if (doinkertrue) {
  doinkerDown();
 } else {
  doinkerUp();
 }
}

//use the doinker using one button
void doinkeroinker() {
 auto now = steady_clock::now();
 auto durLastDoinker = duration_cast<milliseconds>(now-lastDoinker).count();
 if (durLastDoinker > 100) {
   doinkerControl();
   doinkertrue = !doinkertrue;
   lastDoinker = now;
 }
}

//littereally move all wheels
void moveAllWheels(int SpeedLeft, int SpeedRight, int ) {
fl.spin(forward, SpeedLeft + SpeedRight, percent);
ml.spin(forward, SpeedLeft + SpeedRight, percent);
bl.spin(forward, SpeedLeft + SpeedRight, percent);

fr.spin(reverse, SpeedLeft - SpeedRight, percent);
mr.spin(reverse, SpeedLeft - SpeedRight, percent);
br.spin(reverse, SpeedLeft - SpeedRight, percent);
}

// turn left
void turnLeft(double angle) {
// basically the same as right except left motor spins reverse and right is forward
inertialSensor.setRotation(0, degrees);

//turning left using inertial sensor
while (fabs(inertialSensor.rotation(deg)) < angle) {
  double diff =  angle - fabs(inertialSensor.rotation(deg));
  // 5 + diff * 0.3 ,pct means to slow down when reaching the precent target.
  //You have to remember to set the minimum speed to 5 so it does not slowly move
  fr.spin(forward, 5 + diff * 0.3, pct);
  mr.spin(forward, 5 + diff * 0.3, pct);
  br.spin(forward, 5 + diff * 0.3, pct);
 
  fl.spin(reverse, 5 + diff * 0.3, pct);
  ml.spin(reverse, 5 + diff * 0.3, pct);
  bl.spin(reverse, 5 + diff * 0.3, pct);
  wait(5, msec);
}
stopWheels();
}

// intaking in driver control
void intaking() {
if (controller1.ButtonR2.pressing()) {
  intake.spin(forward, 400, rpm);
 //  intake2.spin(forward, 395, rpm);
} else if (controller1.ButtonR1.pressing()) {
  intake.spin(reverse, 80, pct);
 //  intake2.spin(reverse, 90, percent);
} else {
  intake.stop(coast);
 //  intake2.stop(coast);
}
}

//turn right
void turnRight(double angle) {
// set inertial rotation to 0 degrees
inertialSensor.setRotation(0, degrees);
//turn right using inertial sensors
while (inertialSensor.rotation(deg) < angle) {
  double diff =  angle - fabs(inertialSensor.rotation(deg));
  fl.spin(forward, 5 + diff * 0.3, pct);
  ml.spin(forward, 5 + diff * 0.3, pct);
  bl.spin(forward, 5 + diff * 0.3, pct);
  fr.spin(reverse, 5 + diff * 0.3, pct);
  mr.spin(reverse, 5 + diff * 0.3, pct);
  br.spin(reverse, 5 + diff * 0.3, pct);
  wait(5, msec);
}
stopWheels();
}

void stopwheels() {
 fl.stop(coast);
 ml.stop(coast);
 bl.stop(coast);

 fr.stop(coast);
 mr.stop(coast);
 br.stop(coast);
}

//set velocity
void setVelocity(double vel) {
 // set all motors to velocity value of 'vel'
 fl.setVelocity(vel, percent);
 ml.setVelocity(vel, percent);
 bl.setVelocity(vel, percent);
 fr.setVelocity(vel, percent);
 mr.setVelocity(vel, percent);
 br.setVelocity(vel, percent);
}

//intake in auton
void intakeInAuton() {
 intake.spin(reverse, 500, rpm);
 //intake2.spin(reverse, 300, rpm);
}

// outake in auton
void outtakeInAuton() {
 intake.spin(forward, 500, rpm);
 //intake2.spin(forward, 330, rpm);
}

// stop intaking
void stopIntaking() {
 intake.stop(coast);
 //intake2.stop(coast);
}

//this is our old code
//blue right auton (4 ring)
// void blueRight() {
//   kp = 0.11;
//   pid_inches(-31.5);
//   clamp();
//   kp = 0.2;
//   intakeInAuton();
//   wait(1, sec);
//   stopIntaking();
//   turnLeft(115);
//   kp = 0.12;
//   //first ring
//   intakeInAuton();
//   pid_inches(12.5);
//   // intakeInAuton();
//   // pid_inches(6);
//   wait(1, sec);
//   stopIntaking();
//   pid_inches(-5);
//   turnRight(62);
//   stopIntaking();
//   //second ring
//   intakeInAuton();
//   pid_inches(14);
//   wait(0.8, sec);
//   stopIntaking();
//   kp = 0.3;
//   turnLeft(20);
//   pid_inches(10);
//   turnLeft(100);
//   kp = 0.3;
//   intakeInAuton();
//   pid_inches(8);
//   wait(0.75, sec);
//   pid_inches(-15);
//   turnLeft(40);
//   pid_inches(38);
//   WallStakes2.spin(reverse, 90, pct);
//   WallStakes.spin(forward, 90, pct);
//   wait(0.4, sec);
// }

void blueRight() {
 WallStakes.setVelocity(45, pct);
 WallStakes2.setVelocity(45, pct);
 WallStakes.spin(forward, 90, pct);
 WallStakes2.spin(forward, 90, pct);
 wait(0.5, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 pid_inches(-7);
 WallStakes.spin(reverse, 90, pct);
 WallStakes2.spin(reverse, 90, pct);
 wait(0.4, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 turnLeft(11);
 pid_inches(-25);
 mogo.set(true);
 mogo2.set(true);
 wait(200, msec);
 turnLeft(146);
 intakeInAuton();
 pid_inches(22);
 wait(0.3, sec);
 pid_inches(-8);
 turnRight(10);
 pid_inches(14);
 wait(0.7, sec);
 stopIntaking();
 pid_inches(-5);
 intakeInAuton();
 pid_inches(-10);
 turnRight(45);
 pid_inches(17);
 wait(0.7, sec);
 stopIntaking();
 pid_inches(-36);
}

void blueRightElims() {
 WallStakes.setVelocity(45, pct);
 WallStakes2.setVelocity(45, pct);
 WallStakes.spin(forward, 90, pct);
 WallStakes2.spin(forward, 90, pct);
 wait(0.5, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 pid_inches(-7);
 WallStakes.spin(reverse, 90, pct);
 WallStakes2.spin(reverse, 90, pct);
 wait(0.4, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 turnLeft(11);
 pid_inches(-25);
 mogo.set(true);
 mogo2.set(true);
 wait(200, msec);
 turnLeft(146);
 intakeInAuton();
 pid_inches(22);
 wait(0.3, sec);
 pid_inches(-8);
 turnRight(10);
 pid_inches(14);
 wait(0.7, sec);
 stopIntaking();
 pid_inches(-5);
 intakeInAuton();
 pid_inches(-10);
 turnRight(45);
 pid_inches(17);
 wait(1, sec);
 stopIntaking();
 outtakeInAuton();
 wait(0.5, sec);
 turnRight(50);
 pid_inches(20);
 intakeInAuton();
 pid_inches(15);
}

//blue goal rush auton
void blueGoalRush() {
 kp = 0.3;
 pid_inches(-33);
 turnRight(18.5);
 kp = 0.2;
 pid_inches(-9.53);
 clamp();
 wait(0.5, sec);
 //first ring
 intake.spin(reverse, 80, pct);
 turnLeft(35);
 pid_inches(12);
 stopIntaking();
 //second ring
 intake.spin(reverse, 30, pct);
 wait(0.2222222, sec);
 stopIntaking();
 pid_inches(25);
 turnLeft(160);
 pid_inches(-9);
 unclamp();
 kp = 0.3;
 pid_inches(10);
 kp = 0.15;
 turnRight(219.5);
 kp = 0.1;
 pid_inches(-25);
 clamp();
 wait(0.5, sec);
 pid_inches(2);
 //2nd ring in goal
 intakeInAuton();
 wait(0.7, sec);
 //intake2.spin(reverse, 450, rpm);
 pid_inches(-14);
}

void blueGoalRushElims() {
 kp = 0.3;
 pid_inches(-33);
 turnRight(18.5);
 kp = 0.2;
 pid_inches(-9.53);
 clamp();
 wait(0.5, sec);
 //first ring
 intake.spin(reverse, 80, pct);
 turnLeft(35);
 pid_inches(12);
 stopIntaking();
 //second ring
 intake.spin(reverse, 30, pct);
 wait(0.2222222, sec);
 stopIntaking();
 pid_inches(25);
 turnLeft(160);
 pid_inches(-9);
 unclamp();
 kp = 0.3;
 pid_inches(10);
 kp = 0.15;
 turnRight(219.5);
 pid_inches(-25);
 clamp();
 wait(0.5, sec);
 pid_inches(2);
 //2nd ring in goal
 intakeInAuton();
 wait(0.7, sec);
 stopIntaking();
 //intake2.spin(reverse, 450, rpm);
 turnLeft(15);
 pid_inches(50);
}

void redGoalRush() {
 kp = 0.3;
 pid_inches(-30);
 turnLeft(27);
 // going backwards to get the goal rush goal
 pid_inches(-12);
 kp = 0.24;
 clamp();
 //scores preload
 intake.spin(reverse, 80, pct);
 //intake2.spin(reverse, 450, rpm);
 turnRight(43);
 pid_inches(10);
 stopIntaking();
 intake.spin(reverse, 80, pct);
 wait(0.1, sec);
 stopIntaking();
 pid_inches(27);
 turnRight(145);
 pid_inches(-4);
 unclamp();
 kp = 0.1;
 pid_inches(5);
 turnRight(140);
 pid_inches(-20);
 clamp();
 wait(0.4, sec);
 intakeInAuton();
 kp = 0.4;
 wait(1, sec);
 stopIntaking();
 pid_inches(-16);
 // turnRight(150.5);
 // pid_inches(8);
}

void redGoalRushElims() {
 kp = 0.3;
 pid_inches(-30);
 turnLeft(27);
 // going backwards to get the goal rush goal
 pid_inches(-12);
 kp = 0.25;
 clamp();
 //scores preload
 intake.spin(reverse, 80, pct);
 //intake2.spin(reverse, 450, rpm);
 turnRight(43);
 pid_inches(10);
 stopIntaking();
 intake.spin(reverse, 80, pct);
 wait(0.1, sec);
 stopIntaking();
 pid_inches(27);
 turnRight(145);
 pid_inches(-4);
 unclamp();
 kp = 0.1;
 pid_inches(5);
 turnRight(140);
 pid_inches(-20);
 clamp();
 wait(0.4, sec);
 intakeInAuton();
 kp = 0.4;
 wait(1, sec);
 stopIntaking();
 turnRight(15);
 pid_inches(50);
 // turnRight(150.5);
 // pid_inches(8);
}

//our old code
// red left auton
// void redLeft() {
//   kp = 0.14;
//   pid_inches(-30);
//   clamp();
//   kp = 0.17;
//   //first ring
//   intakeInAuton();
//   // wait(0.4, sec);
//   turnRight(117);
//   wait(0.75, sec);
//   stopIntaking();
//   pid_inches(14.5);
//   //second ring
//   intakeInAuton();
//   pid_inches(3);
//   wait(0.7, sec);
//   pid_inches(-5);
//   turnLeft(70);
//   pid_inches(10);
//   //third ring
//   intakeInAuton();
//   pid_inches(4);
//   wait(1, sec);
//   stopIntaking();
//   pid_inches(14);
//   turnRight(115);
//   //fourth ring
//   intakeInAuton();
//   pid_inches(16.187);
//   wait(1, sec);
//   pid_inches(-13);
//   turnRight(60);
//   pid_inches(25);
// }

void redLeft() {
 WallStakes.setVelocity(45, pct);
 WallStakes2.setVelocity(45, pct);
 WallStakes.spin(forward, 90, pct);
 WallStakes2.spin(forward, 90, pct);
 wait(0.5, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 pid_inches(-7);
 WallStakes.spin(reverse, 90, pct);
 WallStakes2.spin(reverse, 90, pct);
 wait(0.5, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 turnRight(11);
 pid_inches(-26);
 kp = 0.17;
 mogo.set(true);
 mogo2.set(true);
 kp = 0.175;
 wait(200, msec);
 turnRight(140);
 intakeInAuton();
 pid_inches(23);
 wait(0.5, sec);
 pid_inches(-8);
 turnLeft(12);
 pid_inches(16.5);
 wait(0.8, sec);
 pid_inches(-15);
 turnLeft(45);
 pid_inches(17);
 wait(0.7, sec);
 stopIntaking();
 pid_inches(-32);
}

void redLeftGoofy() {
 WallStakes.setVelocity(45, pct);
 WallStakes2.setVelocity(45, pct);
 WallStakes.spin(forward, 90, pct);
 WallStakes2.spin(forward, 90, pct);
 wait(0.5, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 pid_inches(-7);
 WallStakes.spin(reverse, 90, pct);
 WallStakes2.spin(reverse, 90, pct);
 wait(0.5, sec);
 WallStakes.stop(coast);
 WallStakes2.stop(coast);
 turnRight(9);
 pid_inches(-26);
 kp = 0.17;
 mogo.set(true);
 mogo2.set(true);
 kp = 0.175;
 wait(200, msec);
 turnRight(140);
 intakeInAuton();
 pid_inches(22);
 wait(0.3, sec);
 pid_inches(-8);
 turnLeft(12);
 pid_inches(16);
 wait(0.7, sec);
 pid_inches(-15);
 turnLeft(45);
 pid_inches(17);
 wait(0.7, sec);
 stopIntaking();
 pid_inches(-35);
}

 void progskills() {
 kp = 0.3;
 intakeInAuton();
 wait(0.71, sec);
 stopIntaking();
 pid_inches(14);
 kp = 0.115;
 turnLeft(82);
 pid_inches(-22);
 clamp();
 //This clamps onto the first mogo (right side red)
 wait(20, msec);
 kp = 0.19;
 //first ring
 turnRight(80);
 intakeInAuton();
 pid_inches(23);
 wait(1, sec);
 //second ring
 turnRight(80);
 pid_inches(17);
 intakeInAuton();
 pid_inches(7);
 wait(1.5, sec);
 pid_inches(5.7);
 //third ring
 turnLeft(80);
 kp = 0.15;
 intakeInAuton();
 pid_inches(20);
 wait(1, sec);
 kp = 0.19;
 //fourth ring
 pid_inches(-15);
 turnRight(183);
 intakeInAuton();
 pid_inches(25);
 stopWheels();
 wait(1, sec);
 //fifth ring
 turnLeft(5);
 pid_inches(10);
 intakeInAuton();
 wait(2, sec);
 pid_inches(-14);
 //sixth ring
 turnLeft(85);
 pid_inches(7);
 wait(2, sec);
 pid_inches(-7);
 //unclamp in the corner
 turnLeft(131.5);
 pid_inches(-15.8);
 stopIntaking();
 unclamp();
 //unclamps into corner and starts second quadrant
 kp = 0.15;
 pid_inches(10);
 kp = 0.09;
 turnRight(128.3);
 kp = 0.11;
 pid_inches(-50);
 kp = 0.1;
 pid_inches(-19);
 clamp();
 kp = 0.15;
 turnLeft(70);
 //first ring
 intakeInAuton();
 pid_inches(23);
 wait(1, sec);
 //secpnd ring
 turnLeft(70);
 pid_inches(23);
 wait(0.7, sec);
 pid_inches(5.7);
 turnRight(90);
 pid_inches(20);
 wait(1, sec);

 //
 //the next part was for only three ring
 // turnRight(163);
 // // this part turns, then gets the first ring of the first quadrant
 // kp = 1;
 // intakeInAuton();
 // pid_inches(15);
 // wait(1.3, sec);
 // // the second ring in the first quadrant
 // intakeInAuton();
 // pid_inches(5);
 // wait(1, sec);
 // pid_inches(-8);
 // //3rd ring in first quadrant
 // turnRight(74.5);
 // intakeInAuton(); 
 // pid_inches(6);
 // wait(1.5, sec);
 // //unclamps first mogo into corner
 // turnRight(120);
 // kp = 0.5;
 // pid_inches(-6.2);
 // unclamp();
 //allyson is a slay bestie westie and also izukiel is really annoying;
 // stopIntaking();
 // pid_inches(16);
 // kp = 0.2;
 // pid_inches(8);
 // wait(0.7, sec);
 // turnRight(140);
 // wait(0.6, sec);
 // kp = 0.17;
 // pid_inches(-20);
 // kp = 0.14;
 // pid_inches(-25);
 // //this clamps the second mogo of the second quadrent
 // clamp();
 // wait(0.8, sec);
 // turnRight(166);
 // //the first ring of the second quadrant
 // intakeInAuton();
 // pid_inches(24);
 // wait(0.75, sec);
 // //the second ring of the second quadrant
 // pid_inches(6);
 // wait(1, sec);
 // pid_inches(-15);
 // // third ring of second quadrant
 // turnLeft(40.1415962);
 // pid_inches(15);
 // wait(1, sec);
 // pid_inches(3);
 // turnLeft(160);
 // pid_inches(-4);
 // stopIntaking();
 // outtakeInAuton();
 // wait(0.1, sec);
 // kp = 0.8;
 // unclamp();
 // wait(1, sec);
 // pid_inches(10);
 // 3rd quadrant slaaay
 // pid_inches(10);
 // turnLeft(45);
 // pid_inches(72);
 }

int auton = 1;

//auton selector
void autonselector() {
 int numofautons = 8;
 if (controller1.ButtonRight.pressing()) {
   auton++;
   wait(200,msec);
 } else if (controller1.ButtonLeft.pressing()) {
   auton--;
   wait(200,msec);
 }
 if (auton > numofautons) {
   auton = 1;
 } else if (auton < 1) {
   auton = numofautons;
 }

 if (auton == 1) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,9);
   controller1.Screen.print("Blue Right");
 } else if (auton == 2) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,6);
   controller1.Screen.print("Blue Right Elims");
 } else if (auton == 3) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,6);
   controller1.Screen.print("Blue Goal Rush");
 } else if (auton == 4) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,4);
   controller1.Screen.print("Blue Goal Rush Elims");
 } else if (auton == 5) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,10);
   controller1.Screen.print("Red Left");
 } else if (auton == 6) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,6);
   controller1.Screen.print("Red Goal Rush");
 } else if (auton == 7) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,4);
   controller1.Screen.print("Red Goal Rush Elims");
 } else if (auton == 8) {
   controller1.Screen.clearScreen();
   controller1.Screen.setCursor(2,9);
   controller1.Screen.print("Prog Skills");
 }
}

// auton
void autonomous(void) {
 if (auton == 1) {
   blueRight();
 } else if (auton == 2){
   blueRightElims();
 } else if (auton == 3){
   blueGoalRush();
 } else if (auton == 4){
   blueGoalRushElims();
 } else if (auton == 5) {
   redLeft();
 } else if (auton == 6) {
   redGoalRush();
 } else if (auton == 7) {
   redGoalRushElims();
 } else if (auton == 8) {
   progskills();
 }
}

void wallstakessetposition(){
WallStakes.setVelocity(60, percent);
WallStakes2.setVelocity(60, percent);
if (controller1.ButtonUp.pressing()){
 while (rotationSensor.angle(degrees)<11) {
 // controller1.Screen.print(rotationSensor.angle(degrees));
 WallStakes.spin(forward);
 WallStakes2.spin(forward);
 }
 WallStakes.stop(hold);
 WallStakes2.stop(hold);
 }
}

void wallstakessetposition2(){
WallStakes.setVelocity(80, percent);
WallStakes2.setVelocity(80, percent);
if (controller1.ButtonUp.pressing()){
 WallStakes.spinFor(58,degrees);
 WallStakes2.spinFor(58,degrees);
}
}

void wallstakesscore() {
 WallStakes.setVelocity(90, percent);
 WallStakes2.setVelocity(90, percent);
if (controller1.ButtonY.pressing()) {
 WallStakes.spin(forward, 80, pct);
 WallStakes2.spin(forward, 80, pct);
} else if (controller1.ButtonA.pressing()) {
 WallStakes.spin(reverse, 80, pct);
 WallStakes2.spin(reverse, 80, pct);
} else {
 WallStakes.stop(hold);
 WallStakes2.stop(hold);
}
}

vex::task ColorSortRed() {

while(1) {
wait(10,msec);
opticalSensor.setLightPower(100,percent);

if (opticalSensor.color() == red) {
wait(250,msec);
sorter.set(true);
wait(250,msec);
sorter.set(false);
}
}
}

vex::task ColorSortBlue() {
while(1) {
wait(10,msec);
opticalSensor.setLightPower(100,percent);

if (opticalSensor.color() == blue) {
wait(250,msec);
sorter.set(true);
wait(250,msec);
sorter.set(false);
}
}
}

void stopWallStakes() {

}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

// driving
void old_arcade() {
//Slower
// int speedleft = controller1.Axis1.value()/2;
// int speedright = controller1.Axis3.value()/2;
// search up the ebot pilons turning curves(or something like that) desmos

double speedleft = controller1.Axis1.value() * 0.7 + controller1.Axis3.value();
double speedright = controller1.Axis1.value() * 0.62 - controller1.Axis3.value();

fl.spin(forward, speedleft, percent);
ml.spin(forward, speedleft, percent);
bl.spin(forward, speedleft, percent);

// RIGHT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!!
fr.spin(reverse, speedright, percent);
mr.spin(reverse, speedright, percent);
br.spin(reverse, speedright, percent);
}

// fast arcade
void fast_arcade() {
//Slower
// int speedleft = controller1.Axis1.value()/2;
// int speedright = controller1.Axis3.value()/2;
// search up the ebot pilons tur`ning curves(or something like that) desmos

double speedleft = controller1.Axis1.value() + controller1.Axis3.value();
double speedright = controller1.Axis1.value() - controller1.Axis3.value();

fl.spin(forward, speedleft, percent);
ml.spin(forward, speedleft, percent);
bl.spin(forward, speedleft, percent);

// RIGHT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!!
fr.spin(reverse, speedright, percent);
mr.spin(reverse, speedright, percent);
br.spin(reverse, speedright, percent);
}

void useFastArcade() {
 if (controller1.ButtonUp.pressing()) {
   fast_arcade();
 } else {
   old_arcade();
 }
}

// toggle slow arcade
bool toggleArcade = 0;

// void slowmode() {
//   auto now = steady_clock::now();
//   auto durLastToggle = duration_cast<milliseconds>(now-lastToggle).count();
//   if (durLastToggle > 200) {
//     toggleArcade = !toggleArcade;
//     lastToggle = now;
//   }
// }

bool selecting = 1;
void usercontrol() {
 while (selecting) {
   autonselector();
   if (controller1.ButtonB.pressing()) selecting = 0;
   wait(5, msec);
 }
 while (!selecting) {
   intaking();
   old_arcade();
   wallstakessetposition();
   //wallstakessetposition2();
   wallstakesscore();
   //mogoControl();
   controller1.ButtonL1.pressed(mogoControl);
   controller1.ButtonL1.released(clamping);
   controller1.ButtonL2.pressed(doinkerControl);
   controller1.ButtonL2.released(doinkeroinker);
   controller1.ButtonUp.pressed(wallstakessetposition2);
   wait(10,msec);
 }
}

// Main will set up the competition functions and callbacks.
//
int main() {
// Set up callbacks for autonomous and driver control periods.

Competition.autonomous(autonomous);
Competition.drivercontrol(usercontrol);
pre_auton();
ColorSortRed();
ColorSortBlue();

// Run the pre-autonomous function.
// Prevent main from exiting with an infinite loop.
while (true) {
  wait(10, msec);
}
}