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

using namespace vex;


// A global instance of competition
competition Competition;

bool clamptrue = false;
bool prevclamp = false;
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
  // All activities that occur before the competition starts
 // Example: clearing encoders, setting servo positions, ...
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*----------------------------- ----------------------------------------------*/


double kp = 0.175;
double ki = 1;
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


#define INCHES_TO_DEGREES 270/12
void pid_inches (double DistanceInInches) {
  double degrees = DistanceInInches * INCHES_TO_DEGREES;
  pid(degrees);
}

 void stopWheels() {
 // stop all motors in brake
 fl.stop(brake);
 ml.stop(brake);
 bl.stop(brake);

 fr.stop(brake);
 mr.stop(brake);
 br.stop(brake);
}

void clamp() {
 mogo.set(true);
 mogo2.set(true);
}

void unclamp() {
 mogo.set(false);
 mogo2.set(false);
}

void useMogo() {
    if (controller1.ButtonL1.pressing()) {
      clamp();
    } else if (controller1.ButtonL2.pressing()) {
      unclamp();
    }
}

using namespace std::chrono;

steady_clock::time_point lastClamp;

void mogoControl() {
  if (clamptrue) {
      clamp();
  } else {
     unclamp();
  }
}


void clamping() {
  auto now = steady_clock::now();
  auto durLastClamp = duration_cast<milliseconds>(now-lastClamp).count();
  if (durLastClamp > 200) {
    mogoControl();
    clamptrue = !clamptrue;
    lastClamp = now;
  }
}

void moveAllWheels(int SpeedLeft, int SpeedRight, int ) {
 fl.spin(forward, SpeedLeft + SpeedRight, percent);
 ml.spin(forward, SpeedLeft + SpeedRight, percent);
 bl.spin(forward, SpeedLeft + SpeedRight, percent);


 fr.spin(reverse, SpeedLeft - SpeedRight, percent);
 mr.spin(reverse, SpeedLeft - SpeedRight, percent);
 br.spin(reverse, SpeedLeft - SpeedRight, percent);
}


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

void intaking() {
 if (controller1.ButtonR2.pressing()) {
   intake.spin(forward, 350, rpm);
   intake2.spin(reverse, 395, rpm);
 } else if (controller1.ButtonR1.pressing()) {
   intake.spin(reverse, 80, pct);
   intake2.spin(fwd, 90, percent);
 } else {
   intake.stop(coast);
   intake2.stop(coast);
 }
}

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

void setVelocity(double vel) {
 // set all motors to velocity value of 'vel'
 fl.setVelocity(vel, percent);
 ml.setVelocity(vel, percent);
 bl.setVelocity(vel, percent);
 fr.setVelocity(vel, percent);
 mr.setVelocity(vel, percent);
 br.setVelocity(vel, percent);
}


void intakeInAuton() {
  intake.spin(reverse, 270, rpm);
  intake2.spin(forward, 300, rpm);
}

void outtakeInAuton() {
  intake.spin(forward, 300, rpm);
  intake2.spin(reverse, 330, rpm);
}

void stopIntaking() {
  intake.stop(coast);
  intake2.stop(coast);
}




  void blueright() {
    kp = 0.175;
    outtakeInAuton();
    wait(0.1, sec);
    stopIntaking();
    pid_inches(-30.5); 
    clamp();
    kp = 0.2;
    intakeInAuton();
    wait(0.9, sec);
    stopIntaking();
    turnLeft(125);
    kp = 0.15;
    pid_inches(16);
    intakeInAuton();
    pid_inches(3);
    wait(1.2, sec);
    pid_inches(-5); 
    turnRight(75);
    pid_inches(3);
    intakeInAuton();
    pid_inches(4);
    wait(2, sec);
    stopIntaking();
    kp = 0.3;
    turnRight(180);
    pid_inches(7);
  }

  void bluegoalrush() {
    kp = 0.3;
    pid_inches(-34);
    turnRight(23);
    kp = 0.15;
    pid_inches(-14);
    clamp();
    intakeInAuton();
    wait(1, sec);
    stopIntaking();
    turnLeft(35);
    pid_inches(15);
    intakeInAuton();
    wait(1, sec);
    stopIntaking();
    pid_inches(23);
    unclamp();
    pid_inches(6);
    turnRight(41.1234567);
    pid_inches(-21);
    clamp();
    pid_inches(-3);
    intakeInAuton();
    wait(1.5, sec);
    stopIntaking();
    unclamp();
  }

  void redleft() {
    kp = 0.19;
    outtakeInAuton();
    wait(0.1, sec);
    stopIntaking();
    pid_inches(-28);
    clamp();
    kp = 0.14;
    pid_inches(-3);
    intakeInAuton();
    turnRight(125);
    wait(0.75, sec);
    stopIntaking();
    pid_inches(16);
    intakeInAuton();
    pid_inches(3);
    wait(1.2, sec);
    pid_inches(-5);
    turnLeft(75);
    pid_inches(3);
    intakeInAuton();
    pid_inches(4);
    wait(2, sec);
    stopIntaking();
    kp = 0.3;
    turnLeft(180);
    pid_inches(7);
  }

  void redgoalrush() {
    kp = 0.3;
    pid_inches(-34);
    turnLeft(23);
    kp = 0.15;
    pid_inches(-14.2);
    clamp();
  }
  


int auton = 1;

void autonselector() {
  int numofautons = 4;
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
    controller1.Screen.print("Blue Goal Rush");
  } else if (auton == 3) {
      controller1.Screen.clearScreen();
            controller1.Screen.setCursor(2,10);
    controller1.Screen.print("Red Left");
  } else if (auton == 4) {
      controller1.Screen.clearScreen();
            controller1.Screen.setCursor(2,6);
    controller1.Screen.print("Red Goal Rush");
  }
}


void autonomous(void) {
  if (auton == 1) {
    blueright();
  } else if (auton == 2){
    bluegoalrush();
  } else if (auton == 3) {
    redleft();
  } else if (auton == 4) {
    redgoalrush();
  }
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

void old_arcade() {
 //Slower
 // int speedleft = controller1.Axis1.value()/2;
 // int speedright = controller1.Axis3.value()/2;
 // search up the ebot pilons turning curves(or something like that) desmos

 double speedleft = controller1.Axis1.value() * 0.75 + controller1.Axis3.value();
 double speedright = controller1.Axis1.value() * 0.75 - controller1.Axis3.value();

 fl.spin(forward, speedleft, percent);
 ml.spin(forward, speedleft, percent);
 bl.spin(forward, speedleft, percent);

 // RIGHT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!!
 fr.spin(reverse, speedright, percent);
 mr.spin(reverse, speedright, percent);
 br.spin(reverse, speedright, percent);
}

void slow_arcade() {
 //Slower
 // int speedleft = controller1.Axis1.value()/2;
 // int speedright = controller1.Axis3.value()/2;
 // search up the ebot pilons turning curves(or something like that) desmos

 double speedleft = controller1.Axis1.value()/2 + controller1.Axis3.value();
 double speedright = controller1.Axis1.value() - controller1.Axis3.value()/2;

 fl.spin(forward, speedleft, percent);
 ml.spin(forward, speedleft, percent);
 bl.spin(forward, speedleft, percent);

 // RIGHT MOTORS ARE REVERSED SO FORWARD = REVERSE!!!!!!!!!
 fr.spin(reverse, speedright, percent);
 mr.spin(reverse, speedright, percent);
 br.spin(reverse, speedright, percent);
}

void useSlowArcade() {
  if (controller1.ButtonL2.pressing()) {
    slow_arcade();
  } else {
    old_arcade();
  }
}


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
    //mogoControl();
    controller1.ButtonL1.pressed(mogoControl);
    controller1.ButtonL1.released(clamping);
    wait(10,msec);
  }
}

// Main will set up the competition functions and callbacks.
//
int main() {
 // Set up callbacks for autonomous and driver control periods.
 pre_auton();
 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);

 // Run the pre-autonomous function.
 
 
 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(10, msec);
 }
}