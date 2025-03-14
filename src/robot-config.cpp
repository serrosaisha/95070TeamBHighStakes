#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
pneumatics mogo = pneumatics(Brain.ThreeWirePort.C);
pneumatics mogo2 = pneumatics(Brain.ThreeWirePort.B);
pneumatics doinker = pneumatics(Brain.ThreeWirePort.A);
pneumatics sorter = pneumatics(Brain.ThreeWirePort.H);
pneumatics lift = pneumatics(Brain.ThreeWirePort.D);
rotation  rotationSensor = rotation (PORT21, false);
controller controller1 = controller(primary);
motor fr = motor(PORT15, ratio18_1, false);
motor fl = motor(PORT7, ratio18_1, true);
motor ml = motor(PORT2, ratio18_1, true);
motor mr = motor(PORT12, ratio18_1, false);
motor bl = motor(PORT1, ratio18_1, true);
motor br = motor(PORT11, ratio18_1, false);
motor intake = motor(PORT16, ratio18_1, false);
motor WallStakes = motor(PORT4, ratio18_1, false);
motor WallStakes2 = motor(PORT10, ratio18_1, true);
// motor intake2 = motor(PORT15, ratio18_1, false);
inertial inertialSensor = inertial(PORT18);
optical opticalSensor = optical(PORT6);
// VEXcode device constructors

// VEXcode generated functions

/**
* Used to initialize code/tasks/devices added using tools in VEXcode Pro.
*
* This should be called at the start of your int main function.
*/
void vexcodeInit (void) {
}