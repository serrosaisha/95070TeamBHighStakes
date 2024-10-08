#include "vex.h"


using namespace vex;
using signature = vision::signature;
using code = vision::code;


// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
pneumatics mogo = pneumatics(Brain.ThreeWirePort.A);
pneumatics mogo2 = pneumatics(Brain.ThreeWirePort.B);
pneumatics doinker = pneumatics(Brain.ThreeWirePort.C);
controller controller1 = controller(primary);
motor fr = motor(PORT1, ratio18_1, false);
motor fl = motor(PORT7, ratio18_1, true);
motor ml = motor(PORT10, ratio18_1, true);
motor mr = motor(PORT5, ratio18_1, false);
motor bl = motor(PORT11, ratio18_1, true);
motor br = motor(PORT4, ratio18_1, false);
motor intake = motor(PORT17, ratio18_1, false);
motor intake2 = motor(PORT15, ratio18_1, false);
inertial inertialSensor = inertial(PORT12);
// VEXcode device constructors




// VEXcode generated functions






/**
* Used to initialize code/tasks/devices added using tools in VEXcode Pro.
*
* This should be called at the start of your int main function.
*/
void vexcodeInit( void ) {
}