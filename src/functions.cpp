// #include "vex.h"
// #include "functions.hpp"
// #include <string>
// #include <cmath>

// using namespace vex;


// void print(std::string text) {
//   controller1.Screen.clearScreen();
//   controller1.Screen.setCursor(2,10);
//   controller1.Screen.print(text.c_str());
// }

// void stopAllWheels(){
//  fl.stop();
//  ml.stop();
//  bl.stop();
//  fr.stop();
//  mr.stop();
//  br.stop();
// }

// //graphing data, used for PID tuning
// void graphPID(std::vector<int> errorHistory, std::vector<float> powerHistory, int goal, float error, int time) {
//   //goal is the PID goal (driveDistance)
//   //error history is a list of all of the errors (range is 0 to driveDistance)
//   //powerHistory is a list of the power applied (range is -1 to 1)
//   //error is the current error
//   //time is the current time, in milliseconds
  
//   //Setup: clear screen and draw the target line
//   Brain.Screen.clearScreen();
//   Brain.Screen.setPenWidth(2);
//   Brain.Screen.setPenColor(white);
//   Brain.Screen.drawLine(0, 60, 480, 60);
//   Brain.Screen.setPenWidth(1);
//   Brain.Screen.setPenColor(green);

//   //display final error and time
//   Brain.Screen.setCursor(1, 1);
//   Brain.Screen.clearLine(1);
//   Brain.Screen.print(" Final Error: ");
//   Brain.Screen.print(error);
//   Brain.Screen.print("    Time: ");
//   Brain.Screen.print(time);
  
//   //define the borders of the graph
//   int minY = 60; //error = 0 (robot is at target)
//   int maxY = 230; //error = driveDistance (Robot is at start)
//   int minX = 10; //time = beginning
//   int maxX = 470; //time = end
  
//   //loop through each data point and graph it
//   for (int i = 0; i < errorHistory.size() - 1; i++) { 
//     int x = minX + (maxX - minX) * i / errorHistory.size(); //find the x-value of this data point
    
//     //graph velocity
//     Brain.Screen.setPenColor(green);
//     Brain.Screen.drawLine(x, minY + (float)errorHistory.at(i) / goal * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), minY + (float)errorHistory.at(i + 1) / goal * (maxY - minY));
    
//     //graph power, changing color based on direction
//     if (powerHistory.at(i) > 0) {
//       Brain.Screen.setPenColor(orange);
//     } else {
//       Brain.Screen.setPenColor(yellow);
//     }
    
//     Brain.Screen.drawLine(x, maxY - std::abs(powerHistory.at(i)) * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), maxY - std::abs(powerHistory.at(i + 1)) * (maxY - minY));
//   }
// }

// int pid(double targetDistance) {
//   double kP = 0;
//   double kI = 0;
//   double kD = 0;
//   double error = 0;
//   double integral = 0;
//   double derivative = 0;
//   double prevError = 0;

//   double power = 0;
//   double prevPower = 0;

//   double errorBreak = 3;
//   double prevErrorBreak = 4;

//   fl.setPosition(0,deg);
//   fr.setPosition(0,deg);
//   ml.setPosition(0,deg);
//   mr.setPosition(0,deg);
//   bl.setPosition(0,deg);
//   br.setPosition(0,deg);

//   //lists
//   std::vector<int> errorHistory; //keep track of error over time
//   std::vector<float> powerHistory; //keep track of motor power over time
//   int currentTime = 0; //keep track of time over time (wow!)

//   while (true) {
//     double currentDistance = (fl.position(deg) + fr.position(deg) + ml.position(deg) + mr.position(deg) + bl.position(deg) + br.position(deg))/6;

//     error = targetDistance - currentDistance;

//     if(fabs(integral) > 200) {
//       integral += error;
//     }

//     derivative = error - prevError;
    
//     power = (kP*error) + (kI*integral) + (kD*derivative);

//     if (power > 1) power = 1;
//     if (power < -1) power = -1;

//     double slew = 0.1;

//     if (power > prevPower + slew) power = prevPower + slew;
//     if (power < prevPower - slew) power = prevPower - slew;

//     fl.spin(fwd,11*power,volt);
//     fr.spin(fwd,11*power,volt);
//     ml.spin(fwd,11*power,volt);
//     mr.spin(fwd,11*power,volt);
//     bl.spin(fwd,11*power,volt);
//     br.spin(fwd,11*power,volt);

//     if (error > errorBreak * -1 && error < errorBreak && error - prevError > prevErrorBreak * -1 && error - prevError < prevErrorBreak) break;

//     prevPower = power;
//     prevError = error;

//     //update histories and current time
//     errorHistory.push_back(error);
//     powerHistory.push_back(fabs(power));
//     currentTime += 20;

//     //graph the PIDs 
//     graphPID(errorHistory, powerHistory, targetDistance, error, currentTime);

//     wait(20,msec);
//   }

//   stopAllWheels();
//   return 0;
// }