/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       sagarpatel                                                */
/*    Description:  Tank Drive Motion Algorithm Using 2 PIDs                  */
/*    Credits:      Cody Nelson from 98881A for answering our questions       */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RightFront           motor         18              
// RightBack            motor         17              
// LeftFront            motor         20              
// LeftBack             motor         14              
// Controller1          controller                    
// MotorExample         motor         13              
// ---- END VEXCODE CONFIGURED DEVICES ----

/*-------------------------------Includes------------------------------------*/
#include "vex.h"
#include <math.h>
#include <iostream> 
using namespace vex;
vex::competition    Competition;
/*-------------------------------Variables-----------------------------------*/
//Variables are defined here for use in the Odometry calculations
#define Pi 3.14159265358979323846
#define SL 5 //distance from tracking center to middle of left wheel
#define SR 5 //distance from tracking center to middle of right wheel
#define SS 7.75 //distance from tracking center to middle of the tracking wheel
#define WheelDiam 4.125 //diameter of all the wheels being used for tracking
#define tpr 360  //Degrees per single encoder rotation
double DeltaL,DeltaR,DeltaB,currentL,currentR,currentB,PreviousL,PreviousR,PreviousB,DeltaTheta;
double X,Y,Theta,DeltaXSide,DeltaYSide,DeltaXBack,DeltaYBack,SideChord,BackChord,OdomHeading;
//PID Variables for straight and turning algorithms
double  DistanceAchieved,StraightDesired,StraightmVrequest,StraightError,xStart,yStart,TurnmVrequest,TurnError;
double StraightkP,StraightPreviousError,StraightDerivative,StraightkD,StraightIntegral,StraightkI,StraightCount,
TurnkP,TurnPreviousError,TurnDerivative,TurnkD,TurnIntegral,TurnkI,TurnCount = 0;
bool toggle = true; //toggle to find the starting point of each 'drive straight' function
/*---------------------------------------------------------------------------*/
/*                              Motion Functions                             */
/*---------------------------------------------------------------------------*/
void TrackPOS() {
  // 2 cases could be occuring in odometry
  // 1: Going in a straight line
  // 2: Going in an arc motion
  // If the bot is on an angle and going straight the displacement would be linear at angle Theta, meaning a right triangle is formed (Trig ratios to calc movement)
  // Since it is a linear motion, the Left and right will move the same amount so we can just pick a side and do our movement calculation
  // Since this calculation is working based of very infinitely small arcs, the displacement of the robot will be a chord
  // Below it Averages the Left and Right integrated motor encoders since we don't have encoders yet
  currentR = (RightFront.position(degrees) + RightBack.position(degrees)) / 2;
  currentL = (LeftFront.position(degrees) + LeftBack.position(degrees)) / 2;

  //Creates variables for change in each side info in inches (12.9590697 is circumference of wheel)
  DeltaL = ((currentL - PreviousL) * 12.9590697) / tpr;
  DeltaR = ((currentR - PreviousR) * 12.9590697) / tpr;
  //DeltaB = ((currentB - PreviousB) * 12.9590697) / tpr;

  //Determines the change in angle of the robot using the rotational change in each side
  DeltaTheta = (DeltaR - DeltaL) / (SL + SR);

  //Creates an if/else statement to prevent NaN values from appearing and causing issues with calculation
  if(DeltaTheta == 0) {  //If there is no change in angle
    X += DeltaL * sin (Theta);
    Y += DeltaL * cos (Theta);
    //X += DeltaB * cos (Theta + 1.57079633);
    //Y += DeltaB * sin (Theta + 1.57079633);

  //If there is a change in angle, it will calculate the changes in X,Y from chords of an arc/circle.
  } else {  //If the angle changes
      SideChord = 2 * ((DeltaL / DeltaTheta) + SL) * sin (DeltaTheta / 2);
      //BackChord = 2 * ((DeltaB / DeltaTheta) + SS) * sin (DeltaTheta / 2);
      DeltaYSide = SideChord * cos (Theta + (DeltaTheta / 2));
      DeltaXSide = SideChord * sin (Theta + (DeltaTheta / 2));
      //DeltaXBack = BackChord * sin (Theta + (DeltaTheta / 2));
      //DeltaYBack = -BackChord * cos (Theta + (DeltaTheta / 2));
      Theta += DeltaTheta;
      X += DeltaXSide;
      Y += DeltaYSide;
    }

    //Odom heading is converting the radian value of Theta into degrees
    OdomHeading = Theta * 57.295779513;

    //Converts values into newer values to allow for code to effectively work in next cycle
    PreviousL = currentL;
    PreviousR = currentR;
    DeltaTheta = 0;

    //This is for printing to the brain for debugging
    Brain.Screen.printAt(100,20, "X: %f",X);
    Brain.Screen.printAt(100,40, "Y: %f",Y);
    Brain.Screen.printAt(100,60, "Theta: %f",Theta);
    Brain.Screen.printAt(100,80, "Angle: %f",OdomHeading);
    Brain.Screen.printAt(100,100, "Displacement1: %f",SideChord);
    Brain.Screen.printAt(100,120, "DeltaLeftInches: %f",DeltaL);
    Brain.Screen.printAt(100,140, "DeltaRightInches: %f",DeltaR);
    Brain.Screen.printAt(100,160, "DeltaX: %f",DeltaXSide);
    Brain.Screen.printAt(100,180, "DeltaY: %f",DeltaYSide);
}
void StraightPID(double distance_desired, double StraightkP, double StraightkI, double StraightkD){ //function for driving straight
  while(DistanceAchieved != distance_desired) { //loop that exits once the robot achieves the desired target
    TrackPOS(); //function to track the robot position
    if(toggle == true){ //this snippet of code allows us to determine the starting coordinates of the robot before beginnning movement
      xStart = X;
      yStart = Y;
      toggle = false; //variable becomes false since we only need to do this once in the beginning
    }
    DistanceAchieved = sqrt(((X - xStart)*(X - xStart))+((Y - yStart)*(Y - yStart)));//uses pythagorean theorem to calculate error since the robot could be at an angle
    StraightError = distance_desired - DistanceAchieved; //error is calculated by subtracting the desired to the actual value
    //derivative is calculated by error minus the previous error, which is obtained from the error before outputting motor values
    StraightDerivative = StraightError - StraightPreviousError; 
    //integral portion of PID loop
    if ((StraightError < 0.07) && (StraightError > -0.07)){ //if the error is insignifant to us, which is 0.07 inches...
      StraightIntegral = 0;
      StraightCount += 1;
    }
    else { 
      StraightIntegral += StraightError; 
      StraightCount = 0;
    }
    if (StraightCount >= 10) {
    //if the variable is 10, it breaks from the function. even though it may not achieve the exact location, it will be a very small error and save us some time
       break;
    }
    StraightmVrequest = StraightError * StraightkP + StraightDerivative * StraightkD + StraightIntegral * StraightkI;//calculates the motor values from the math above
    LeftFront.spin(forward, StraightmVrequest, voltageUnits::mV); //Motion
    LeftBack.spin(forward, StraightmVrequest, voltageUnits::mV);  //Motion
    RightFront.spin(forward, StraightmVrequest, voltageUnits::mV);//Motion
    RightBack.spin(forward, StraightmVrequest, voltageUnits::mV); //Motion
    StraightPreviousError = StraightError; //previous error is calculated here from the error
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
void TurnPID(double angle_desired, double TurnkP, double TurnkI, double TurnkD){ //function for turning to a specific angle
  while(angle_desired != OdomHeading) { //loop tht exits once the robot reaches target
    TrackPOS(); //function to track the robot position
    TurnError = angle_desired - OdomHeading; //calculates error by subtracting the desired angle and actual angle
    TurnDerivative = TurnError - TurnPreviousError; //the derivative is calculated by subtracting the error by previous error
    //integral portion of PID loop
    if ((TurnError < 0.5) && (TurnError > -0.5)){ //if the error is insignifant to us, which is 0.5 degrees...
      TurnIntegral = 0;  
      TurnCount += 1;
    }
    else { 
      TurnIntegral += TurnError; 
      TurnCount = 0;
    }
    if (TurnCount >= 10) { //if turncount is 10...
      break; //it will break from the loop since it is taking to long to reach to target so it will break after 10 counts
    }
    TurnmVrequest = TurnError * TurnkP + TurnDerivative * TurnkD + TurnIntegral * TurnkI;//calculates the motor values from the math above
    LeftFront.spin(reverse, TurnmVrequest, voltageUnits::mV); //Motion
    LeftBack.spin(reverse, TurnmVrequest, voltageUnits::mV);  //Motion
    RightFront.spin(forward, TurnmVrequest, voltageUnits::mV);//Motion
    RightBack.spin(forward, TurnmVrequest, voltageUnits::mV); //Motion
    TurnPreviousError = TurnError; //previous error is calculated here from the error
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
void VariableReset ( void ){ //This resets all the variables for the PID functions
  TurnPreviousError = 0;
  TurnError = 0;
  TurnDerivative = 0;
  TurnIntegral = 0;
  TurnCount = 0;
  StraightPreviousError = 0;
  StraightError = 0;
  StraightDerivative = 0;
  StraightIntegral = 0;
  toggle = true;
  xStart = 0;
  yStart = 0;
  StraightmVrequest = 0;
  TurnmVrequest = 0;
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
void pre_auton( void ) {
  vexcodeInit(); //Initializing Robot Configuration - Required!!!
  Brain.resetTimer(); //Resets The Brain Timer
  RightFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  RightBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftFront.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
  LeftBack.resetRotation(); //Resets Motor Values For Accuracy In Autonomous
}
/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomous( void ) {
  MotorExample.spin(forward,100,velocityUnits::pct); //sets an example motor to 100 to see if it will run while moving
  TurnPID(80, 160, 0, 0); //Example: Turns the robot 80 degrees counterclockwise (CAST)
  MotorExample.stop(brakeType::brake); //sets to motor to 0 to stop it
  VariableReset (); //Resets all values so the next function can run (just in case!!)
  StraightPID(10, 2000, 0, 0); //Example: Robot drives stright 10 inches
  VariableReset (); //Resets all values so the next function can run (just in case!!)
  TurnPID(90, 400, 0, 0); //Example: Turns the robot 90 degrees counterclockwise
}
/*----------------------------------------------------------------------------*/
/*                              User Control Task                             */
/*----------------------------------------------------------------------------*/
void usercontrol( void ) {
  while (1){
    Brain.Screen.clearScreen(); //clears the screen to continuously display the odometry info
    //provides power to the motors to allow for movement of robot for testing using the controller
    LeftBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    LeftFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() + (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd, ((Controller1.Axis3.value() - (Controller1.Axis1.value())*0.1)), vex::velocityUnits::pct);
    TrackPOS(); //Calls the TrackPosition function
    Brain.Screen.render(); //push data to the LCD all at once to prevent image flickering
    vex::task::sleep(10); //Slight delay so the Brain doesn't overprocess
  }
}
int main() {
    pre_auton(); //Calls the pre-autonomous function
    Competition.autonomous( autonomous ); //Calls the autonomous function
    Competition.drivercontrol( usercontrol ); //Calls the driver control function
    while(1) {
      vex::task::sleep(15); //Slight delay so the Brain doesn't overprocess
    }
}
