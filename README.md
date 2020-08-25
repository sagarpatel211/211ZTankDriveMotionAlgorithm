# 211ZTankDriveMotionAlgorithm
This is our motion algorithm for a tank drive using odometry and 2 PID feedback loops! It allows us to create precise autonomous routines with straight and point turn motions.


## Table of Contents
* [Dependencies](#dependencies)
* [Installation](#installation)
* [Features](#features)
* [How To Use](#how-to-use)
* [Contributors](#contributors)
* [Contact](#contact)


## Dependencies
[VexCode Pro V5 Text 2.0.0 or later](https://www.vexrobotics.com/vexcode-download)


## Installation
* Make sure all the dependencies are installed
* Download the files
  * Option 1: 🍴 Fork this repository!
  * Option 2: 🧪 Clone the repository to your local machine using https://github.com/sagarpatel211/211ZTankDriveMotionAlgorithm.git!
* Open *211TankMotionAlgo.v5code* in VexCode to open the program
* Download the program to the brain by connecting the V5 Brain or controller to the device via micro-USB and select *download*. In both options, the V5 Brain must be on!
* Run the program by selecting it from the V5 Brain or pressing the *play* button in VexCode **if** the V5 Brain or controller is attached to the device via micro-USB.


## Features
* Contains odometry for robot position tracking
* Straight PID function for autonomous 
```
void StraightPID(double distance_desired, double StraightkP, double StraightkI, double StraightkD){ //function for driving straight
  while(DistanceAchieved != distance_desired) { //loop that exits once the robot achieves the desired target
    TrackPOS(); //function to track the robot position
    if(toggle == true){ //this snippet of code allows us to determine the starting coordinates of the robot before beginnning movement
      xStart = X;
      yStart = Y;
      toggle = false; //variable becomes false since we only need to do this once in the beginning
    }
    DistanceAchieved = sqrt(((X - xStart)*(X - xStart))+((Y - yStart)*(Y - yStart)));
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
```
* Point-Turn PID function for autonomous 
```
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
```


## How To Use
**To drive stright in autonomous:**
```
StraightPID(distance in inches (Note: negative values = moving backwards), kP value, kI value, kD value);
```

**To point turn in autonomous:**
```
TurnPID(degrees to turn (Note: default is counterclockwise, so negative values = clockwise), kP value, kI value, kD value);
```


## Contributors
| <a href="https://github.com/sagarpatel211" target="_blank">**Sagar Patel**</a> | <a href="http://github.com/saurinpatel20" target="_blank">**Saurin Patel**</a> |
| :---: |:---:|
| [![Sagar Patel](https://avatars1.githubusercontent.com/u/34544263?s=200)](https://github.com/sagarpatel211)    | [![Saurin Patel](https://avatars3.githubusercontent.com/u/62221622?s=200)](http://github.com/saurinpatel20) |
| <a href="https://github.com/sagarpatel211" target="_blank">`github.com/sagarpatel211`</a> | <a href="http://github.com/saurinpatel20" target="_blank">`github.com/saurinpatel20`</a> |


## Contact
[Email](mailto:patelsag@students.dsbn.org) | [Website](https://sagarpatel211.github.io/)
