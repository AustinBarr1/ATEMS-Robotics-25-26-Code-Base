//Adding neccessary settings and files
#include "vex.h"                                      
using namespace vex;                                
competition Competition; 

////////////////////////////////////////////////////////////

//Initializing Important things

//Brain and Controller
brain  Brain;              
controller myController = controller(primary);

//Motors, Sensors, and Pneumatics
motor front_left = motor(PORT2, ratio18_1, true); // Change false to true to change the + direction
motor back_left = motor(PORT4, ratio18_1, true); // ratio18_1 is green motors (Normal)
motor front_right = motor(PORT3, ratio18_1, false); // ratio36_1 is red motors (Power)
motor back_right = motor(PORT6, ratio18_1, false); // ratio6_1 is blue motors (Speed)
inertial inertialsensor = inertial(PORT1);
motor_group left_motors = motor_group(front_left, back_left);
motor_group right_motors = motor_group(front_right, back_right);
motor_group alldrive = motor_group(front_left, back_left, front_right, back_right);

motor intakemotor = motor(PORT9, ratio36_1, true); 
motor conveyormotor = motor(PORT10, ratio36_1, false);


//Drivetrain Function(in inches)
const double wheelTravel = 12.56; 
const int trackWidth = 16; //Width between wheels
const int wheelbase = 12; //Length between wheels
drivetrain myDrivetrain(left_motors, right_motors, wheelTravel, trackWidth, wheelbase, distanceUnits::in);

////////////////////////////////////////////////////////////

//PD Variables and Constants

//Constants
double kP = 0.8;
double kI = 0;
double kD = 0.15;
double turnkP = 2.0;
double turnkI = 0.0;
double turnkD = 0.15;
////////////////////////////////////////////////////////////
//Position based variables
int error;  
int prevError = 0;
int derivative; 
int turnError; 
int turnPrevError = 0; 
int turnDerivative;
///Changable variables
int desiredPosition = 0;
int desiredTurnPosition = 0;


// controller enable/disable           
bool RemoteControlCodeEnabled = true; 
                                    
void vexcodeInit( void ) {          
  // nothing to initialize        
}                                

////////////////////////////////////////////////////////////

//PD Function


int runDrivePID(int DesiredPos, int DesiredTurn){
  left_motors.setPosition(0,degrees);
  right_motors.setPosition(0,degrees);
  conveyormotor.setPosition(0,degrees);
  desiredPosition = (DesiredPos / (4.125 * 3.14159)) * 360.0;
  desiredTurnPosition = DesiredTurn;


  while(true) {
    myController.Screen.setCursor(3,16);                                
    myController.Screen.print(turnError);


    ////////////////////////////////////////////
    ///Forward and Backwards PD
    ////////////////////////////////////////////


    //get your average position
    int leftMotorPosition = (front_left.position(degrees) + back_left.position(degrees))/2;
    int rightMotorPosition = (front_right.position(degrees) + back_right.position(degrees))/2;
    int averagePosition = (leftMotorPosition + rightMotorPosition)/2;


    //Calculate the error
    error = averagePosition - desiredPosition;


    //Calculating derivative
    derivative = error - prevError;


    int totalError =+ error;


    //finding the motor power
    double motorPower = ((error * kP + derivative * kD + totalError * kI) / 12) * -1;


    ////////////////////////////////////////////
    ///Turning PD
    ////////////////////////////////////////////


    // int turnDifference = (leftMotorPosition - rightMotorPosition);


    //Calculate the error
    turnError = inertialsensor.rotation() - desiredTurnPosition;


    //Calculating derivative
    turnDerivative = turnError - turnPrevError;


    //finding the motor power
    double turningPower = ((turnError * turnkP + turnDerivative * turnkD) / 12) * -1;


    //applying motor power
    left_motors.spin(forward, motorPower + turningPower, voltageUnits::volt);
    right_motors.spin(forward, motorPower - turningPower, voltageUnits::volt);


    if(abs(error) < 25 && abs(turnError) < 25){
      left_motors.stop();
      right_motors.stop();
      inertialsensor.resetRotation();
      break;
    }
    else{
    }
    prevError = error;
    turnPrevError = turnError;
    task::sleep(20);
  }
  task::sleep(1000);
  return 1;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Controller Functions

void driverControls(){
while(true){
  //Make the robot drive based on controller stick positions
  left_motors.setStopping(brake);  
  right_motors.setStopping(brake); 
  int forwardSpeed = (myController.Axis3.position()) * -1;                                                
  int turnSpeed = (myController.Axis1.position()) * -1.25;
                                                                                                                                                                                                                                                             
  left_motors.spin(reverse, forwardSpeed + turnSpeed, percent);             
  right_motors.spin(reverse, forwardSpeed - turnSpeed, percent);            
  vex::task::sleep(20);
  }}           
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Operate the conveyor system, intake, climbing system, and pneumatics using the controller buttons
void buttonFunctions(){                                                               
  while(true){                                                                  
    if (myController.ButtonUp.pressing()) {                
      intakemotor.spin(forward,-100,percent);                
    }                                         
                                                                              
   if (myController.ButtonDown.pressing()){     
    intakemotor.spin(forward,100,percent);
    conveyormotor.spin(forward,100,percent);
 }
    if (myController.ButtonR2.pressing()){     
    conveyormotor.spin(forward,-100,percent);
    intakemotor.spin(forward,-100,percent); 
 }
     if (myController.ButtonR1.pressing()){     
    intakemotor.spin(forward,0,percent);
    conveyormotor.spin(forward,0,percent);
 }


 vex::task::sleep(20);
 }}                                                                  



//Print motor temperatures to the controller screen
void controllerControllerInMyHand(){     
while(true){  
  myController.Screen.setCursor(1,1);                              
  myController.Screen.print(front_left.power());  
  myController.Screen.setCursor(1,16);                               
  myController.Screen.print(back_left.power());     
  myController.Screen.setCursor(2,1);                          
  myController.Screen.print(front_right.power());   
  myController.Screen.setCursor(2,16);                               
  myController.Screen.print(back_right.power());         
  myController.Screen.setCursor(3,1);                                          
  myController.Screen.print(inertialsensor.heading()); 


  wait(500,msec);                               
}}                                                                  

/************************************* V Competition Commands V ****************************************/

void pre_auton(void) {    //Resetting everything before the match
  left_motors.setPosition(0,degrees);
  right_motors.setPosition(0,degrees);
  conveyormotor.setPosition(0,degrees);
  intakemotor.setPosition(0,degrees);
  inertialsensor.calibrate();
  while(inertialsensor.isCalibrating()){
    wait(50, msec);
  }
}

void autonomous() {
  intakemotor.spin(forward,-100,percent);
  wait(100, msec);
  runDrivePID(-70, 0);
  intakemotor.spin(forward,0,percent);
  runDrivePID(10, 0);
  runDrivePID(0, -60);
  runDrivePID(-14,0);
  intakemotor.spin(forward,100,percent);
  conveyormotor.spin(forward,100,percent);
  wait(2000, msec);
  intakemotor.spin(forward,0,percent);
  conveyormotor.spin(forward,0,percent);
/*
Right 
  intakemotor.spin(forward,-100,percent);
  wait(100, msec);
  runDrivePID(-70, 0);
  intakemotor.spin(forward,0,percent);
  runDrivePID(10, 0);
  runDrivePID(0, -60);
  runDrivePID(-14,0);
  intakemotor.spin(forward,100,percent);
  conveyormotor.spin(forward,100,percent);
  wait(2000, msec);
  intakemotor.spin(forward,0,percent);
  conveyormotor.spin(forward,0,percent);

  Left
    intakemotor.spin(forward,-100,percent);
  wait(100, msec);
  runDrivePID(-70, 0);
  intakemotor.spin(forward,0,percent);
  wait(1000, msec);
  runDrivePID(10, 0);
  wait(100, msec);
  runDrivePID(0, -120);
  wait(100, msec);
  runDrivePID(25  , 0);
  intakemotor.spin(forward,-100,percent);
  conveyormotor.spin(forward,-100,percent);
*/
}

void userControl() {    //Telling the robot how to function during driver control

  //Adding all the functions
  thread driver = thread(driverControls);                                                                                              
  thread buttons = thread(buttonFunctions);                                                                                                     
  thread EnterTextHere = thread(controllerControllerInMyHand);


  while (true) {                       
    this_thread::sleep_for(10);        
    //Add this to stop the CPU from overheating
  }                                 
}                                   


/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Setting up everything together, make sure this is last
int main() {                                             
  Competition.autonomous(autonomous);      
  Competition.drivercontrol(userControl);                                         
  pre_auton();
  //Don't let exit the loop                       
  while (true) {                   
    wait(100, msec);             
  }                              
}                               
