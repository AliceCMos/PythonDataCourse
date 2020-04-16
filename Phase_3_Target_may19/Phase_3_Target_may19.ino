// SCARA TASK - Phase 3
// Reward delivered when entering Circle target area
// by Alice Mosberger Nov 2017

// changes made by Alice Mosberger Jan 2018:
// - second capacitive sensor for lick detection (no IR-LED anymore)
// - sending digital triggers to arduino UNO (in Firmata) to sync with Ephys setup
// - Motor Delay after Target
// - Reward counter to stop session after X rewards

// changes made by Richard Hormigo Feb 2018:
// addressing each motor with its own serial connection

// changes made by Alice Mosberger March 2018:
// - Joystick moves back after x amount of time if target not reached.

// Changes by Alice Mosberger Apr 2018:
// - NEW CIRCLE TARGET AND HOME
// - TouchOff interval before motors engage (TouchOffTime) > 250msec --> always home

// changes made by Alice Mosberger May 2018:
// - Reward delayed to LED (RewDelayTime)

// changes made by Alice Mosberger June 2018:
// New strings for most events, to shorten and also to simplify the matlab analysis

// changes made by Alice Mosberger Jan 2019:
// - added third touch sensor for paw rest of left paw
// - changed pin assignment to clean up breadboard a bit

// changes made by Alice Mosberger Feb 2019:
// - new capacitive sensor pin assignment
// - change to c strings for serial print and added functions

// changes made by Alice Mosberger May 2019:
// - Changed LED to click sound (piezo)
// - Introduced a stay at home time (motors disengage only after that) --> increase ITI

/*
  Maxon EPOS4 Initial Driver for Arduino  
  Simplified CAN protocol over Serial Interface, with no validation implemented
  By R.Hormigo, Zuckerman Institute AIC 
  Columbia University 2017
  */ 

#include <CapacitiveSensor.h> // to detect touch of joystick

// create an instance of the library
// pin 12/4 sends electrical energy
// pin 11/3 senses senses a change
CapacitiveSensor capSensor1 = CapacitiveSensor(7, 6);
CapacitiveSensor capSensor2 = CapacitiveSensor(4, 3);
//CapacitiveSensor capSensor3 = CapacitiveSensor(9, 8);

#define HDR 0x0290
#define VERSION "1.0"
byte node;
int pos; // For -2047, 2048 steps, 4096 steps around the shaft
bool vErr= false; //validation error
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MODIFY THESE VALUES TO CHANGE THE TASK REQUIREMENTS /////////////////////////////////////////////////

int MaxRewards = 150; // define the maximal number of reward in this session
int RewardCounter = 0;

int MotorDelayTime = 750; //750msec before motors engage, after target hit
int RewDelayTime = 0; //500msec before reward is delivered after target hit
int StayHomeTime = 1000; //Time before motors disengage once home is reached --> increasing the ITI

int OutofHomeTime = 7500; //Time in ms allowed outside home before motors engage. 
int TouchOffTime = 100; //Time of Touch off allowed before motors move home

double Target_X = 1.00;                   // Center Target: X = 0.00, Y = 55.7
double Target_Y = 57.60;                  // Left Target:   X = -5.50. Y = 57.50
                                          // Right Target:  X = 5.50, Y = 57.50
double OldTarget_X = -7.10;
double OldTarget_Y = 62.70;                                
                                          
double Target_r = 2.50; // Radius of Target circle

long SolenoidTime = 53; //time solenoid is open in msec

int ToneFreq = 10000; //frequency in Hz of tone
int ToneDur = 2; //msec duration of tone

/////////////////////////////////////////////////  SETUP OF VARIABLES - do not change! /////////////////////////////////////////////////////////

int Home_Step1 = -1203;
int Home_Step2 = -845;

double Home_X = 0.00;
double Home_Y = 65.00;

double Home_r = 1.00;

int step1 = 0;
int step2 = 0;

const int L0 = 59;
const int L1 = 35;
const int L2 = 35;
const int L3 = 50;
const int L4 = 50;

double P1x = 0;
double P1y = 0;
double P2x = 0;
double P2y = 0;
double P3x = 0;
double P3y = 0;
double P4x = 0;
double P4y = 0;
double P5x = 0;
double P5y = 0;

double Dist1 = 0;
double Dist2 = 0;

float alpha = 0;
float beta = 0;

float th1 = 0;
float th2 = 0;

double Dist_Home = 0; //Distance to Home
double Dist_Target = 0; //Distance to Target
double Dist_OldTarget = 0; // Distance to previous Target

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

int Touch_threshold = 75; //threshold for detecting touch
int Lick_threshold = 75; // value needed to detect lick (with capacitive sensor)
//int Rest_threshold = 75;

long TouchSensor;
long LickSensor;
//long RestSensor;

const int SolenoidPin = 11;
const int ClickPin = 12;  //signals REWARD NOW
const int ButtonPin = 23; // button to elicit reward manually by experimenter
//const int SHomePin = 23; //pin for syncing - Home
//const int STouchPin = 25; //pin for syncing - Touch
//const int SRewardPin = 27; //pin for syncing - Reward
//const int SLickPin = 29; // pin for syncing - Lick

unsigned long previousMillisMotorDelay;
unsigned long previousMillisRewDelay;
unsigned long previousMillisSolenoid;
unsigned long currentMillis;
unsigned long previousMillisOutofHome;
unsigned long previousMillisTouchOff;
unsigned long previousMillisHome;

int SolState = LOW; //variable for Solenoid status
int TouchState = LOW; //variable for Touch status
int LickState = LOW; //variable for Lick status
int ButtonPress = HIGH; //detecting button press
int ButtonState = LOW;
int TargetState = LOW; // for detecting threshold crossing --> only one reward when crossing, then back to 0/0 position needed.
int MotorState = LOW; //state to indicate that motors are engaged.
int HomeState = HIGH;
int InactiveState = LOW; // State goes HIGH when outside of home too long (no reward target)
int MotorDelay = LOW; // state to define delay state before engaging motors after reward
int RewDelay = LOW; // state to define delay state for solenoid opening
//int RestState = LOW;
int OldTargetState = LOW;
        
int val1 = 0;     // variables to store the value of the joystick
int val2 = 0;


///////////////////////////// SETTING UP ALL STRINGS FOR PRINTING //////////////////////////////////////////////
const char *XPosition = "XPO";
const char *YPosition = "YPO";
const char *TimeMSec = "TMS";
const char *TouchEnd = "TOE";
const char *TouchStart = "TOS";
//const char *RestEnd = "REE";
//const char *RestStart = "RES";
const char *LeavHome = "LHO";
const char *EntHome = "EHO";
const char *Lick = "LIC";
const char *Hit = "HIT";
const char *Solenoid = "SOL";
const char *ManRew = "MAR";
const char *MotorsTE = "MTE";
const char *MotorsON = "MON";
const char *MotorsOFF = "MOF";
const char *OldTarget = "OTA";
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////SETTING UP I/O and SERIAL///////////////////////////////////////////////////////////////////////


void setup() {
  // initialize serial 0 and 1:
  Serial.begin(115200);  //Port 0 reads and runs simple ASCII comands 
  Serial1.begin(115200); //Port 1 EPOS4 defaults at 115K and runs in binary.
  Serial2.begin(115200); //Port 1 EPOS4 defaults at 115K and runs in binary.
  Serial.flush();
  Serial1.flush(); 
  Serial2.flush(); 
  //Serial.print("   Simple protocol to command EPOS4 with examples "); 
  //Serial.println(VERSION);
  //serialPrintProtocol();

  capSensor1.reset_CS_AutoCal(); //autocalibrate once at beginning of the session
  capSensor2.reset_CS_AutoCal();
  //capSensor3.reset_CS_AutoCal();

  capSensor1.set_CS_AutocaL_Millis(0xFFFFFFFF); //do not autocalibrate capacitive sensors during task!
  capSensor2.set_CS_AutocaL_Millis(0xFFFFFFFF);
 //capSensor3.set_CS_AutocaL_Millis(0xFFFFFFFF);

//////////////////////////////////////////////////////////// pin Setup ///////////////////////////////////////////
  pinMode(SolenoidPin, OUTPUT); // initializing pins
  pinMode(ClickPin, OUTPUT);
  pinMode(ButtonPin, INPUT);
  
//  pinMode(SHomePin, OUTPUT); // send to other Arduino board
//  pinMode(STouchPin, OUTPUT);
//  pinMode(SRewardPin, OUTPUT);
//  pinMode(SLickPin, OUTPUT);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

sHomePos(1);
sHomePos(2);
delay(10);
myflushSerial12(); //Clear answers  
}

String inCmd;

/////////////////////////////////////////////////////// THIS LOOP RUNS THE TASK /////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

currentMillis = millis(); 

ButtonPress = digitalRead(ButtonPin);

// store the value reported by the sensor in a variable
TouchSensor = capSensor1.capacitiveSensor(15); //15 works but slow, trying lower again
LickSensor = capSensor2.capacitiveSensor(15);
//RestSensor = capSensor3.capacitiveSensor(15);


val1 = cPosition(2);    // read the Scara position of motor 1 and 2
val2 = cPosition(1);
myflushSerial12(); //Clear answers

///////////////// Forward Kinematics to determine Position P5x and P5y /////////////////////////////

step1 = val1 + (Home_Step1); //Entry1 (negative steps are counterclock)
step2 = val2 + (Home_Step2); //Entry2 (negative steps are counterclock)

th1=((2.0*M_PI)/4096)*step1; //Convert step1 position to theta radians
th2=((2.0*M_PI)/4096)*step2; //Convert step2 position to theta radians

P1x=-L0/2.0;
P1y=0;
P2x=+L0/2.0;
P2y=0;

//Get back arms cartesian position
P3x=(L1*cos(-th1))-(L0/2.0);
P3y=L1*sin(-th1);
P4x=(L2*cos(-th2))+(L0/2.0);
P4y=L2*sin(-th2);

//get D1 and D2 By diferences and Pythagorean
Dist1=sqrt(sq(P4x-P3x) + sq(P3y-P4y));
Dist2=sqrt(sq(P4x-P1x) + sq(P4y));
//Now plug D1 and D2 in cosines law for missing angles alpha and beta
alpha=asin((sq(L1)+sq(Dist1)-sq(Dist2))/(2.0*L1*Dist1));
beta=asin((sq(L3)+sq(Dist1)-sq(L4))/(2.0*L3*Dist1));

//Now get P5 by transform plugging all other variables abailable
P5x=P3x + (L3*cos(-th1-alpha-beta));
P5y=P3y + (L3*sin(-th1-alpha-beta));

P5x = round(P5x*10.0)/10.0;
P5y = round(P5y*10.0)/10.0;

Dist_Home = sqrt(sq(P5x-Home_X) + sq(P5y-Home_Y));
Dist_Target = sqrt(sq(P5x-Target_X) + sq(P5y-Target_Y));

Dist_OldTarget = sqrt(sq(P5x-OldTarget_X) + sq(P5y-OldTarget_Y));

/////////////////////////////////////////////////////////////////////////////////////////////

// save the joystick position at all times! Independent of TouchState
   PrintPosition (XPosition, P5x, YPosition, P5y, TimeMSec, currentMillis);
   
//TOUCH DETECTION
// case if no touch, touch end
 if (TouchSensor < Touch_threshold && TouchState == HIGH) {
   TouchState = LOW; // joystick not touched
   previousMillisTouchOff = currentMillis;
   PrintEvent(TouchEnd, currentMillis);
 }
 // case if touch start
 if (TouchSensor >= Touch_threshold && TouchState == LOW) { //start of touch, before there was no touch
   TouchState = HIGH; // joystick being touched
   PrintEvent(TouchStart, currentMillis);
}

//TOUCH DETECTION for Paw Rest
// case if no touch, touch end
// if (RestSensor < Rest_threshold && RestState == HIGH) {
//   RestState = LOW; // joystick not touched
//   PrintEvent(RestEnd, currentMillis);
// }
// // case if touch start
// else if (RestSensor >= Rest_threshold && RestState == LOW) { //start of touch, before there was no touch
//   RestState = HIGH; // joystick being touched
//   PrintEvent(RestStart, currentMillis);
//}



//LEAVING HOME DETECTION
// Print timestamp when entering and leaving home zone
if (HomeState == HIGH && Dist_Home > Home_r){
   HomeState = LOW;
   previousMillisOutofHome = currentMillis;
   PrintEvent(LeavHome, currentMillis);
}

if (HomeState == LOW && Dist_Home < Home_r){
   HomeState = HIGH;
   InactiveState = LOW;
   TargetState = LOW;
   OldTargetState = LOW;
   MotorDelay = LOW;
   previousMillisHome = currentMillis;
   PrintEvent(EntHome, currentMillis);
}


//// Sending digital signal out to Uno board, Pin HIGH for 'touch', Pin LOW for 'no touch'
//if (TouchState == HIGH){
//digitalWrite(STouchPin, HIGH);
//}
//else if (TouchState == LOW) {
//digitalWrite(STouchPin, LOW);
//}
//
//// Sending digital signal out to Uno board, Pin HIGH for 'at home', Pin LOW for 'out of home'
//if (HomeState == HIGH){
//digitalWrite(SHomePin, HIGH);
//}
//else if (HomeState == LOW) {
//digitalWrite(SHomePin, LOW);
//}

//////////////////////////////////////////// IF in Target circle, give Reward ///////////////////////////////////////////////////////////////////////////

// giving reward for crossing into target area - CIRCLE. Only if Motors OFF
  
     if (Dist_Target < Target_r && (SolState == LOW) && (TouchState == HIGH) && (TargetState == LOW) && (MotorState == LOW) && (RewDelay == LOW)){
       TargetState = HIGH;
       previousMillisRewDelay = currentMillis; //Start counting time since target hit, for reward delay
       RewDelay = HIGH;
       MotorDelay = HIGH;
       previousMillisMotorDelay = currentMillis; //start measuring time of target hit, for motor delay
       RewardCounter = RewardCounter + 1;
       
       tone(ClickPin,ToneFreq,ToneDur);
       //digitalWrite(SRewardPin, HIGH);
       PrintEvent(Hit, currentMillis);
     }


// Solenoid opening with Delay!
     if ((currentMillis - previousMillisRewDelay >= RewDelayTime) && (SolState == LOW) && (RewDelay == HIGH)){
       previousMillisSolenoid = currentMillis; //start counter for solenoid open
       SolState = HIGH; //solenoid opens
       RewDelay = LOW;
       PrintEvent(Solenoid, currentMillis);
     }


// Timestamp for entering previous target area
     if ((Dist_OldTarget < Target_r) && (OldTargetState == LOW) && (SolState == LOW) && (TouchState == HIGH) && (TargetState == LOW) && (MotorState == LOW)){
       OldTargetState = HIGH;
       PrintEvent(OldTarget, currentMillis);
     }


//////////////// Motors going home after OutofHomeTime has elapsed and no Reward occured ///////////////////////////

if ((currentMillis - previousMillisOutofHome >= OutofHomeTime) && (MotorState == LOW) && (InactiveState == LOW) && (HomeState == LOW) && (TouchState == HIGH) && (SolState == LOW) && (TargetState == LOW)){
       MotorState = HIGH;
       InactiveState = HIGH;
       
       enable(1,true); //when target reached, enabling motors
       enable(2,true);
       delay(8);
       myflushSerial12(); //Clear answers
  
      goToPosition(1,0); //motors moving to initial (home) position = 0
      goToPosition(2,0);
      delay(8);   
      myflushSerial12(); //Clear answers

      PrintEvent(MotorsON, currentMillis);
}


// give reward upon button press
if (SolState == LOW && ButtonState == LOW && ButtonPress == LOW){
       previousMillisRewDelay = currentMillis; //Start counting time since target hit, for reward delay
       RewDelay = HIGH;
       tone(ClickPin,ToneFreq,ToneDur);
       ButtonState = HIGH;
     //digitalWrite(SRewardPin, HIGH);
     PrintEvent(ManRew, currentMillis);
}

if (ButtonPress == HIGH){
  ButtonState = LOW;
}
 
// once the solenoid is open, time is measured until SolenoidTime --> solenoid closes
if (SolState == HIGH && (currentMillis - previousMillisSolenoid >= SolenoidTime)){
    SolState = LOW;
    //digitalWrite(SRewardPin, LOW);
}

digitalWrite(SolenoidPin, SolState); //checks in each loop what the Solenoid State is and writes it

//detecting licking
if (LickSensor < Lick_threshold) {  
  LickState = LOW;
  //digitalWrite(SLickPin, LOW);
}
else if (LickSensor >= Lick_threshold && LickState == LOW) { 
  LickState = HIGH;
  //digitalWrite(SLickPin, HIGH);
  PrintEvent(Lick, currentMillis);
}


// Joystick moves back to home position when animal is not touching the joystick
if ((currentMillis - previousMillisTouchOff >= TouchOffTime) && (TouchState == LOW) && (SolState == LOW) && (MotorState == LOW) && Dist_Home > Home_r) {
    MotorState = HIGH;
    enable(1,true); //when target reached, enabling motors
    enable(2,true);
    delay(8);
    myflushSerial12(); //Clear answers
  
    goToPosition(1,0); //motors moving to initial (home) position = 0
    goToPosition(2,0);
    delay(8);   
    myflushSerial12(); //Clear answers
    
    PrintEvent(MotorsTE, currentMillis);
}

//// Joystick moves back to home position when leaving home direction is a pull (Y > 66.0)
//if (P5y > 66  && (TouchState == HIGH) && (SolState == LOW) && (MotorState == LOW) && Dist_Home > Home_r) {
//    MotorState = HIGH;
//    enable(2,true); //when target reached, enabling motors
//    enable(1,true);
//    delay(8);
//    myflushSerial12(); //Clear answers
//
//    goToPosition(2,0); //motors moving to initial (home) position = 0
//    goToPosition(1,0);
//    delay(8);   
//    myflushSerial12(); //Clear answers
//
//    Serial.print("Motors_on, ");
//    Serial.println(currentMillis);
//}


// Joystick moves back to home position after target reach, AFTER DELAY OF 'MotorDelayTime'
if ((currentMillis - previousMillisMotorDelay >= MotorDelayTime) && (MotorDelay == HIGH) && (SolState == LOW) && (MotorState == LOW) && Dist_Home > Home_r) {
    MotorState = HIGH;
    MotorDelay = LOW;
    enable(1,true); //when target reached, enabling motors
    enable(2,true);
    delay(8);
    myflushSerial12(); //Clear answers
 
    goToPosition(1,0); //motors moving to initial (home) position = 0
    goToPosition(2,0);
    delay(8);   
    myflushSerial12(); //Clear answers

     PrintEvent(MotorsON, currentMillis);
}

// to prevent the joystick from getting blocked, motor disengages if Y postion negative.
if (MotorState == LOW && (P5x < -28.0|| P5x > 28.0) && SolState == LOW) {
    MotorState = HIGH;
    enable(1,true); 
    enable(2,true);
    delay(8);
    myflushSerial12(); //Clear answers

    goToPosition(1,0);
    goToPosition(2,0);
    delay(8);   
    myflushSerial12(); //Clear answers

    PrintEvent(MotorsON, currentMillis);
}


// Scara: at 0/0 position, disable motors again.

// joystick needs to go back to home position before new target cross will be rewarded. But some wiggle room allowed = 2 x 2 mm area within which joystick can end up as 'home'
if (Dist_Home < Home_r && MotorState == HIGH && SolState == LOW && (currentMillis - previousMillisHome >= StayHomeTime)) {
   TargetState = LOW;
   MotorState = LOW;
   
   enable(1,false);
   enable(2,false);
   delay(8);
   myflushSerial12(); //Clear answers
   
   PrintEvent(MotorsOFF, currentMillis);
}


if ((RewardCounter >= MaxRewards) && SolState == LOW && HomeState == HIGH){
  Serial.print("Maximal numbers of rewards reached - end of session");
  delay(1000);  
  exit(0);
}
}

/////////////////////// Function for printing serial text //////////////////////////////////////////////////////
void PrintEvent (const char *Event, unsigned long Time){
     Serial.print(Event);
     Serial.print(", ");
     Serial.println(Time);  
}

void PrintPosition (const char *PositionX, double X, const char *PositionY, double Y, const char *TimeMS, unsigned long Time){
     Serial.print(PositionX);
     Serial.print(", ");
     Serial.print(X);
     Serial.print(", ");
     Serial.print(PositionY);
     Serial.print(", ");
     Serial.print(Y);
     Serial.print(", ");
     Serial.print(TimeMS);
     Serial.print(", ");
     Serial.println(Time);  
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**********************************************************************************
 *                           Functions to Control Maxon EPOS4                     *
 **********************************************************************************
 * void enable(char node, bool st)                                                *
 * Enable/Disable (or engage/disengage) the EPOS4 motor.                          *
 * The function parameters are the EPOS board node as a 8 bit char number,        *
 * and st  (status)  either true (Enable node) or false (Disable Node)            *
 * ********************************************************************************
 * void goToPosition(char node, int pos)                                          *
 * Move the motor to specific encoder position, supported -32768 +32767 (16 bit)  *
 * but for our current encode encoder limited to -2047 to +2048.                  *
 * The function parameters are the EPOS board node as a 8 bit char number,        *
 * and pos (position) as a signed integer.                                        *
 * ********************************************************************************
 * void sHomePos(char node)                                                       *
 * Set Home Position. It resets the current encoder 0 or Home position            * 
 * to the current position.                                                       *
 * This function requires the EPOS board node parameter as an 8-bit char number   *
 * ********************************************************************************
 * int cPosition(char node)                                                       *
 * Current position returns the current position where the motor encoder          *
 * is relative to the home position.                                              *
 * This function requires the EPOS board node parameter as an 8-bit char number   *
 * and it returns a signed integer value with the current position in a 16-bit    *
 * margin -32768 +32767 (16 bit).  Note that is possible to go over our current   *
 * -2047 to +2048 limits if the shaft encoder is rotated further                  *
 * than those limits that correspond with +/-180 degrees.                         *
 **********************************************************************************/

// Enable/Disable sequence
void enable(char node, bool st){
    word txStr[6];
    if(st){  //ENABLE
      txStr[0]= 0x0468;  //Read, Len 2
      txStr[1]=0x4001; //IdxLSB & Node  
      txStr[2]=0x0060; //subIdx & IdxMSB  
      txStr[3]=0x0006; //Usr Value
      txStr[4]=0x0000; //Usr Value
      txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
      if (node==1)
        SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
      else if (node==2)
        SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing
      delay(5);
      txStr[0]= 0x0468;  //Read, Len 2
      txStr[1]=0x4001; //IdxLSB & Node 
      txStr[2]=0x0060; //subIdx & IdxMSB  
      txStr[3]=0x000F; //Usr Value
      txStr[4]=0x0000; //Usr Value
      txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
      if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
      else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
    //  delay(10); // was 50 ----move to original call--Rick
    //  myflushSerial12(); //Clear answers
    }
    else{   //DISABLE
      txStr[0]= 0x0468;  //Read, Len 2
      txStr[1]=0x4001; //IdxLSB & Node
      txStr[2]=0x0060; //subIdx & IdxMSB  
      txStr[3]=0x0000; //Usr Value
      txStr[4]=0x0000; //Usr Value
      txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
      if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
      else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing 
   //   delay(10); // was 50
   //   myflushSerial12(); //Clear answers
    }
}

void goToPosition(char node, int pos){
    word txStr[6];
    txStr[0]=0x0468;  //Read, Len 2
    txStr[1]=0x6001; //IdxLSB & Node   
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=0x0001; //Usr Value     SET Position Mode 
    txStr[4]=0x0000; //Usr Value
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
    delay(5); 
    txStr[0]= 0x0468;  //Read, Len 2
    txStr[1]=0x7A01; //IdxLSB & Node  
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=(word)pos; //Usr Value Pos
    if (pos<0) //Position negative/positive (We only support 16 bit, 65536 positons, Can/Maxon protocol 32bit )
      txStr[4]=0xFFFF; //Usr Value MSW Padding
    else
      txStr[4]=0x0000; //Usr Value MSW padding
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
    delay(5);
    txStr[0]= 0x0468;  //Read, Len 2
    txStr[1]=0x4001; //IdxLSB & Node  
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=0x003F; //Usr Value  (Go to abs position inmediatly)
    txStr[4]=0x0000; //Usr Value
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
    delay(5);
    txStr[0]= 0x0468;  //Read, Len 2
    txStr[1]=0x4001; //IdxLSB & Node  
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=0x000F; //Usr Value  (Toggle new position)
    txStr[4]=0x0000; //Usr Value
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
   // delay(10); // was 50 
   // myflushSerial12(); //Clear answers
}

void sHomePos(char node){
    word txStr[6];
    txStr[0]=0x0468;  //Read, Len 2
    txStr[1]=0x6001; //IdxLSB & Node  
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=0x0006; //Usr Value
    txStr[4]=0x0000; //Usr Value
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
    delay(5);   
    txStr[0]=0x0468;  //Read, Len 2
    txStr[1]=0x9801; //IdxLSB & Node  
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=0x0025; //Usr Value
    txStr[4]=0x0000; //Usr Value
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
    delay(5);
    txStr[0]=0x0468;  //Read, Len 2
    txStr[1]=0x4001; //IdxLSB & Node  
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=0x000F; //Usr Value
    txStr[4]=0x0000; //Usr Value
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
    delay(5);
    txStr[0]=0x0468;  //Read, Len 2
    txStr[1]=0x4001; //IdxLSB & Node  
    txStr[2]=0x0060; //subIdx & IdxMSB  
    txStr[3]=0x001F; //Usr Value
    txStr[4]=0x0000; //Usr Value
    txStr[5]=strCRC(txStr,5); //Calculate CRC-CCIT for command string
    if (node==1)
         SerialEPOSWrite1((byte*)txStr, 12); //Especial TX with header and Stuffing
    else if (node==2)
         SerialEPOSWrite2((byte*)txStr, 12); //Especial TX with header and Stuffing   
  //  delay(10); // was 50
  //  myflushSerial12(); //Clear answers
}

//Get Current Position
int cPosition(char node){
  char rxIdx=0, tOut=255; //Time out default to 256ms
  word txStr[4];
  byte rxZ=0, rxZ1, rxZ2;//Current and last RX byte recieved // added rxZ2 here
  int pos=0x7FFF; //Initial value out of range
  txStr[0]= 0x0260;  //Read, Len 2
  txStr[1]=0x6401; //IdxLSB & Node  
  txStr[2]=0x0060; //subIdx & IdxMSB  
  txStr[3]=strCRC(txStr,3); //Calculate CRC-CCIT for command string
  if (node==1){
      SerialEPOSWrite1((byte*)txStr, 8); //Especial TX with header and Stuffing
      while (!Serial1.available() && tOut){// Wait until data. Experimentally, with port at 115K response available takes under 5mS
        tOut--;
        delay(1);
      }
      delay(1); //Let extra mS to let the available frame be recieved to the point we care, at 115.2Kbps takes around 87uS by byte
      while (Serial1.available()) {
        rxZ1=rxZ; // Hold last byte
        rxZ=Serial1.read(); //Current Byte
        // read all the incoming answer
        switch(rxIdx){
        case 8:  
          pos = rxZ; 
          break;
        case 9:
          if (rxZ==0x90 && rxZ1==0x90) //Stuffing found
            rxIdx--; //Hold index and ignore
          else
            pos += (rxZ<<8);
          break;
        }    
        rxIdx++;
        if (!rxIdx)
            rxIdx=0; //Hold index if buffer has unexpected data or garbage (over 256 chars, hits 0 again) until empty  
    }
  }
  else if (node==2){
      SerialEPOSWrite2((byte*)txStr, 8); //Especial TX with header and Stuffing
      while (!Serial2.available() && tOut){// Wait until data. Experimentally, with port at 115K response available takes under 5mS
        tOut--;
        delay(1);
      }
      delay(1); //Let extra mS to let the available frame be recieved to the point we care, at 115.2Kbps takes around 87uS by byte
      while (Serial2.available()) {
        rxZ1=rxZ; // Hold last byte
        rxZ=Serial2.read(); //Current Byte
        // read all the incoming answer
        switch(rxIdx){
        case 8:  
          pos = rxZ; 
          break;
        case 9:
          if (rxZ==0x90 && rxZ1==0x90) //Stuffing found
            rxIdx--; //Hold index and ignore
          else
            pos += (rxZ<<8);
          break;
        }    
        rxIdx++;
        if (!rxIdx)
            rxIdx=0; //Hold index if buffer has unexpected data or garbage (over 256 chars, hits 0 again) until empty  
    }
  }
  return pos; //a 7FFF is timed out with no data 
}

//CRC calculator for Maxon controller
word strCRC(word *trxStr,  char sLen){
  word shifter, c;
  word carry;
  word CRC = 0; //CRC with Initial 0 
  sLen++;  //Needed to xor last value in CRC calc.
  while(sLen--) {
     shifter = 0x8000;          //Initialize BitX to Bit15
     if (sLen)
        c = *trxStr++;               //Copy next 16 bit Word to c
     else
        c = 0x0000; //Stuff last value 0
     do {
        carry = CRC & 0x8000;    //Check carry Bit15 of CRC is set
        CRC <<= 1;               //CRC = CRC * 2
        if(c & shifter) CRC++;   //CRC = CRC + 1, if BitX is set in c
        if(carry) CRC ^= 0x1021; //Xor CRC (CCITT X^16 + X^12 +X^5 + 1) 
        shifter >>= 1;          //Set BitX to next lower Bit, shifter = shifter/2
     } while(shifter);
  }
  return CRC;
}

//Clear Serial Buffer needed when Control responds data that is not used.
void myflushSerial12(){
  while(Serial1.available()) Serial1.read();
  while(Serial2.available()) Serial2.read();
}

//Sending data to EPOS  and stuffing special DLE 
void SerialEPOSWrite1(byte* txStr, byte len){ 
  byte idx=0;
  word hdr=HDR;
  //Send header first
  Serial1.write((byte*)&hdr,2);
  while(idx<len){
      Serial1.write(txStr[idx]);
      if (txStr[idx]==0x90){ //DLE found out of header
        Serial1.write(txStr[idx]); //Insert another one
      }
      idx++;
  }
}
 //Sending data to EPOS  and stuffing special DLE 
void SerialEPOSWrite2(byte* txStr, byte len){ 
  byte idx=0;
  word hdr=HDR;
  //Send header first
  Serial2.write((byte*)&hdr,2);
  while(idx<len){
      Serial2.write(txStr[idx]);
      if (txStr[idx]==0x90){ //DLE found out of header
        Serial2.write(txStr[idx]); //Insert another one
      }
      idx++;
  }
}
