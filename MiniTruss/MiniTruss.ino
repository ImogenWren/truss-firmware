//Firmware for MiniTruss remote lab experiment
// Single controller firmware with state machine for reading and writing.

//Author: David P. Reid
//dprydereid@gmail.com
// 17/03/22

// Update for new LinearServo load control
// 02/05/22
// dprydereid@gmail.com

// IMPORT LIBRARIES
#include "HX711.h"
#include "LinearServo.h"
#include "ArduinoJson-v6.9.1.h"

//JSON serialization
#define COMMAND_SIZE 64  //originally 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

//STEPPER VARIABLES
#define DRIVE 12
//#define SDIR 15
//#define SPUL 16

LinearServo servo = LinearServo(DRIVE);
int currentPos = 0;     //the position of the servo as output from LinearServo library
int moveToPos = 0;      //the position the servo should move to between 0 (full retraction) to 100 (full extension)
unsigned long move_interval = 5000L;    //this is updated depending on the move distance
unsigned long step_interval = 100L;     //at 5V takes approx. 100ms to move 1 position
unsigned long enter_move_time = millis();    //the time at which the move state started

//LIMIT SWITCHES
bool limitSwitchesAttached = false;
#define limitSwitchLower 11
bool lowerLimitReached = false;
#define limitSwitchUpper 13
bool upperLimitReached = false;

//GAUGE READINGS
const int numGauges = 7;
float data[numGauges] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//GAUGE SETUP
const int SCK_PIN = 2;  //Common CLOCK pin
const int GAUGE_0_DT = 3; //DATA pins
const int GAUGE_1_DT = 4;
const int GAUGE_2_DT = 5;
const int GAUGE_3_DT = 6;
const int GAUGE_4_DT = 7;
const int GAUGE_5_DT = 8;
const int GAUGE_6_DT = 9;

const int pins[numGauges] = {GAUGE_0_DT, GAUGE_1_DT, GAUGE_2_DT, GAUGE_3_DT, GAUGE_4_DT, GAUGE_5_DT, GAUGE_6_DT};

HX711 gauge_0;
HX711 gauge_1;
HX711 gauge_2;
HX711 gauge_3;
HX711 gauge_4;
HX711 gauge_5;
HX711 gauge_6;

//these will need to be calibrated for new truss setup
// OLD values for large truss
//const int scale_load = -15184;
//const int scale_factor_1 = 4050;
//const int scale_factor_2 = scale_factor_1*1.030;
//const int scale_factor_3 = scale_factor_1*0.977;
//const int scale_factor_4 = scale_factor_1*0.947;
//const int scale_factor_5 = scale_factor_1*0.818;
//const int scale_factor_6 = scale_factor_1*0.924;

const int scale_load = 15184;
const int scale_factor_1 = 3284;
const int scale_factor_2 = scale_factor_1;
const int scale_factor_3 = scale_factor_1;
const int scale_factor_4 = scale_factor_1;
const int scale_factor_5 = scale_factor_1;
const int scale_factor_6 = scale_factor_1;


HX711 gauges[numGauges] = {gauge_0, gauge_1, gauge_2, gauge_3, gauge_4, gauge_5, gauge_6};

//TIMING FOR GAUGE WRITING
unsigned long timeInterval = 1000;    //write out gauge readings with a period no smaller than 1s
unsigned long previousTime = 0;

/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_STANDBY = 0,        //no drive to motor, no reading of gauges
  STATE_READ = 1,           //reads each gauge
  STATE_MOVE = 2,           //allows stepper motor to move to new position
  STATE_ZERO = 3,           //zeroes the position of the servo
  STATE_TARE_GAUGES = 4,           //tares (zeroes) the gauge readings
  STATE_TARE_LOAD = 5,      //tares the load force gauge
  STATE_TARE_ALL = 6,      //tares both the gauges and load cell
  STATE_GAUGE_RESET = 7,    //resets all gauges
  
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Standby(void);
void Sm_State_Read(void);
void Sm_State_Move(void);
void Sm_State_Zero(void);
void Sm_State_Tare_Gauges(void);
void Sm_State_Tare_Load(void);
void Sm_State_Tare_All(void);
void Sm_State_Gauge_Reset(void);

/**
 * Type definition used to define the state
 */
 
typedef struct
{
  StateType State; /**< Defines the command */
  void (*func)(void); /**< Defines the function to run */
} StateMachineType;

/**
 * A table that defines the valid states of the state machine and
 * the function that should be executed for each state
 */
StateMachineType StateMachine[] =
{
  {STATE_STANDBY, Sm_State_Standby},
  {STATE_READ, Sm_State_Read},
  {STATE_MOVE, Sm_State_Move},
  {STATE_ZERO, Sm_State_Zero},
  {STATE_TARE_GAUGES, Sm_State_Tare_Gauges},
  {STATE_TARE_LOAD, Sm_State_Tare_Load},
  {STATE_TARE_ALL, Sm_State_Tare_All},
  {STATE_GAUGE_RESET, Sm_State_Gauge_Reset},
};
 
int NUM_STATES = 8;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_READ;    //START IN THE READ STATE

//DEFINE STATE MACHINE FUNCTIONS================================================================

//TRANSITION: STATE_STANDBY -> STATE_STANDBY
void Sm_State_Standby(void){

  //is there a need to detach these interrupts? Best to just attach and keep attached?
  if(limitSwitchesAttached)
  {
    detachInterrupt(digitalPinToInterrupt(limitSwitchLower));
    detachInterrupt(digitalPinToInterrupt(limitSwitchUpper));

    limitSwitchesAttached = false;
  }

  
  SmState = STATE_STANDBY;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ READ ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_READ -> STATE_READ
// State Read loops through all the gauges, reading their values and storing for writing.
// Remains in read state until user makes the change.
void Sm_State_Read(void){

  upperLimitReached = false;    //if in read state then clear the limit flags.    =====NEW
  lowerLimitReached = false;

  if(millis() - previousTime >= timeInterval){
  
   for(int i=0; i< numGauges; i++){
      //if(gaugeScales[i].wait_ready_timeout(100)){
      if(gauges[i].is_ready()){
        
        data[i] = gauges[i].get_units(5);       //what is the best number of readings to take?
        
      } 

      delay(10);    //necessary?
   }

    report();
    previousTime = millis();
    
  }
  
   SmState = STATE_READ;
  
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ MOVE ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_MOVE -> STATE_READ
//Remains in move state for an appropriate time to complete the move. There is no feedback on position from servo so need to base on time.
//This blocks gauge reading, but high servo speed and slow update of gauges should make this fine.
void Sm_State_Move(void){
  
  if(!limitSwitchesAttached)
  {
    attachInterrupt(digitalPinToInterrupt(limitSwitchLower), doLimitLower, FALLING);
    attachInterrupt(digitalPinToInterrupt(limitSwitchUpper), doLimitUpper, FALLING);

    limitSwitchesAttached = true;
  }

  if(lowerLimitReached || upperLimitReached)
  {
    lowerLimitReached = false;
    upperLimitReached = false;
  }
  
  currentPos = servo.update();
   
   if(millis() - enter_move_time >= move_interval){
    
      SmState = STATE_READ;
    
   } 
   else
   {
    
      SmState = STATE_MOVE;
    
   }
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ ZERO ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_ZERO -> STATE_READ
//
void Sm_State_Zero(void){

  if(!limitSwitchesAttached)
  {
    attachInterrupt(digitalPinToInterrupt(limitSwitchLower), doLimitLower, FALLING);
    attachInterrupt(digitalPinToInterrupt(limitSwitchUpper), doLimitUpper, FALLING);

    limitSwitchesAttached = true;
  }
  
  servo.zero();
  
  SmState = STATE_READ;
 
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ TARE GAUGES++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_TARE_GAUGES -> STATE_READ
void Sm_State_Tare_Gauges(void){
  
  tareGauges();
  delay(100);
  
  SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ TARE LOAD ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_TARE_LOAD -> STATE_READ
void Sm_State_Tare_Load(void){
  
  tareLoad();
  delay(100);
  
  SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ TARE ALL ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_TARE_ALL -> STATE_READ
void Sm_State_Tare_All(void){
  
  tareAll();
  delay(100);
  
  SmState = STATE_READ;
  
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ GAUGE RESET ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//TRANSITION: STATE_GAUGE_RESET -> STATE_READ
void Sm_State_Gauge_Reset(void){

  resetGauges();
  delay(100);
  
  SmState = STATE_READ;
  
}

//STATE MACHINE RUN FUNCTION
void Sm_Run(void)
{
  if (SmState < NUM_STATES)
  {
    SmState = readSerialJSON(SmState);      
    (*StateMachine[SmState].func)();        //reads the current state and then runs the associated function
    
  }
  else{
    Serial.println("Exception in State Machine");
  }
  
}



void setup() {

  pinMode(limitSwitchLower, INPUT_PULLUP);
  pinMode(limitSwitchUpper, INPUT_PULLUP);

  previousTime = millis();

  //Serial communication for sending data -> RPi -> Server
  Serial.begin(57600);
  while(!Serial);


  resetGauges();

}

void loop() {

  Sm_Run();

}


StateType readSerialJSON(StateType SmState){
  if(Serial.available() > 0)
  {

    Serial.readBytesUntil(10, command, COMMAND_SIZE);
    deserializeJson(doc, command);
    
    const char* set = doc["set"];

    if(strcmp(set, "position")==0)
    {
  
        float new_position = doc["to"];
        
        if(new_position >= 0 && new_position <= 100)
        {
          moveToPos = new_position;
          servo.updateMoveTo(moveToPos);
          move_interval = abs(moveToPos - currentPos)*step_interval;
        } 
        else
        {
          Serial.println("Outside position range");
        }
     
  } 
    else if(strcmp(set, "mode")==0)
    {
      
      const char* new_mode = doc["to"];

        if(strcmp(new_mode, "standby") == 0)
        {
          SmState = STATE_STANDBY;
          reportState(STATE_STANDBY);//necessary?
        } 
        else if(strcmp(new_mode, "move") == 0)
        {
          SmState = STATE_MOVE;
          enter_move_time = millis();
          reportState(STATE_MOVE);//necessary?
        }
        else if(strcmp(new_mode, "zero") == 0)
        {
          SmState = STATE_ZERO;
          reportState(STATE_ZERO);//necessary?
        }
        else if(strcmp(new_mode, "tare") == 0)
        {
          SmState = STATE_TARE_GAUGES;
          reportState(STATE_TARE_GAUGES);//necessary?
        }
        else if(strcmp(new_mode, "tare_load") == 0)
        {
          SmState = STATE_TARE_LOAD;
          reportState(STATE_TARE_LOAD);   //necessary?
        }
        else if(strcmp(new_mode, "tare_all") == 0)
        {
          SmState = STATE_TARE_ALL;
          reportState(STATE_TARE_ALL);   //necessary?
        }
        else if(strcmp(new_mode, "gauge_reset") == 0)
        {
          SmState = STATE_GAUGE_RESET;
          reportState(STATE_GAUGE_RESET);//necessary?
        }
        
    }  
    
  }
      return SmState;     //return whatever state it changed to or maintain the state.
 } 

 //On an interrupt - will interrupt all state functions
//TRANSITION: -> READ
void doLimitLower(void){
  if(!lowerLimitReached)
  {
//    //if the lower limit is reached, then the stepper should move a small distance back towards the centre, away from the limit
//    lowerLimitReached = true;
//    
//    //TEMP OUTPUT OF DATA
//    //Serial.print("Lower limit at pos: ");
//    //Serial.println(currentPos);
//      
//    stepper.step(-15*stepperStepsPerRev*direction);
//    moveToPos = currentPos;
    SmState = STATE_READ; 
  }
}

//On an interrupt - will interrupt all state functions
//TRANSITION: -> READ
void doLimitUpper(void){
  if(!upperLimitReached)
  {
//    upperLimitReached = true;
//
//    //TEMP OUTPUT OF DATA
//    //Serial.print("Upper limit at pos: ");
//    //Serial.println(currentPos);
//
//    stepper.step(15*stepperStepsPerRev*direction);
//    currentPos = 0;
//    moveToPos = 0;
    SmState = STATE_READ;  
  }
}

void report(){
  Serial.print("{\"load_cell\":");
  Serial.print(data[0]);
  
  for(int i=1;i<numGauges;i++){
    Serial.print(",\"gauge_");
    Serial.print(i);
    Serial.print("\":");
    Serial.print(data[i]);
  }

  Serial.print(",\"state\":");
  Serial.print(SmState);
  Serial.print(",\"pos\":");
  Serial.print(currentPos);
  
  Serial.println("}");
 
}

//DO I NEED this function when state is being reported in report()?
void reportState(int state){
  Serial.print("{\"state\":");
  Serial.print(state);
  Serial.println("}");
}

void initialiseGauges(){
  for(int i=0;i<numGauges;i++){
    gauges[i].begin(pins[i], SCK_PIN);
  }
}

void setGain(int gain){
  for(int i=0;i<numGauges;i++){
    gauges[i].set_gain(gain);
  }
}

//Just tares the load cell
void tareLoad(){
  data[0] = 0.0;
  gauges[0].tare();
}

//tares all gauge scales, but not load cell
void tareGauges(){
  for(int i=1;i<numGauges;i++){
    data[i] = 0.0;       //set the stored data value to 0
    gauges[i].tare();
   }
}

//tares all gauges, including load cell
void tareAll(){
  for(int i=0;i<numGauges;i++){
    data[i] = 0.0;       //set the stored data value to 0
    gauges[i].tare();
   }
}

void resetGauges(){
  initialiseGauges();
  setGain(128);
 
  gauges[0].set_scale(scale_load);   //calibrated with the load cell on the real truss -> OUTPUTS force in newtons
  gauges[1].set_scale(scale_factor_1);          //member 1, calibrated with truss member 1  -> outputs strain in micro-strain
  gauges[2].set_scale(scale_factor_2);          //member 2
  gauges[3].set_scale(scale_factor_3);          //member 3
  gauges[4].set_scale(scale_factor_4);          //member 4
  gauges[5].set_scale(scale_factor_5);          //member 5
  gauges[6].set_scale(scale_factor_6);          //member 6

  
  tareAll();

}
