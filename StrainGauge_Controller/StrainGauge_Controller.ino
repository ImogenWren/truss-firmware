//Controller Arduino for Strain Gauge measurements

//Communicates through I2C with PERIPHERAL Arduino to receive strain gauge measurements.
#include "TrussStepper.h"
#include "ArduinoJson-v6.9.1.h"
#include <Wire.h>

//JSON serialization
#define COMMAND_SIZE 64  //originally 64
StaticJsonDocument<COMMAND_SIZE> doc;
char command[COMMAND_SIZE];

//STEPPER VARIABLES
#define SDIR 2
#define SPUL 3
const int stepperStepsPerRev = 200;
const int stepperStepPeriod = 1000; //microseconds
TrussStepper stepper = TrussStepper(stepperStepsPerRev, SDIR, SPUL);
int currentPos = 0;     //the position of the stepper in terms of number of steps
int moveToPos = 0;      //the position the stepper should move to in terms of steps.
const int positionLimit = 4*stepperStepsPerRev;
const int direction = -1;   //reverse the direction of the steps

//LIMIT SWITCHES
bool limitSwitchesAttached = false;
#define limitSwitchLower 11
bool lowerLimitReached = false;
#define limitSwitchUpper 9
bool upperLimitReached = false;
volatile int upperSteps = 0;
volatile int lowerSteps = 0;

//TIMING FOR GAUGE READING
unsigned long timeInterval = 5000;    //request gauge readings on this time interval
unsigned long currentTime = millis();

//GAUGE VARIABLES
const int PERIPHERAL_ADDRESS = 8;
const int numGauges = 7;
int next_index = 0;       //the index of the next gauge to read data from

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION;

FLOATUNION data_0;
FLOATUNION data_1;
FLOATUNION data_2;
FLOATUNION data_3;
FLOATUNION data_4;
FLOATUNION data_5;
FLOATUNION data_6;

FLOATUNION data[numGauges] = {data_0, data_1, data_2, data_3, data_4, data_5, data_6};

/**
 * Defines the valid states for the state machine
 * 
 */
typedef enum
{
  STATE_STANDBY,        //no drive to motor, no reading of gauges
  STATE_READ,           //requests, reads data from peripheral and writes data to serial
  STATE_MOVE,           //allows stepper motor to move to new position
  STATE_ZERO,           //zeroes the position of the servo
  STATE_TARE,           //tares (zeroes) the gauge readings
  
} StateType;

//state Machine function prototypes
//these are the functions that run whilst in each respective state.
void Sm_State_Standby(void);
void Sm_State_Read(void);
void Sm_State_Move(void);
void Sm_State_Zero(void);
void Sm_State_Tare(void);

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
  {STATE_TARE, Sm_State_Tare},
};
 
int NUM_STATES = 5;

/**
 * Stores the current state of the state machine
 */
 
StateType SmState = STATE_STANDBY;    //START IN THE STANDBY STATE


//DEFINE STATE MACHINE FUNCTIONS================================================================

//TRANSITION: STATE_STANDBY -> STATE_STANDBY
void Sm_State_Standby(void){

//  if(limitSwitchesAttached)
//  {
//    detachInterrupt(digitalPinToInterrupt(limitSwitchLower));
//    detachInterrupt(digitalPinToInterrupt(limitSwitchUpper));
//
//    limitSwitchesAttached = false;
//  }
  
  SmState = STATE_STANDBY;
}

//TRANSITION: STATE_READ -> STATE_READ
void Sm_State_Read(void){

  if(millis() >= currentTime + timeInterval)
  {

    if(Wire.requestFrom(PERIPHERAL_ADDRESS, 4))   //request 4 bytes of data from each gauge (returning a float value) from peripheral address PERIPHERAL_ADDRESS
    {     
      
      for(byte i=0; i<4; i++)
      {
          data[next_index].bytes[i] = Wire.read();
      }
      
      next_index = (next_index + 1) % numGauges;
        
    } else 
    {
      Serial.print("{\"error\":\"gauge\":");
      Serial.print(next_index);
      Serial.println("}");
    }


    printToSerial();
    currentTime = millis();

  }
  
  SmState = STATE_READ;
}

//TRANSITION: STATE_MOVE -> STATE_READ
//Remains in move state until current position matches moveTo position.
//This blocks gauge reading, but high stepper speed and slow update of gauges should make this fine.
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
  
  if(moveToPos != currentPos)
  {
    if(currentPos > moveToPos)
    {
      //step clockwise with stepper class
      stepper.step(-1*direction);    //might want to put a direction offset in the library
      currentPos -= 1;
    } 
    else if(currentPos < moveToPos)
    {
      //step anticlockwise with stepper class
      stepper.step(1*direction);
      currentPos += 1;  
    }

    SmState = STATE_MOVE;
    
  }
  else
  {
    //current position has reached the requested moveTo position so can go back to reading the gauges.
    SmState = STATE_READ;
  }
  
}

//TRANSITION: STATE_ZERO -> STATE_READ
void Sm_State_Zero(void){
  //Should move until limit switch interrupt hits which defines a max/min point - then move half the distance between limit switches.
  //Do I have the number of steps between limit switches hardcoded,
  //Or do I calculate that by moving between the limits each time we zero?
  
  if(lowerLimitReached && upperLimitReached)
  {
    stepper.step((upperSteps - lowerSteps)*direction/2);
    currentPos = 0;
    lowerLimitReached = false;
    upperLimitReached = false;
    SmState = STATE_READ;
  }
  else if(upperLimitReached)
  {
    stepper.step(-1*direction);
    currentPos -= 1;   
    SmState = STATE_ZERO; 
  }
  else
  {
    stepper.step(1*direction);
    currentPos += 1;
    SmState = STATE_ZERO;
  }

}

//TRANSITION: STATE_TARE -> STATE_READ
void Sm_State_Tare(void){
  
  Wire.beginTransmission(PERIPHERAL_ADDRESS);
  Wire.write('t');
  Wire.endTransmission();
  delay(1000);
  
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
  
  //I2C communication with peripheral arduino
  Wire.begin();

  //Serial communication for sending data -> RPi -> Server
  Serial.begin(57600);
  while(!Serial);

  stepper.setDelay(stepperStepPeriod);

  //on startup make sure peripheral device has set the gauge index to 0.
  Wire.beginTransmission(PERIPHERAL_ADDRESS);
  Wire.write('0');
  Wire.endTransmission();
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

    if(strcmp(set, "speed")==0)
    {
      float new_speed = doc["to"];
    } 
    else if(strcmp(set, "position")==0)
    {
  
        float new_position = doc["to"];
        
        if(new_position >= -positionLimit && new_position <= positionLimit)
        {
          moveToPos = new_position;
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
        } 
        else if(strcmp(new_mode, "read") == 0)
        {
          SmState = STATE_READ;
        }
        else if(strcmp(new_mode, "move") == 0)
        {
          SmState = STATE_MOVE;
        }
        else if(strcmp(new_mode, "zero") == 0)
        {
          SmState = STATE_ZERO;
        }
        else if(strcmp(new_mode, "tare") == 0)
        {
          SmState = STATE_TARE;
        }
        
    }  
    
  }
      return SmState;     //return whatever state it changed to or maintain the state.
 } 

//On an interrupt - will interrupt all state functions
void doLimitLower(void){
  if(!lowerLimitReached)
  {
    //if the lower limit is reached, then the stepper should move a small distance back towards the centre, away from the limit
    lowerLimitReached = true;
    lowerSteps = currentPos;
    
    if(SmState == STATE_ZERO)
    {
       SmState = STATE_ZERO;
    }
    else 
    {
      //TEMP OUTPUT OF DATA
      Serial.print("Lower limit at pos: ");
      Serial.println(currentPos);
      moveToPos = currentPos + stepperStepsPerRev;
      SmState = STATE_MOVE;  
    }
  }
  
    

}

//On an interrupt - will interrupt all state functions
void doLimitUpper(void){
  if(!upperLimitReached)
  {
    upperLimitReached = true;
    upperSteps = currentPos;

    if(SmState == STATE_ZERO)
    {
        SmState = STATE_ZERO;
    } 
    else 
    {
      //TEMP OUTPUT OF DATA
      Serial.print("Upper limit at pos: ");
      Serial.println(currentPos);
      moveToPos = currentPos - stepperStepsPerRev;
      SmState = STATE_MOVE;  
    }
  }
    
    

}

void printToSerial(){
  Serial.print("{\"load_cell\":");
  Serial.print(data[0].number);
  
  for(int i=1;i<numGauges;i++){
    Serial.print(",\"gauge_");
    Serial.print(i);
    Serial.print("\":");
    Serial.print(data[i].number);
  }
  
  Serial.println("}");
 
}
