//v0.1
//Controller Arduino for Strain Gauge measurements

//Communicates through I2C with PERIPHERAL Arduino to receive strain gauge measurements.
//Tare button
//Will also control a servo motor.

#include <Wire.h>

#define button 13

unsigned long timeInterval = 2000;
unsigned long currentTime = millis();

const int numGauges = 2;

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION;

FLOATUNION data_1;
FLOATUNION data_2;

FLOATUNION data[numGauges] = {data_1, data_2};


void setup() {
  
  pinMode(button, INPUT_PULLDOWN);

  //I2C communication with peripheral
  Wire.begin();

  Serial.begin(57600);
  while(!Serial);

 }

void loop() {
  if(millis() >= currentTime + timeInterval){

    Wire.requestFrom(8, numGauges*4);   //request 4 bytes of data from each gauge (returning a float value) from peripheral address 8

    int i = 0;
    int j = 0;
    while(Wire.available()){
      data[i].bytes[j] = Wire.read();
      j = j+1;

      if(j == 4){
        i = i + 1;
        j = 0;
      }
 
    }
    
    printDataToSerial();
    
    if(digitalRead(button) == HIGH){
      Serial.println("TARE");
      Wire.beginTransmission(8);
      Wire.write('t');
      Wire.endTransmission();
    }
    
    currentTime = millis();
  }

}

void printDataToSerial(){
  Serial.print("{\"gauge_1\":");
  Serial.print(data[0].number, 0);
  Serial.print(",\"gauge_2\":");
  Serial.print(data[1].number, 0);
  Serial.println("}");
 

  
}
