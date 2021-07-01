//v0.1
//Controller Arduino for Strain Gauge measurements

//Communicates through I2C with PERIPHERAL Arduino to receive strain gauge measurements.
//Tare button
//Will also control a servo motor.

#include <Wire.h>

#define button 13

unsigned long timeInterval = 1000;
unsigned long currentTime = millis();

const int numGauges = 3;

typedef union
{
  float number;
  uint8_t bytes[4];
} FLOATUNION;

FLOATUNION data_0;
FLOATUNION data_1;
FLOATUNION data_2;
//FLOATUNION data_3;
//FLOATUNION data_4;
//FLOATUNION data_5;
//FLOATUNION data_6;

//FLOATUNION data[numGauges] = {data_0, data_1, data_2, data_3, data_4, data_5, data_6};
FLOATUNION data[numGauges] = {data_0, data_1, data_2};

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
      Serial.println("{\"tare\":true}");
      Wire.beginTransmission(8);
      Wire.write('t');
      Wire.endTransmission();
      delay(100);
    }
    
    currentTime = millis();
  }

}

void printDataToSerial(){
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
