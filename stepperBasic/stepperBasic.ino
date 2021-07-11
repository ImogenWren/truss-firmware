//Simple firmware for turning stepper clockwise and anticlockwise

//#include <Stepper.h>

#define SDIR 2
#define SPUL 5

#define BUP 8
#define BDOWN 7

const int steps_in_one_rotation = 200;
const int microsecond_delay = 500;

//Stepper stepper = Stepper(steps_in_one_rotation, SDIR, SPUL);

void setup() {
  // put your setup code here, to run once:
  pinMode(BUP, INPUT_PULLDOWN);
  pinMode(BDOWN, INPUT_PULLDOWN);

  pinMode(SDIR, OUTPUT);
  pinMode(SPUL, OUTPUT);
  
  //stepper.setSpeed(stepper_speed);

//  Serial.begin(9600);
//  while(!Serial);
}

void loop() {
  if(digitalRead(BUP) == HIGH){
    //Serial.println("UP");
    step(10);
  } else if(digitalRead(BDOWN) == HIGH){
    //Serial.println("DOWN");
    step(-10);
  }

  delay(100);

}

void step(int num_steps){
 
  if(num_steps > 0){
    digitalWrite(SDIR, HIGH);
  } else {
    digitalWrite(SDIR, LOW);
  }
  
  for (int i=0;i<abs(num_steps);i++){
    digitalWrite(SPUL, HIGH);
    delayMicroseconds(microsecond_delay);
    digitalWrite(SPUL, LOW);
    delayMicroseconds(microsecond_delay);
  }
}


//void rotateFullClockwise(){
//  stepper.step(steps_in_one_rotation);
//  
//}
//
//void rotateFullAntiClockwise(){
//  stepper.step(-steps_in_one_rotation);
//}
