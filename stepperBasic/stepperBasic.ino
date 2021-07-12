//Simple firmware for turning stepper clockwise and anticlockwise at different speeds

#define SDIR 2
#define SPUL 5

#define BUP 14
#define BDOWN 20

#define BSPE 9

#define LED_SPEED_1 7
#define LED_SPEED_2 8

int microsecond_delay;

int delays[4] = {1000, 2000, 6000, 10000};
int speed_index = 0;

void setup() {
  
  pinMode(BUP, INPUT_PULLDOWN);
  pinMode(BDOWN, INPUT_PULLDOWN);
  pinMode(BSPE, INPUT_PULLDOWN);

  pinMode(SDIR, OUTPUT);
  pinMode(SPUL, OUTPUT);

  pinMode(LED_SPEED_1, OUTPUT);
  pinMode(LED_SPEED_2, OUTPUT);


  microsecond_delay = delays[speed_index];
  setSpeedLED(speed_index);
  
}

void loop() {
  if(digitalRead(BUP) == HIGH){
    step(1);
  } 
  else if(digitalRead(BDOWN) == HIGH){
    step(-1);
  } 
  else if(digitalRead(BSPE) == HIGH){
    speed_index += 1;
    
    if(speed_index == 4){
      speed_index = 0;
    }
    
    microsecond_delay = delays[speed_index];
    setSpeedLED(speed_index);
    
  }


}

void step(int num_steps){
 
  if(num_steps > 0){
    digitalWrite(SDIR, HIGH);
  } else {
    digitalWrite(SDIR, LOW);
  }
  
  for (int i=0;i<abs(num_steps);i++){
    digitalWrite(SPUL, HIGH);
    delayMicroseconds((1.0/10.0)*microsecond_delay);
    digitalWrite(SPUL, LOW);
    delayMicroseconds((9.0/10.0)*microsecond_delay);
  }
}

void setSpeedLED(int index){
  
  if(index == 0){
      digitalWrite(LED_SPEED_1, HIGH);
      digitalWrite(LED_SPEED_2, HIGH);
    } else if(index == 1){
      digitalWrite(LED_SPEED_1, HIGH);
      digitalWrite(LED_SPEED_2, LOW);
    } else if(index == 2){
      digitalWrite(LED_SPEED_1, LOW);
      digitalWrite(LED_SPEED_2, HIGH);
    }
    else if(index == 3){
      digitalWrite(LED_SPEED_1, LOW);
      digitalWrite(LED_SPEED_2, LOW);
    }
    delay(200);
}
