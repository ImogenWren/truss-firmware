/*
 * LinearServo library
 *
	Author: David Reid
 *
 
 For remote control of the Actuonix L16-50-150-6-R linear actuator.
 Library only works for 3 pin servo - signal pin, +VCC, GND
 */

// ensure this library description is only included once
#ifndef LinearServo_h
#define LinearServo_h

#include <Arduino.h>

// library interface description
class LinearServo {
  public:
    // constructor:
    LinearServo(int signal_pin);
    
    // speed setter method:
    void setDelay(long delay);

    // move multiple steps:
    void set_position(int position);
    
    void zero();

  private:
    void singleStep(int position);

    int pwmFreq;            // Direction of rotation
    unsigned long delay; // delay between steps, in micros, based on speed
    int min_sig;		
    int max_sig;
    int current_position;
    int move_position;		
    
    // motor pin numbers:
    int signal_pin;
    
};

#endif

