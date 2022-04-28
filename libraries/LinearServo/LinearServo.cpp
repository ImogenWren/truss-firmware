/*
 * LinearServo library
 *
	Author: David Reid
 *
 
 For remote control of the Actuonix L16-50-150-6-R linear actuator.
 Library only works for 3 pin servo - signal pin, +VCC, GND
 */

#include <Arduino.h>
#include "LinearServo.h"

/*
 * one-wire constructor.
 * Sets which pin should control the motor.
 * Speed of servo can be set in terms of the delay between steps in milliseconds
 */
LinearServo::LinearServo(int signal_pin)
{
  this->pwmFreq = 490;	//pwm frequency of most Arduino boards inlcuding Uno
  this->delay = 10000L;		//step period in microseconds. Default value
  
  // Arduino pins for the motor control connection:
  this->signal_pin = signal_pin;

  // setup the pins on the microcontroller:
  pinMode(this->signal_pin, OUTPUT);
  
  //set the minimum signal for full retraction of the linear servo
  this->min_sig = round(255*1/(1000/this->pwmFreq));
  //set the maximum signal for full extension of the linear servo
  this->max_sig = round(255*2/(1000/this->pwmFreq));
  
  this->current_position = 0;		//between 0(full retraction) and 100(full extension)
  this->move_position = 0;

}

/*
 * Sets the delay between steps in millseconds -> effectively the speed of the stepper
 */
void LinearServo::setDelay(long whatDelay)
{
  this->delay = whatDelay;
}

/*
 * Zeros the servo, returning it to fully retracted
 */
void LinearServo::zero()
{
	for(int i=255;i>0;i--){
		analogWrite(this->signal_pin, i);
      		delayMicroseconds(10000);
  	}
  	this->current_position = 0;
  	this->move_position = 0;
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void LinearServo::set_position(int position)
{
	this->move_position = position;
	
	if(this->move_position > this->current_position){
	
	    this->current_position += 1;
	    singleStep(this->current_position);
	    
	  } else if(this->move_position < this->current_position){
	  
	    this->current_position -= 1;
	    singleStep(this->current_position);
	   
	  } 
}

/*
 * Moves the motor a single step in the direction already defined.
 */
void LinearServo::singleStep(int position)
{
    	float high_delay = 1000+(position*10);
	float low_delay = this->delay - high_delay;
	digitalWrite(this->signal_pin, HIGH);
	delayMicroseconds(high_delay);
	digitalWrite(this->signal_pin, LOW);
	delayMicroseconds(low_delay);
    
  
}

