# truss-firmware

//v0.1
//PERIPHERAL Arduino for taking parallel strain gauge measurements.
//Each (full bridge) Wheatstone bridge is connected to an HX711 analogue to digital converter (ADC).
//Each HX711 has a DATA (DT) pin connected to a unique pin on the Arduino.
//Each HX711 is also connected to a common CLOCK (SCK) pin.
//Vcc is Arduino logic level (+3.3V for Nano).

//Measure and transfer strain gauge values through I2C when requested from the CONTROLLER

//Controller Arduino for Strain Gauge measurements

//Communicates through I2C with PERIPHERAL Arduino to receive strain gauge measurements.
//Tare button
//Will also control a servo motor.
