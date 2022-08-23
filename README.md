# truss-firmware

UNDER ACTIVE DEVELOPMENT - no guarantees that there are no issues currently.

Arduino Nano 33 IoT firmware for controlling and reading a 6 member truss remote laboratory for use with Structural Engineering and Engineering Design courses at the University of Edinburgh, 2022-23. 

![mini-truss](images/mini-truss.jpg)

Each truss member has a full-bridge strain gauge arrangement using two biaxial strain gauges (Kyowa KFGS-2-120-D16-11 - link to docs). Strain gauge voltages are converted to digital signals using the HX711 ADC (add link to docs) and a [HX711 library](https://www.arduino.cc/reference/en/libraries/hx711-arduino-library/).  The truss is loaded using a linear servo (add link to docs) controlled by the firmware.

Truss remote lab UI available [here](https://github.com/dpreid/truss-ui). 
Truss remote lab hardware documents and PCB schematics are available here (link to be updated).

# mini-truss remote lab (in development)

The mini-truss remote lab firmware uses a state machine with 11 states for reading, writing, loading, taring and resetting both strain gauges and load cell. A single microcontroller (Arduino Nano 33 IoT) performs both reading and writing of data. The state machine will remain in the READ state until a user command to change state. Upon performing that state the state machine will return to READ.

![mini-truss-schematic](images/mini-truss-schematic.png)

![mini-truss-servo](images/linear_actuator.jpg)


# Large truss (single, prototype)

The prototype large truss uses very similar firmware to the mini-truss except it utilises 2 microcontrollers - one is dedicated to reading the gauge values; the other acts as the state machine for all other functions. Potential issues with the Arduino I2C communications resulted in the simplification down to a single microcontroller for the mini-truss experiments.

![truss](images/truss-image.jpg)
![truss-schematic](images/large-truss-schematic.png)

