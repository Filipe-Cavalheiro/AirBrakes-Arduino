# AirBrake Library
An Air Brake library for arduino to control a servo given a altitude, total velocity and vertical velocity.  When given a simulation (csv file), a PID controller processes the rocket's state in order to select required drag in order to bring your vehicle's apogee as close as possible to a given target altitude. 

Air brake library was designed to work with the [OR-airbrakes-Plugin](https://github.com/Filipe-Cavalheiro/OR-airbrakes-Plugin) it works as long as air brake surface area variation is linear and remains consistently perpendicular to the rocket's thrust vector.

The `example.csv` file used does not fit in the arduino memory so a `arduinoComunication.py` was created to give the arduino the csv file one line at a time through the serial port the code to receive the values and process them is the `rocket.ino`. 

## Installing
1. To install the library, click on release on the right.
1. Place the files in the same directory where your .ino arduino file is located 
1. Import the library using `#include "airbrakes.h"`

## Contributing
1. Run `git clone https://github.com/Filipe-Cavalheiro/AirBrakes-Arduino.git` to download all the files and run the code on the arduino IDE.
2. To use the current example given with the library a file of type csv must be included and the location of the file has to be changed to the correct location this file must have Time, Altitude, Vertical Velocity,  Total velocity. If extra rows are in the file this will be ignores as well as lines starting with #.
3. The way the csv file was intended to be created is by using a openrocket simulation with the [OR-airbrakes-Plugin](https://github.com/Filipe-Cavalheiro/OR-airbrakes-Plugin) since the motion of the arduino does not affect the data in the csv file.

## Quick Tests 
The code takes longer to run than real time taking 10min to run a 180s simulation, a copy of the arduino code is given in C this serves the purpose of running a fast test on the computer of the given .csv file allowing to see what would be the angle and area output of the given simulation if ran in the arduino.

## Dependencies 
There are no known dependencies
