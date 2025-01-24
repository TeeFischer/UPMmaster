# UPMmaster
Universal Press (or Pull) Machine 

This Arduino script is designed to run on a Controllino Mega, therefore the Controllino library and boards have to be added to your IDE.

[Get the CONTROLLINO stuff](https://www.controllino.com/board-library-setup-in-arduino-ide/) 

This code currently uses the HX711_ADC library from Olav Kallhovd. After downloading I set the values in the Arduino\libraries\HX711_ADC\src\config.h as follows for the fastest response time (raw data, no averaging):
#define SAMPLES          1
#define IGN_HIGH_SAMPLE  0
#define IGN_LOW_SAMPLE   0
