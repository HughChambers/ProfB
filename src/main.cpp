#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  
  /*
  Run all of the initialisation functions
  If one of the functions fails to initialise,
  flag it and beep out an error code at the end of setup.
  Re-run setup until we pass initialisation without any errors
  
  If initialisation occurs without any problems, then proceed into the main loop
  */
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}




/*
-----------------------FUNCTION PLAN-------------------------------

-----------------------VOID_SETUP------------------------------

_______________________GENERAL INITIALISATION______________________
Setup serial
anything miscellaneous relating to setup

_______________________BARO INITIALISATION______________________
configure barometer settings
initialise barometer

_______________________SERVO INITIALISATION______________________
configure servo settings
initialise servo

_______________________BUZZER INITIALISATION______________________
configure buzzer settings
initialise buzzer

_______________________ROTARY SWITCH INITIALISATION______________________
configure rotary switch settings
initialise rotary switch

_______________________EEPROM INITIALISATION______________________
Configure eeprom settings
initialise eeprom



*/