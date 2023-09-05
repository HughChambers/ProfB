#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "bmp_i2c.h"
#include <Servo.h> 

#define SERVO_PIN 5
#define BUZZER_PIN 6
#define BUTTON 7

// put function declarations here:
void General_Init();
void Baro_Init();
void Servo_Init();
int EEPROM_Init();
int Altitude_Select();

// Objects
Servo ReleaseServo;
BMP3_I2C bmp(0x76);
struct bmp_data sensorData;

void setup() {
  // put your setup code here, to run once:
  /*
  Run all of the initialisation functions
  If one of the functions fails to initialise,
  flag it and beep out an error code at the end of setup.
  Re-run setup until we pass initialisation without any errors
  
  If initialisation occurs without any problems, then proceed into the main loop
  */

  General_Init();
  Servo_Init();
  Baro_Init();
  EEPROM_Init();

}

void loop() {
  // put your main code here, to run repeatedly:


  //Statemachine();


}

// put function definitions here:



void General_Init(){

  // Start UART comms
  Serial.begin(9600);

  // Configure I/O pins
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
}

void Servo_Init(){
  ReleaseServo.attach(SERVO_PIN);
}

void Baro_Init(){
  // Start baro operations
  bmp.init();

  // set sensor in forced mode with desired settings
  bmp.setSensorInForcedMode(BMP3_OVERSAMPLING_16X, BMP3_OVERSAMPLING_2X, BMP3_IIR_FILTER_COEFF_3);
}

int EEPROM_Init(){
for (uint16_t i = 0; i < EEPROM.length(); i++) {
    // this performs as EEPROM.write(i, i)
    if(EEPROM.read(i) == 255){
      return i;
    }
  }
  return 0;
}




int Altitude_Select(){
  int Altitude = 0;
  int timeout = 0;
  int entertime = millis();

  while(timeout < 5000 ){
    timeout = millis() - entertime;
  if(digitalRead(BUTTON)){
    Altitude = Altitude + 100;
    if(Altitude > 1400){
      Altitude = 0;
    }
    timeout = 0;
    }
  }
  return Altitude;
}



/*
-----------------------FUNCTION PLAN-------------------------------

-----------------------VOID_SETUP------------------------------

_______________________GENERAL INITIALISATION______________________ NEEDS TESTING*******************
Setup serial
anything miscellaneous relating to setup

_______________________BARO INITIALISATION______________________ NEEDS TESTING*******************
configure barometer settings
initialise barometer

_______________________SERVO INITIALISATION______________________ NEEDS TESTING*********************
configure servo settings
initialise servo

_______________________EEPROM INITIALISATION______________________ NEEDS TESTING***********************
Configure eeprom settings
initialise eeprom



-----------------------VOID_LOOP------------------------------

___________________________Configuration State______________________

// 
*/