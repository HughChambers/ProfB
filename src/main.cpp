#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>

#define SERVO_PIN 5
#define BUZZER_PIN 6
#define BUTTON 7

// State Enum

enum State
{
  RETRIEVE_DATA = 0,
  CONFIGURATION,
  IDLE,
  ASCENDING,
  APOGEE,
  DESCENDING,
  RELEASE,
  TOUCHDOWN
};

int state = 0;

int ReleaseAltitude = 0;
unsigned long LastDebounce = 0;
unsigned long CurrentTime = 0;
unsigned long PreviousTime = 0;

// put function declarations here:
void General_Init();
void Baro_Init();
void Servo_Init();
int EEPROM_Init();
int Altitude_Select();
void Buzz_Num(int num);
void StateMachine();

// Objects
Servo ReleaseServo;



void setup()
{
  // put your setup code here, to run once:
  /*
  Run all of the initialisation functions
  If one of the functions fails to initialise,
  flag it and beep out an error code at the end of setup.
  Re-run setup until we pass initialisation without any errors

  If initialisation occurs without any problems, then proceed into the main loop
  */

  // Disable sensor initialistaiton for testing
  General_Init();

  // Servo_Init();
  // Baro_Init();
  // EEPROM_Init();
  // delay(2000);
  Serial.println("Init complete");
  delay(2000);
}

void loop()
{
  // put your main code here, to run repeatedly:

  StateMachine();
  // Serial.println("Init complete");
  // delay(100);
}

// put function definitions here:

void General_Init()
{

  // Start UART comms
  Serial.begin(9600);

  // Configure I/O pins
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
}

void Servo_Init()
{
  ReleaseServo.attach(SERVO_PIN);
}

void Baro_Init()
{
  // Start baro operations
}

int EEPROM_Init()
{
  for (uint16_t i = 0; i < EEPROM.length(); i++)
  {
    // this performs as EEPROM.write(i, i)
    if (EEPROM.read(i) == 255)
    {
      return i;
    }
  }
  return 0;
}

void StateMachine()
{
  switch (state)
  {
  case RETRIEVE_DATA:
    Serial.println("I am retrieving saved data (read flights from EEPROM) and will buzz them out");

    // Insert function here to retrieve data from EEPROM
    delay(2000);
    state = 1;
    break;


  case CONFIGURATION:

    Serial.println("I will now set the Release altitude");
    ReleaseAltitude = Altitude_Select();
    Serial.print("The release altitude has been set to: ");
    Serial.println(ReleaseAltitude);
    state = 2;

    break;
  case IDLE:
  Serial.println("I am now in IDLE");
  delay(2000);
    break;
  case ASCENDING:
    break;
  case APOGEE:
    break;
  case DESCENDING:
    break;
  case RELEASE:
    break;
  case TOUCHDOWN:
    break;

  default:
    break;
  }
}

void Buzz_Num(int num)
{
  {
    Serial.println(num);
    if (num > 0)
      for (int i = 0; i < num; i++)
      {
        tone(BUZZER_PIN, 2000, 75);
        //Serial.println("buzz");
        delay(300);
      }
  }
}

int Altitude_Select()
{
  int Altitude = 0;
  unsigned long timeout = 0;

  unsigned long entertime = millis();
  // Serial.println("test 1");

  while (timeout < 5000)
  {
    timeout = millis() - entertime;
    // Serial.println("debug 1");

    if (!digitalRead(BUTTON))
    {
      unsigned long DebounceDelay = 400;
      if (millis() - LastDebounce > DebounceDelay)
      {
       // Serial.println("debug 2");
        timeout = 0;
        entertime = millis();
        Altitude = Altitude + 1;
        //Serial.println(Altitude);
        LastDebounce = millis();
        //Serial.print("Millis: ");
        //Serial.println(millis());
       // Serial.print("Last debounce: ");
        //Serial.println(LastDebounce);
        tone(BUZZER_PIN, 2000, 50);
      }
    }
  }

  if (Altitude > 1400)
    Altitude = 0;
  Buzz_Num(Altitude);
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

__________________________Altitude select_______________________
reads altitude from button presses
beeps out the altitude select

___________________________Buzzer function______________________
beeps out a number




*/