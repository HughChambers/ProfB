#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>
#include <Servo.h>

#define buffer 16
#define SERVO_PIN 5
#define BUZZER_PIN 6
#define BUTTON 7

float groundPressure;
int CurrentAddress = 0;
int address = 0;
int state = 0;
int ReleaseAltitude = 0;

// Timer variables
unsigned long LastDebounce = 0;
unsigned long CurrentTime = 0;
unsigned long PreviousTime = 0;


SimpleKalmanFilter pressureKalmanFilter(5, 5, 5);
Adafruit_BMP280 bmp; // I2C
Servo ReleaseServo;

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

// put function declarations here:
void General_Init();
void baromSetup();
void Servo_Init();
int EEPROM_Init();
int Altitude_Select();
void printAPOGEE(int CurrentAddress);
void Buzz_Num(int num);
void StateMachine();
float RAW_ALTITUDE();
float KALMAN_ALTITUDE();
void readFlights();

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


// Debug
  EEPROM.write(0, 5);
  EEPROM.write(1, 2);
  EEPROM.write(2, 7);
  CurrentAddress = 3;

  General_Init();
  //baromSetup();
  Servo_Init();
  CurrentAddress = EEPROM_Init();
  Serial.print("There are ");
  Serial.print(CurrentAddress);
  Serial.println(" flight(s) stored");
  delay(5000);
  Serial.println("Init complete");
  delay(5000);


}

void loop()
{
  // put your main code here, to run repeatedly:
  //run state machine here 

  StateMachine();

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

void baromSetup()
{
   //Serial.begin(9600);
  while (!Serial)
  delay(100); // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  // Serial.println("test");
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);

  // status = bmp.begin(0x76);
  if (!status)
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  groundPressure = bmp.readPressure() / 100; // /100 converts Pa to hPa
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

//Use the kalman filter to filter the alitutde to the apogee readings then save that one apogee reading to the EEPROM address read and write it 
//Barometer initialisation //Kalman filter ---> save

//Tell functon where data goes
void printAPOGEE(int CurrentAddress)
{
      float lastAltitude = KALMAN_ALTITUDE();
      float Filtered_altitude = KALMAN_ALTITUDE();
      bool apogeeReached = false;
      if ((Filtered_altitude < lastAltitude) and (apogeeReached == false))
        {
            Serial.println("APOGEE");
            apogeeReached = true;
            EEPROM.put(CurrentAddress,Filtered_altitude);
        }
        lastAltitude = Filtered_altitude;
      Serial.println(Filtered_altitude);
    }
 
void StateMachine()
{
  switch (state)
  {
  case RETRIEVE_DATA:
    Serial.println("I am retrieving saved data (read flights from EEPROM) and will buzz them out");
    
    readFlights();
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

void readFlights(){
  for(int i = 0; i < CurrentAddress; i++){
    Serial.print("Apogee ");
    Serial.print(i);
    Serial.print(": ");
    Buzz_Num(EEPROM.read(i));
    delay(5000);
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

float RAW_ALTITUDE()
{
  /**Adjust pressure at sealevel for where you are*/ /*write a function to acquire that number*/

  float altitude = bmp.readAltitude(groundPressure);

  return altitude;
}

float KALMAN_ALTITUDE()
{
  float estimated_altitude = pressureKalmanFilter.updateEstimate(RAW_ALTITUDE());
  return estimated_altitude;
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