#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>

#define buffer 16
SimpleKalmanFilter pressureKalmanFilter(5, 5, 5);
Adafruit_BMP280 bmp; // I2C
float groundPressure;
int CurrentAddress = 0;
int address = 0;

// put function declarations here:
int myFunction(int, int);

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
  //run state machine here 
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
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