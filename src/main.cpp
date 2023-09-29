#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>
#include <Servo.h>


#define buffer 16
#define SERVO_PIN 5
//#define BUZZER_PIN 1
#define BUZZER_PIN A1
#define BUTTON 6
#define ASCENDING_ALTITUDE 20/2 // halve because of altitude halve
#define ARMING_ALTITUDE 20/2 // halve because of altitude halve
#define TOUCHDOWN_ALTITUDE 5
#define ANGLE_OPEN 0 //Servo min angle
#define ANGLE_CLOSED 180 //Servo max angle
#define IdleInterval 5000
#define KalmanInterval 100
#define beepduration 300

float groundPressure;
//float ApogeeAltitude;
byte ApogeeAltitude;
int CurrentAddress = 0;
int address = 0;
int state = 0;
int ReleaseAltitude = 0;

// Timer variables
unsigned long LastDebounce = 0;
unsigned long CurrentTime = 0;
unsigned long PreviousTime = 0;

unsigned long beepStartTime;  // Variable to store the start time of the beep
const unsigned long beepDuration = 1000;  // Duration of the beep in milliseconds

const unsigned long beepDuration_touchdown = 200;  // Duration of each beep in milliseconds
const unsigned long beepInterval = 500;  // Interval between beeps in milliseconds


unsigned long previousKalmanMillis = 0; // Store the last time a beep was generated
unsigned long previousIdleMillis = 0;
bool beepOn = false; // Flag to track if a beep should be generated



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
  FINALDESCENT,
  TOUCHDOWN
};

// put function declarations here:
void General_Init();
void baromSetup();
void Servo_Init();
int EEPROM_Init();
int Altitude_Select();
void PRINT_APOGEE(int CurrentAddress);
void Buzz_NumOnes(int num);
void Buzz_NumHundreds(int num);
void Buzz_NumTens(int num);
void Buzz_NumThousands(int num);


void Buzz_Setup_Pass();
void Buzz_User_Enter();
void StateMachine();
float RAW_ALTITUDE();
float KALMAN_ALTITUDE();
void readFlights();
void Servo_ReleaseDeployed();
int APOGEE_DETECTION(byte lastAltitude);

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
 //hardware serial print to native



  General_Init();
  baromSetup();
  Servo_Init();
  delay(2000);
  Serial.println("Init complete");
  
  Buzz_Setup_Pass();

   EEPROM.write(0, 187);
   EEPROM.write(1, 47);
   EEPROM.write(2, 5);
  CurrentAddress = EEPROM_Init();
  Serial.print("There are ");
  Serial.print(CurrentAddress);
  Serial.println(" flight(s) stored");
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
  ReleaseServo.write(ANGLE_OPEN);
}

void Servo_Idle()
{
  ReleaseServo.write(ANGLE_CLOSED);
}

void Servo_ReleaseDeployed()
{
  // Calibrate the servo
  ReleaseServo.write(ANGLE_OPEN);    // Move to the minimum position
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

   //status = bmp.begin(0x76);
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

//Call eeprom.length it returns whatever length it is
//Function needs to be rewritten if we are using floats 
int EEPROM_Init()
{
  for (uint16_t i = 0; i < 1024; i++)
  {
    // this performs as EEPROM.write(i, i)
    int val = EEPROM.read(i);
    //Serial.print("I am printing val:   ");
    //Serial.println(val);
    delay(10);
    if (val == 255){
      return i;
      break;
    }
  }
  //return 0;
   // {
   //   return i;
   //   Serial.print("reading");
    }
  
 // return 0;

//DetectApogee instead of PrintApogee
//Use the kalman filter to filter the alitutde to the apogee readings then save that one apogee reading to the EEPROM address read and write it 
//Barometer initialisation //Kalman filter ---> save

//Tell functon where data goes
//Lockout based system, set a timer wait 2 seconds for example, check it again
//if this alittude is still descending then we believe we are still descending
//if new measurement is higher reset the counter 
int APOGEE_DETECTION(byte lastAltitude)
{
      
      byte Filtered_altitude = KALMAN_ALTITUDE();
      bool apogeeReached = false;
      if ((Filtered_altitude < lastAltitude) and (apogeeReached == false))
        {
            Serial.println("APOGEE");
            apogeeReached = true;
            ApogeeAltitude = Filtered_altitude;
            return 1;
        }
       
        else
        lastAltitude = Filtered_altitude;
        Serial.println(Filtered_altitude);
        return 0;

}
  
  //need to read all 4 bytes and get value for all of them
  //alter EEPROM.READ function
  void PRINT_APOGEE(int CurrentAddress)
  {
    EEPROM.write(CurrentAddress,ApogeeAltitude);
    byte rawAlt = bmp.readAltitude(groundPressure);
    EEPROM.write(rawAlt, CurrentAddress+1);
  }

  void PRINT_APOGEE_INT(int CurrentAddress)
  {
    EEPROM.write(CurrentAddress,ApogeeAltitude);
  }

  

// void buzzer_idle() {
//       if (beepStartTime == 0) {
//     // Start the beep
//     tone(BUZZER_PIN, 1000);
//     beepStartTime = millis();
//       }
  
//   // Check if the beep duration has passed
//   if (millis() - beepStartTime >= beepDuration) {
//     noTone(BUZZER_PIN);  // Stop the beep
//     //beepStartTime = 0;  // Reset the beep start time
//   }
//   }

//Still need to incorporate millis e.g. beep every 5seconds 
void buzzer_idle(){
  // Beep the buzzer
  tone(BUZZER_PIN, 4000); // 1000 Hz frequency
  delay(100); // Beep duration in milliseconds
  noTone(BUZZER_PIN); // Turn off the buzzer

  // Wait for 10 seconds
  delay(10000);
}

void buzzer_idle_test(){
  unsigned long currentMillis = millis(); // Get the current time

  // Check if it's time to generate a beep
  if ((currentMillis - previousIdleMillis) > IdleInterval && !beepOn) {
    // Save the current time for the next interval
    previousIdleMillis = currentMillis;

    // Set the flag to indicate a beep should be generated
   beepOn = true;
   tone(BUZZER_PIN, 4000);
  }

  // // Check if a beep should be generated
  if (beepOn) {
    // Generate the beep
     // Adjust the beep duration as needed
     currentMillis = millis();
    if (currentMillis - previousIdleMillis < beepDuration) {
       // Turn the buzzer on
       // do nothing
    } else {
      noTone(BUZZER_PIN); // Turn the buzzer off
      beepOn = false; // Reset the flag
    }
  }
}


void buzzer_touchdown(){
  if (millis() - beepStartTime >= beepInterval) {
    // Start a new beep
    tone(BUZZER_PIN, 4000);
    beepStartTime = millis(); //Keep beeping until the rocket is retrieved
  }

}
 
void StateMachine()
{
  switch (state)
  {
  case RETRIEVE_DATA:
    
    readFlights();
    delay(2000);
    state = CONFIGURATION;
    break;

  case CONFIGURATION:

    Serial.println("I will now set the Release altitude");
    Buzz_User_Enter();
    ReleaseAltitude = Altitude_Select();
    Serial.print("The release altitude has been set to: ");
    Serial.print(ReleaseAltitude);
    Serial.println(" feet");
    Serial.println(groundPressure);


    state = IDLE;

    break;
  case IDLE:
  
    // while altitude is less than logging threshold, do nothing
    while (KALMAN_ALTITUDE() < ARMING_ALTITUDE)
    {
      buzzer_idle_test();
      //Serial.println("Servo is now locked in place");
    };
    Serial.println("leaving idle");
    noTone(BUZZER_PIN);
    state = ASCENDING;
  //delay(2000);
    break;


  // insert condition for changing state from 
  case ASCENDING:
  Serial.println("ASCENDING");
  byte lastAltitude = KALMAN_ALTITUDE();
  while (KALMAN_ALTITUDE() > ASCENDING_ALTITUDE)
  {
    APOGEE_DETECTION(lastAltitude);


  }

  state = APOGEE;
    break;
  //Save to EEPROM
  case APOGEE:
  {
    PRINT_APOGEE(CurrentAddress);
    state = DESCENDING;

  }
  state = DESCENDING;
    break;
  case DESCENDING:
  while (KALMAN_ALTITUDE() < ReleaseAltitude)
  {
    state = RELEASE;
  }
    break;
  case RELEASE:
  {
    //servo release function
    Servo_ReleaseDeployed();
  }
   state = FINALDESCENT;
    break;
  case FINALDESCENT:
  while(KALMAN_ALTITUDE() < TOUCHDOWN_ALTITUDE)
  {
    state = TOUCHDOWN;
  }
  case TOUCHDOWN:
  {
    //perform some sort of idle buzzer 
  }
    break;

  default:
    break;
  }
}

//Task//Doesnt buzz out the number 
//e.g if user has 351 it should beep 3 times quickly, then break, 5 times quickly, then break than one time quickly
//seperate each interval
void Buzz_NumOnes(int num)
{
  {
    if (((num/1U) %10) > 0){
      for (int i = 0; i < ((num/1U) %10); i++)
      {
        tone(BUZZER_PIN, 2000, 75);
        delay(300);
      }
  }
  else{
  tone(BUZZER_PIN, 300, 75);
         delay(300);
  }
  }
}

void Buzz_NumThousands(int num)
{
  {
    if ((num/1000U)%10 > 0){
      for (int i = 0; i < ((num/1000U) % 10); i++)
      {
        tone(BUZZER_PIN, 2000, 75);
        delay(300);
      }
  }
  else
  tone(BUZZER_PIN, 300, 75);
         delay(300);
  // for (int i = 0; i < 11; i++)
  //     {
  //       tone(BUZZER_PIN, 2000, 75);
  //       delay(300);
  //     }
  }
}

void Buzz_NumHundreds(int num)
{
  {
    if ((num/100U)%10> 0){
      for (int i = 0; i < ((num/100U) % 10); i++)
      {
        tone(BUZZER_PIN, 2000, 75);
        delay(300);
      }
  }
   else{
    
   tone(BUZZER_PIN, 300, 75);
         delay(300);
   }
  // for (int i = 0; i < 10; i++)
  //     {
  //       tone(BUZZER_PIN, 2000, 75);
  //       delay(300);
  //     }
  }
}

void Buzz_NumTens(int num)
{
  {
    if (num > 0){
      for (int i = 0; i < ((num/10U) %10); i++)
      {
        tone(BUZZER_PIN, 2000, 75);
        delay(300);
      }
  }
  else{
   tone(BUZZER_PIN, 300, 75);
         delay(300);
  }
}
}

// Implement breakdown of hundreds, tens and ones
void readFlights(){

  Serial.println("I am retrieving saved data (read flights from EEPROM) and will buzz them out");
  for(int i = 0; i < CurrentAddress; i++){
    Serial.print("Apogee ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(EEPROM.read(i)*2);
    Buzz_NumThousands(EEPROM.read(i)*2);
    delay(2000);
    Buzz_NumHundreds(EEPROM.read(i)*2);
    delay(2000);
    Buzz_NumTens(EEPROM.read(i)*2);
    delay(2000);
    Buzz_NumOnes(EEPROM.read(i)*2);
  
  tone(BUZZER_PIN, 1250,500);
  delay(500);
  tone(BUZZER_PIN, 1000,500);
  delay(500);
  tone(BUZZER_PIN, 750,500);
  delay(500);


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
    if (!digitalRead(BUTTON))
    {
      unsigned long DebounceDelay = 400;
      if (millis() - LastDebounce > DebounceDelay)
      {

        timeout = 0;
        entertime = millis();
        Altitude = Altitude + 1;
        LastDebounce = millis();
        tone(BUZZER_PIN, 2000, 50);
      }
    }
  }
  if (Altitude > 15)
    Altitude = 15;
  Buzz_NumOnes(Altitude);
  delay(1000);
  
  return (Altitude*100)/2;
}

float RAW_ALTITUDE()
{
  /**Adjust pressure at sealevel for where you are*/ /*write a function to acquire that number*/
  float altitude = bmp.readAltitude(groundPressure);
  Serial.println(altitude);
  return altitude/2;
}

float KALMAN_ALTITUDE()
{
  unsigned long currentMillis = millis();  // Get the current time

  // Check if the specified interval has elapsed
  if (currentMillis - previousKalmanMillis >= KalmanInterval) {
   // Save the current time as the last checked time
    previousKalmanMillis = currentMillis;
    //Serial.println("kalman");

  //implement a timer for this
  float estimated_altitude = pressureKalmanFilter.updateEstimate(RAW_ALTITUDE());
  return estimated_altitude;

}
}

void Buzz_Setup_Pass(){
  tone(BUZZER_PIN, 750,500);
  delay(500);
  tone(BUZZER_PIN, 1000,500);
  delay(500);
  tone(BUZZER_PIN, 1250,500);
  delay(500);
}

void Buzz_User_Enter(){
  tone(BUZZER_PIN, 750, 200);
  delay(500);
  tone(BUZZER_PIN, 1000, 200);
  delay(500);
  tone(BUZZER_PIN, 750, 200);
  delay(500);
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

//Task Test the kalman filter 