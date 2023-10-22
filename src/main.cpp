#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>
#include <Servo.h>


#define buffer 16
#define SERVO_PIN 18
#define BUZZER_PIN 1
//#define BUZZER_PIN A1
#define BUTTON 6
#define LAUNCH_DETECT_ALTITUDE 30/2 // halve because of altitude halve
#define ARMING_ALTITUDE 25/2 // halve because of altitude halve
#define TOUCHDOWN_ALTITUDE 50/2
#define ANGLE_OPEN 130 //Servo min angle
#define ANGLE_CLOSED 72 //Servo max angle
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

const unsigned long AltitudePrintDuration = 250;
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
void buzzer_idle();


void Setup_Pass();
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
  Setup_Pass();


  // for(int i = 0; i< EEPROM.length(); i++){
  //   EEPROM.write(i, 255);
  //   delay(25);
  // }
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
    delay(3000);

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
  delay(10000);
  ReleaseServo.write(ANGLE_CLOSED);
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
                  Adafruit_BMP280::STANDBY_MS_250); /* Standby time. */

  groundPressure = bmp.readPressure() / 100; // /100 converts Pa to hPa
  Serial.print("Ground level air pressure: ");
  Serial.print(groundPressure);
  Serial.println(" hPa");
}

//Call eeprom.length it returns whatever length it is
//Function needs to be rewritten if we are using floats 
int EEPROM_Init()
{
  for (uint16_t i = 0; i < 1024; i++)
  {
    // this performs as EEPROM.write(i, i)
    int val = EEPROM.read(i);
    delay(10);
    if (val == 255){
      return i;
      break;
    }
  }
    }

//DetectApogee instead of PrintApogee
//Use the kalman filter to filter the alitutde to the apogee readings then save that one apogee reading to the EEPROM address read and write it 
//Barometer initialisation //Kalman filter ---> save

//Tell functon where data goes
//Lockout based system, set a timer wait 2 seconds for example, check it again
//if this alittude is still descending then we believe we are still descending
//if new measurement is higher reset the counter 
int APOGEE_DETECTION(byte lastAltitude)
{
    Serial.println("Trying to detect apogee");     
      byte Filtered_altitude = KALMAN_ALTITUDE();
      //bool apogeeReached = false;

      Serial.print("lastAltitude:   ");
      Serial.println(lastAltitude);



      if (Filtered_altitude < lastAltitude)
        {
            Serial.println("APOGEE");
            ApogeeAltitude = lastAltitude;
            return 1;
        }
       
        else
        //lastAltitude = Filtered_altitude;
        //Serial.println(Filtered_altitude);
        return 0;

}
  
  //need to read all 4 bytes and get value for all of them
  //alter EEPROM.READ function
  void PRINT_APOGEE(int CurrentAddress)
  {
    Serial.print("Writing APOGEE to address:   ");
    Serial.println(CurrentAddress);
    Serial.print("ApogeeAltitude:   ");
    Serial.println(ApogeeAltitude);
    delay(50);
    EEPROM.write(CurrentAddress,ApogeeAltitude);
    delay(50);
    
   
    byte rawAlt = RAW_ALTITUDE();
    EEPROM.write(CurrentAddress+1, rawAlt);
  }

  void PRINT_APOGEE_INT(int CurrentAddress)
  {
    EEPROM.write(CurrentAddress,ApogeeAltitude);
  }

  



//Still need to incorporate millis e.g. beep every 5seconds 


void buzzer_idle(){
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
  tone(BUZZER_PIN, 4000, 1000);
  delay(10000);
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
    Serial.println(" metres");
    


    state = IDLE;

    break;
  case IDLE:
  
    // while altitude is less than logging threshold, do nothing
    while (KALMAN_ALTITUDE() < LAUNCH_DETECT_ALTITUDE)
    {
      buzzer_idle();
      
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
  
  while (KALMAN_ALTITUDE() > ARMING_ALTITUDE)
  {
    //Serial.println("test while loop");
    //delay(50);
   // Serial.println("test while loop");
        //delay(50);
      byte lastAltitude = KALMAN_ALTITUDE();
    if(APOGEE_DETECTION(lastAltitude) == 1){
      state = APOGEE;
      break;
    };


  };
  //Save to EEPROM

  case APOGEE:
  {
    PRINT_APOGEE(CurrentAddress);
    Serial.println("I am at apogee");
    state = DESCENDING;
    
    //break;
  }
  case DESCENDING:
  {
  Serial.println("entering descent");
  while (1)
  {
    if(KALMAN_ALTITUDE() < ReleaseAltitude){
    Serial.println("Release parachute");
    state = RELEASE;
    break;
    }
  }
   
  }
  case RELEASE:
  
    //servo release function
    Servo_ReleaseDeployed();
    Serial.println("parachute released");
   state = FINALDESCENT;


  case FINALDESCENT:
  while(1){
  if(KALMAN_ALTITUDE() < TOUCHDOWN_ALTITUDE)
  {
    Serial.println("enter touchdown");
    state = TOUCHDOWN;
    break;
  }
  }

  case TOUCHDOWN:
  {
    while(1){
    Serial.println("Touchdown");
    //perform some sort of idle buzzer 
    //buzzer_touchdown();
    Serial.print("Apogee altitude:   ");
    Serial.println(ApogeeAltitude*2);
    Buzz_NumThousands(ApogeeAltitude*2);
    delay(2000);
    Buzz_NumHundreds(ApogeeAltitude*2);
    delay(2000);
    Buzz_NumTens(ApogeeAltitude*2);
    delay(2000);
    Buzz_NumOnes(ApogeeAltitude*2);
    delay(10000);
  }


  default:
    break;
  }
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
  Buzz_NumOnes(CurrentAddress);
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
  delay(2000);
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

  return altitude/2;
}

float KALMAN_ALTITUDE()
{
  //implement a timer for this
  float estimated_altitude = pressureKalmanFilter.updateEstimate(RAW_ALTITUDE());

  float currentMillis = millis();
    if (currentMillis - previousKalmanMillis < AltitudePrintDuration) {
       // Turn the buzzer on
       // do nothing
    } 
    else {
      Serial.print("Kalman Altitude: ");
      Serial.print(estimated_altitude);
      Serial.println(" m");
      previousKalmanMillis = currentMillis;
    }
 return estimated_altitude;

}
 

void Setup_Pass(){
  Serial.println("Init complete");
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
