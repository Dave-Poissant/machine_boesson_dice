#include <Arduino.h>
#include "DiceInfosNeeded.h"
#include "Wire.h"
#include "I2Cdev/I2Cdev.h"
#include "MPU6050/MPU6050.h"

MPU6050 accelgyro;
int16_t initAx, initAy, initAz; // define initial accel as ...
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

DiceInfosNeeded currentDice;
DiceSide lastSide;

unsigned long timerStarts;
unsigned long timerStops;
unsigned int switchingSideTime = 2000;

#define LED_PIN 13
bool blinkState = false;
bool startTimerReseted = false;

void setNewSide();
void setTimers();

void setup() {
  // put your setup code here, to run once:

  Wire.begin();      // join I2C bus
  Serial.begin(9600);    //  initialize serial communication
  Serial.println("Initializing I2C devices...");
  char data = 'n';
  Serial.println("Ready for init? ('y/n'): ");
  while(Serial.available() == 0) {}
  data = Serial.read();
  
  if(data != 'y')
  {
    Serial.println("Waiting for being ready! ('y')");
  }
  while(data != 'y')
  {
    data = Serial.read();
  }
  
  accelgyro.initialize();
  delay(50);
  accelgyro.getAcceleration(&initAx, &initAy, &initAz);
  currentDice.setInitialValue(initAx, initAy, initAz, currentDice.wichSide(initAx, initAy, initAz));
  lastSide = static_cast<DiceSide>(currentDice.getCurrentDiceSide());

  Serial.print("First angular position: ");
  Serial.print(initAx);
  Serial.print("   ");
  Serial.print(initAy);
  Serial.print("   ");
  Serial.println(initAz);
  Serial.print("For dice on side showing value: ");
  Serial.println(currentDice.getCurrentDiceSide());

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(LED_PIN, OUTPUT);  // configure LED pin
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(50);
  setTimers();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  if(currentDice.wichSide(ax, ay, az) <= DiceSide::SideSix)
  {
    if(currentDice.wichSide(ax, ay, az) != lastSide)
    {
      setNewSide();
    }
  }
  
  if((timerStops - timerStarts) >= switchingSideTime)
    {
      //TODO: Add wifi communication HERE babay
      Serial.print("Side ");
      Serial.print(currentDice.getCurrentDiceSide());
      Serial.println(" send com to CHUGGG!");
    }
    else 
    {
      Serial.println("Im turning bro!");
    }

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  timerStops = millis();
}

void setTimers()
{
  if(!startTimerReseted)
  {
    timerStarts = millis();
    timerStops = millis();
    startTimerReseted = true;
  }
}

void setNewSide()
{
  timerStarts = millis();
  timerStops = millis();
  lastSide = static_cast<DiceSide>(currentDice.getCurrentDiceSide());
  currentDice.setNewSide(currentDice.wichSide(ax, ay, az));
}

