#include <Arduino.h>
#include "DiceInfosNeeded.h"
#include "Wire.h"
#include "I2Cdev/I2Cdev.h"
#include "MPU6050/MPU6050.h"

MPU6050 accelgyro;
int16_t initAx, initAy, initAz; // define initial accel as ...
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

int16_t onSideValue = 15000;


#define LED_PIN 13
bool blinkState = false;

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
  DiceInfosNeeded currentDice(initAx, initAy, initAz, DiceSide::SideOne);

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
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // read measurements from device

  if(ax >= onSideValue)
  {
    Serial.println("Side 4");
  }
  else if(ax <= (-onSideValue))
  {
    Serial.println("Side 3");
  }
  else if(ay >= onSideValue)
  {
    Serial.println("Side 5");
  }
  else if(ay <= (-onSideValue))
  {
    Serial.println("Side 2");
  }
  else if(az >= onSideValue)
  {
    Serial.println("Side 6");
  }
  else if(az <= (-onSideValue))
  {
    Serial.println("Side 1");
  }
  else 
  {
    Serial.println("Im turning bro!");
  }
  // display tab-separated accel/gyro x/y/z values
  /*Serial.print("a/g:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);*/

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
}
