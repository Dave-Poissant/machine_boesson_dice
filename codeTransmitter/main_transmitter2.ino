#include <Arduino.h>
#include <DiceInfosNeeded.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24Network.h>

MPU6050 accelgyro;
int16_t initAx, initAy, initAz; // define initial accel as ...
int16_t ax, ay, az;  // define accel as ax,ay,az
int16_t gx, gy, gz;  // define gyro as gx,gy,gz

DiceInfosNeeded currentDice;
DiceSide lastSide;

unsigned long timerStarts;
unsigned long timerStops;
unsigned int switchingSideTime = 1000;

#define LED_PIN 13

bool blinkState = false;
bool startTimerReseted = false;

void setNewSide();
void setTimers();

RF24 radio(7, 8); // CE, CSN
RF24Network network(radio);
bool rslt;

const uint16_t dice2 = 02;   // Address of our node in Octal format ( 04,031, etc)
const uint16_t master00 = 00;    // Address of the other node in Octal format

void setup() {
  // put your setup code here, to run once:
  Wire.begin();      // join I2C bus
  Serial.begin(9600);    //  initialize serial communication
  SPI.begin();
  radio.begin();
  network.begin(90, dice2);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
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
  char result[] = "2,0;";
  String str;
  char side_char[] = "0";
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
      Serial.print("Side ");
      Serial.println(currentDice.getCurrentDiceSide());
      Wire.write(currentDice.getCurrentDiceSide());  // Envoi le chiffre au Arduino Master
      str = String(currentDice.getCurrentDiceSide());
      str.toCharArray(side_char,2);
      result[2] = side_char[0];
    }
  else 
  {
    result[2] = '0';
    Serial.println("Im turning bro!");
  }
  Serial.println(result);
  RF24NetworkHeader header(master00);
  rslt = network.write(header, &result, sizeof(result));  
  if (rslt) {
    Serial.println("Acknoledge");
  }
  else {
    Serial.println("Failed");
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
