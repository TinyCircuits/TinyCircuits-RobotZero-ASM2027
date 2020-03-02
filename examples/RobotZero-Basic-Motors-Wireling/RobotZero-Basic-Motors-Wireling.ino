/**********************************************************************
 * RobotZero Peripheral Basics
 * This program illustrates the abilities of the RobotZero processor 
 * board by spinning any attached motors or servos, while reading the
 * accelerometer data of the onboard 9-Axis sensor and Color
 * Sensor data for a Color Sensor Wireling attached to port 0. 
 * 
 * NOTES: 
 *   - Serial Monitor must be open for program to run.
 *   - Battery must be plugged in to power motors.
 * 
 * Hardware by: TinyCircuits
 * Written by: Ben Rose & Laveréna Wienclaw for TinyCircuits
 * 
 * Initialized: June 2019
 * Last modified: Jan 2020
 **********************************************************************/

#include <Wire.h>
#include <MotorDriver.h> // Download latest here: https://github.com/TinyCircuits/TinyCircuits-TinyShield_Motor_Library/archive/master.zip
#include <Wireling.h>
#include "Adafruit_TCS34725.h"  // The library used for the Color Sensor Wireling

MotorDriver servo(15);// Value passed is the address- RobotZero is always address 15
#define SerialMonitorInterface SerialUSB // for SAMD21 processors
int aX, aY, aZ, gX, gY, gZ, mX, mY, mZ, tempF; // 9-Axis variables 
uint16_t r, g, b, c, colorTemp, lux; // Color Sensor Wireling variables

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

void setup() {
  // Enable and Power Wireling
  Wireling.begin();
  Wireling.selectPort(0);

  SerialMonitorInterface.begin(9600);
//  while (!SerialMonitorInterface); // Halt everything until Serial Monitor is opened
  Wire.begin();

  // Initialize stepper motor driver
  stepperInit();

  // Initialize 9-Axis Sensor
  IMUInit();
  delay(100);

  // Initialize servo driver
  if(servo.begin(20000)){
    while(1);
    SerialMonitorInterface.println("Servo driver not detected!");
  }

  // Initialize Color Sensor Wireling
  if (tcs.begin()) {
    SerialMonitorInterface.println("Found sensor");
  } else {
    SerialMonitorInterface.println("No TCS34725 found ... check your connections");
//    while (1);
  }
}

void loop() {
  tcs.setInterrupt(false); // Turn onboard Color Sensor Wireling LEDs off

  // Motors go forward
  goForward();
  
  // Command all Servos to rotate
  servo.writeCommand(COMMAND_SERVO_1,1000);
  servo.writeCommand(COMMAND_SERVO_2,1000);
  servo.writeCommand(COMMAND_SERVO_3,1000);
  servo.writeCommand(COMMAND_SERVO_4,1000);
  delay(1000); //delay to allow motors and servos to move
  
  tcs.setInterrupt(true); // Turn onboard Color Sensor Wireling LEDs on

  SerialMonitorInterface.println(" ");
  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  SerialMonitorInterface.print("Color Temp: "); SerialMonitorInterface.print(colorTemp); SerialMonitorInterface.print(" K, ");
  SerialMonitorInterface.print("Lux: "); SerialMonitorInterface.print(lux, DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("R: "); SerialMonitorInterface.print(r, DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("G: "); SerialMonitorInterface.print(g, DEC); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("B: "); SerialMonitorInterface.print(b); SerialMonitorInterface.print(", ");
  SerialMonitorInterface.print("Clr: "); SerialMonitorInterface.print(c, DEC);
  SerialMonitorInterface.println(" ");
  
  stopMotors();
  servo.writeCommand(COMMAND_SERVO_1,2000);
  servo.writeCommand(COMMAND_SERVO_2,2000);
  servo.writeCommand(COMMAND_SERVO_3,2000);
  servo.writeCommand(COMMAND_SERVO_4,2000);
  delay(1000);

  // Read and print 9-Axis Accelerometer and Temperature data 
  //(gyro and magentometer data can also be printed using the gX, gY, gZ, mX, mY, and mZ variables)
  IMURead();
  SerialMonitorInterface.print("X: ");
  SerialMonitorInterface.print(aX);
  SerialMonitorInterface.print(" Y: ");
  SerialMonitorInterface.print(aY);
  SerialMonitorInterface.print(" Z: ");
  SerialMonitorInterface.print(aZ);
  SerialMonitorInterface.print(" Temp: ");
  SerialMonitorInterface.println(tempF);
}

void goForward(){
  setMotorCurrent(100);
  setMotor(1, 10);
  setMotor(2, -10);
}

void stopMotors(){
  setMotorCurrent(1);
  setMotor(1, 0);
  setMotor(2, 0);
}
