/**********************************************************************
 * RobotZero Peripheral Basics
 * This program illustrates the abilities of the RobotZero processor 
 * board by spinning any attached servos using the TinyCircuits
 * ATtiny841 Arduino library.
 * 
 * NOTES: 
 *   - Serial Monitor must be open for program to run.
 *   - Battery must be plugged in to power motors.
 * 
 * Hardware by: TinyCircuits
 * Written by: Ben Rose & Laveréna Wienclaw for TinyCircuits
 * 
 * Initialized: June 2019
 * Last modified: Mar 2020
 **********************************************************************/

#include <Wire.h>
#include <ServoDriver.h> 
#include <Wireling.h>

ServoDriver servo(15);// Value passed is the address- RobotZero is always address 15
#define SerialMonitorInterface SerialUSB // for SAMD21 processors
int aX, aY, aZ, gX, gY, gZ, mX, mY, mZ, tempF; // 9-Axis variables 

void setup() {
  // Enable and Power Wirelings
  Wireling.begin();

  SerialMonitorInterface.begin(9600);
  while (!SerialMonitorInterface); // Halt everything until Serial Monitor is opened
  Wire.begin();

  // Initialize 9-Axis Sensor
  /*IMUInit();*/
  delay(100);

  // Initialize servo driver
  if(servo.begin(20000)){
    while(1);
    SerialMonitorInterface.println("Servo driver not detected!");
  }

}

void loop() {
  // Command all Servos to rotate
  servo.writeCommand(COMMAND_SERVO_1,1000);
  servo.writeCommand(COMMAND_SERVO_2,1000);
  servo.writeCommand(COMMAND_SERVO_3,1000);
  servo.writeCommand(COMMAND_SERVO_4,1000);
  delay(1000); //delay to allow motors and servos to move
  
  servo.writeCommand(COMMAND_SERVO_1,2000);
  servo.writeCommand(COMMAND_SERVO_2,2000);
  servo.writeCommand(COMMAND_SERVO_3,2000);
  servo.writeCommand(COMMAND_SERVO_4,2000);
  delay(1000);

  // Read and print 9-Axis Accelerometer and Temperature data 
  //(gyro and magentometer data can also be printed using the gX, gY, gZ, mX, mY, and mZ variables)
  /*IMURead();
  SerialMonitorInterface.print("X: ");
  SerialMonitorInterface.print(aX);
  SerialMonitorInterface.print(" Y: ");
  SerialMonitorInterface.print(aY);
  SerialMonitorInterface.print(" Z: ");
  SerialMonitorInterface.print(aZ);
  SerialMonitorInterface.print(" Temp: ");
  SerialMonitorInterface.println(tempF);*/
}
