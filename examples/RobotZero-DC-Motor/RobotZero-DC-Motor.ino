//-------------------------------------------------------------------------------
//  TinyCircuits RobotZero DC Motor and stepper motor example
//  Last Updated 28 Oct 2020
//
//
//
//  Written by Ben Rose for TinyCircuits, https://tinycircuits.com
//
//-------------------------------------------------------------------------------


#include <Wire.h>
#include <Wireling.h>

#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif

int maxPWM = 10000;
int steps = 300;
int stepSize = maxPWM / steps;

bool useDcMotors = true; 

void setup() {
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  if (useDcMotors) {
    DcMotorInit(maxPWM);
  } else {
    stepperInit();
  }
  delay(100);
  setMotorCurrent(100);
}

void loop() {
  if (useDcMotors) {
    DcMotorLoop();
  } else {
    stepperLoop();
  }
}

void stepperLoop() {
  setMotor(1, 10, 1000);
  setMotor(2, -10, 1000);
  while (isMotorSpinning());
  setMotor(1, -10, 1000);
  setMotor(2, 10, 1000);
  while (isMotorSpinning());
}

void DcMotorLoop() {
  int i;
  for (i = -maxPWM; i < maxPWM; i += stepSize) {
    delay(10);
    setDCMotor(1, i);
    setDCMotor(2, i);
  }
  for (i = maxPWM; i > -maxPWM; i -= stepSize) {
    delay(10);
    setDCMotor(1, i);
    setDCMotor(2, i);
  };
}
