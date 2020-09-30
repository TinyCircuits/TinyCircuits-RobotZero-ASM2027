//-------------------------------------------------------------------------------
//  TinyCircuits Robot Kit Examples
//  Last Updated 24 Feb 2020
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

void setup() {
  SerialMonitorInterface.begin(115200);
  Wire.begin();
  stepperInit();
  delay(100);
  setMotorCurrent(100);
  initPWM(maxPWM);
  //setDCMotor
}



void loop() {
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
