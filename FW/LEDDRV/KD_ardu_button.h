#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Ethernet.h>


class Button {

  private:
  int pin = 0;
  bool released = true;
  bool inverted = false;

  public:
  Button(int, bool, bool);
  Button(int, bool);
  Button(int);
  Button(void) { }
  void setPin(int _pin) { pin = _pin; }
  void setPullup(bool);
  void setInverted(bool _invert) { inverted = _invert; released = !state(); }
  bool pressed(void);
  bool state(void);
  bool isReleased(void);
};
