#pragma once

#include <Arduino.h>

class Button {

  private:
  int button_pin = 0;
  bool released = true;
  bool inverted = false;

  public:
  Button(int _button_pin, bool _pullup, bool _inverted);
  Button(int _button_pin, bool _pullup);
  Button(int _button_pint);
  Button(void) { }
  void setPin(int);
  void setPullup(bool);
  void setInverted(bool);
  bool pressed(void);
  bool state(void);
  bool isReleased(void);
};
