#include "KD_ardu_button.h"

Button::Button(int _button_pin, bool _pullup, bool _inverted) {
  button_pin = _button_pin;
  setPullup(_pullup);
  setInverted(_inverted);
  released = !state();
}

Button::Button(int _button_pin, bool _pullup) {
  button_pin = _button_pin;
  setPullup(_pullup);
  released = !state();
}

Button::Button(int _button_pin) {
  button_pin = _button_pin;
  setPullup(false);
  released = !state();
}

void Button::setPullup(bool _pullup) {
  if(_pullup) {
    pinMode(button_pin, INPUT_PULLUP);
  } else {
    pinMode(button_pin, INPUT);
  }
}

bool Button::state() { //returns state
  if(!inverted && digitalRead(button_pin) || inverted && !digitalRead(button_pin)) { // button is pressed (inverted != digitalRead(button_pin))
    return true;  
  }
  else {
    return false;
  }
}

bool Button::pressed() { //returns true if the button has been released and pressed
  if(released && state()) { //button has been released and pressed
    released = false;
    return true;
  } else if(state()) { //button is continously pressed
    return false;
  } else if(!state()) { //button is not pressed
    released = true;
    return false;
  } else {
    return false;
  }
}

bool Button::isReleased() {
  return released;
}

void Button::setPin(int _button_pin) 
{ 
  button_pin = _button_pin; 
}

void Button::setInverted(bool _invert) 
{
  inverted = _invert; 
  released = !state();
}
