#include "KD_ardu_button.h"

Button::Button(int _pin, bool _pullup, bool _inverted) {
  pin = _pin;
  setPullup(_pullup);
  setInverted(_inverted);
  released = !state();
};

Button::Button(int _pin, bool _pullup) {
  pin = _pin;
  setPullup(_pullup);
  released = !state();
};

Button::Button(int _pin) {
  pin = _pin;
  setPullup(false);
  released = !state();
};

void Button::setPullup(bool _pullup) {
  if(_pullup) {
    pinMode(pin, INPUT_PULLUP);
  } else {
    pinMode(pin, INPUT);
  }
};

bool Button::state() { //returns state
  if(!inverted && digitalRead(pin) || inverted && !digitalRead(pin)) { // button is pressed (inverted != digitalRead(pin))
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
