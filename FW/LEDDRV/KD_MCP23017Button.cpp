#include "KD_MCP23017Button.h"
#include "Adafruit_MCP23017.h"
#include <Arduino.h>

MCP23017Button::MCP23017Button(Adafruit_MCP23017 *_mcp_chip, int _button_pin, bool _pullup, bool _inverted)
{
  mcp_chip = _mcp_chip;
  button_pin = _button_pin;
  setPullup(_pullup);
  setInverted(_inverted);
  released = !state();
}

MCP23017Button::MCP23017Button(Adafruit_MCP23017 *_mcp_chip, int _button_pin, bool _pullup) {
  mcp_chip = _mcp_chip;
  button_pin = _button_pin;
  setPullup(_pullup);
  released = !state();
}

MCP23017Button::MCP23017Button(Adafruit_MCP23017 *_mcp_chip, int _button_pin) {
  mcp_chip = _mcp_chip;
  button_pin = _button_pin;
  setPullup(false);
  released = !state();
}
  
int MCP23017Button::init(void)
{
  mcp_chip->pinMode(button_pin, INPUT);
  return 1;
}

void MCP23017Button::setPullup(bool _pullup) {
  if(_pullup) {
    mcp_chip->pinMode(button_pin, INPUT_PULLUP);
  } else {
    mcp_chip->pinMode(button_pin, INPUT);
  }
}

bool MCP23017Button::state(void)
{
  if(!inverted && mcp_chip->digitalRead(button_pin) || inverted && !mcp_chip->digitalRead(button_pin)) { // button is pressed (inverted != digitalRead(pin))
    return true;  
  }
  else {
    return false;
  }
}

bool MCP23017Button::pressed() { //returns true if the button has been released and pressed
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

bool MCP23017Button::isReleased() {
  return released;
}

bool MCP23017Button::setDevice(Adafruit_MCP23017 *_mcp_chip)
{
  mcp_chip = _mcp_chip;
}

void MCP23017Button::setPin(int _pin) 
{ 
  button_pin = _pin; 
}

void MCP23017Button::setInverted(bool _invert) 
{
  inverted = _invert; 
  released = !state();
}
