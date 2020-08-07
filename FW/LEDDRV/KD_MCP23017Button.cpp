MCP23017Button::MCP23017Button(Adafruit_MCP23017 *mcp_chip, int _pin, bool _pullup, bool _inverted)
{
  _mcp_chip = mcp_chip;
  _button_pin = buttonpin;
  setPullup(_pullup);
  setInverted(_inverted);
  released = !state();
}

MCP23017Button::MCP23017Button(Adafruit_MCP23017 *mcp_chip, int _pin, bool _pullup) {
  _mcp_chip = mcp_chip;
  _button_pin = _pin;
  setPullup(_pullup);
  released = !state();
};

Button::Button(Adafruit_MCP23017 *mcp_chip, int _pin) {
  _mcp_chip = mcp_chip;
  _button_pin = _pin;
  setPullup(false);
  released = !state();
};
  
int MCP23017Button::init(void)
{
  _mcp_chip->pinMode(_button_pin, INPUT);
  return 1;
}

void Button::setPullup(bool _pullup) {
  if(_pullup) {
    _mcp_chip->pinMode(pin, INPUT_PULLUP);
  } else {
    _mcp_chip->pinMode(pin, INPUT);
  }
};

bool MCP23017Button::state(void)
{
  if(!inverted && _mcp_chip->digitalRead(_button_pin) || inverted && !_mcp_chip->digitalRead(_button_pin)) { // button is pressed (inverted != digitalRead(pin))
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

bool MCP23017Button::isReleased() {
  return released;
}

bool MCP23017Button::setDevice(Adafruit_MCP23017 *_mcp_chip)
{
  _mcp_chip = mcp_chip;
}

void I2C_MomentaryButton::setPin(int _pin) 
{ 
  pin = _pin; 
}

void MCP23017Button::setInverted(bool _invert) 
{
  inverted = _invert; 
  released = !state();
}
