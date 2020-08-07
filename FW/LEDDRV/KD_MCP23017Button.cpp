I2C_MomentaryButton::I2C_MomentaryButton(Adafruit_MCP23017 *mcp_chip, int buttonpin, int command)
{
  _button_pin = buttonpin;
  _command = command;
  _state_old = HIGH;
  _state = LOW;
  _mcp_chip = mcp_chip;
}
  
int I2C_MomentaryButton::init(void)
{
  _mcp_chip->pinMode(_button_pin, INPUT);
  return 1;
}
  
int I2C_MomentaryButton::readstate(void)
{
  _state = _mcp_chip->digitalRead(_button_pin);
  if(_state != _state_old)
  {
    _state_old = _state;
    Serial.print(_command);
    Serial.print(" 1 ");
    Serial.print(_state);
    Serial.print(" ");
    Serial.println(_state+1+_command);
    return 1;
  }
}
