#pragma once

#include "Adafruit_MCP23017.h"
#include <Arduino.h>


class MCP23017SevenSegDisplay {

  private:
  Adafruit_MCP23017 *mcp_chip;
  bool inverted = LOW;
  int pins[8];

  public:
  MCP23017SevenSegDisplay(Adafruit_MCP23017 *_mcp_chip);
  MCP23017SevenSegDisplay(void) { }
  bool setDevice(Adafruit_MCP23017 *_mcp_chip);
  void init(bool _inverted, int _pin_a, int _pin_b, int _pin_c, int _pin_d, int _pin_e, int _pin_f, int _pin_g, int _pin_dp);
  void setNumber(int _number);
};
