#include "Adafruit_MCP23017.h"
#include <Arduino.h>

byte numbersToDisplay[] = {
  B10001000,  //  0
  B11101011,  //  1
  B01001100,  //  2
  B01001001,  //  3
  B00101011,  //  4
  B00011001,  //  5
  B00011000,  //  6
  B11001011,  //  7
  B00001000,  //  8
  B00001011,  //  9
  B00001010,  //  A
  B00111000,  //  B
  B10011100,  //  C
  B01101000,  //  D
  B00011100,  //  E
  B00011110,  //  F
  B01011101  //  Error
};

class MCP23017SevenSegDisplay {

  private:
  Adafruit_MCP23017 *mcp_chip;
  bool inverted = LOW;
  int pins[8];

  public:
  MCP23017SevenSegDisplay(Adafruit_MCP23017 *_mcp_chip);
  void init(bool _inverted, int _pin_a, int _pin_b, int _pin_c, int _pin_d, int _pin_e, int _pin_f, int _pin_g, int _pin_dp);
  void setNumber(int _number);
};
