#include "Adafruit_MCP23017.h"
#include "KD_MCP23017_7seg.h"
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

MCP23017SevenSegDisplay::MCP23017SevenSegDisplay(Adafruit_MCP23017 *_mcp_chip) {
  mcp_chip = _mcp_chip;
}

void MCP23017SevenSegDisplay::init(bool _inverted, int _pin_a, int _pin_b, int _pin_c, int _pin_d, int _pin_e, int _pin_f, int _pin_g, int _pin_dp)
{
  pins[0] = _pin_a;
  pins[1] = _pin_b;
  pins[2] = _pin_c;
  pins[3] = _pin_d;
  pins[4] = _pin_e;
  pins[5] = _pin_f;
  pins[6] = _pin_g;
  pins[7] = _pin_dp;

  inverted = _inverted;
  
  for(int i = 0; i < 8; i++) {
      mcp_chip->pinMode(pins[i], OUTPUT);
      mcp_chip->digitalWrite(pins[i], !inverted);
  }
}

bool MCP23017SevenSegDisplay::setDevice(Adafruit_MCP23017 *_mcp_chip)
{
  mcp_chip = _mcp_chip;
  return HIGH;
}

void MCP23017SevenSegDisplay::setNumber(int _number)
{
  for(int i = 0; i < 8; i++) {
    if(inverted) {
      mcp_chip->digitalWrite(pins[i], bitRead(numbersToDisplay[_number],i));
    } else {
      mcp_chip->digitalWrite(pins[i], !bitRead(numbersToDisplay[_number],i));
    }
  }
}
