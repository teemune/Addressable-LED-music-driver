#include "Adafruit_MCP23017.h"
#include "KD_MCP23017_7seg.h"
#include <Arduino.h>

byte numbersToDisplay[] = {
  B00111111,  //  0: 0
  B00000110,  //  1: 1
  B01011011,  //  2: 2
  B01001111,  //  3: 3
  B01100110,  //  4: 4
  B01101101,  //  5: 5
  B01111101,  //  6: 6
  B00000111,  //  7: 7
  B01111111,  //  8: 8
  B01101111,  //  9: 9
  B01110111,  //  10: A
  B01111100,  //  11: B
  B00111001,  //  12: C
  B01011110,  //  13: D
  B01111001,  //  14: E
  B01110001,  //  15: F
  B01000000,  //  16: -
  B01111001  //  17: Error
};

MCP23017SevenSegDisplay::MCP23017SevenSegDisplay(Adafruit_MCP23017 *_mcp_chip) {
  mcp_chip = _mcp_chip;
}

void MCP23017SevenSegDisplay::init(Adafruit_MCP23017 *_mcp_chip, int _pin_a, int _pin_b, int _pin_c, int _pin_d, int _pin_e, int _pin_f, int _pin_g, int _pin_dp)
{
  pins[0] = _pin_a;
  pins[1] = _pin_b;
  pins[2] = _pin_c;
  pins[3] = _pin_d;
  pins[4] = _pin_e;
  pins[5] = _pin_f;
  pins[6] = _pin_g;
  pins[7] = _pin_dp;

  mcp_chip = _mcp_chip;
  
  for(int i = 0; i < 8; i++) {
      mcp_chip->pinMode(pins[i], OUTPUT);
      mcp_chip->digitalWrite(pins[i], !inverted);
  }
}

void MCP23017SevenSegDisplay::setInverted(bool _inverted)
{
  inverted = _inverted;
}

void MCP23017SevenSegDisplay::setNumber(int _number)
{
  //clearDisplay();
  for(int i = 0; i < 7; i++) {
    if(inverted) {
      mcp_chip->digitalWrite(pins[i], !bitRead(numbersToDisplay[_number],i));
    } else {
      mcp_chip->digitalWrite(pins[i], bitRead(numbersToDisplay[_number],i));
    }
  }
}

void MCP23017SevenSegDisplay::flatLine(void)
{
  setNumber(16);
}

void MCP23017SevenSegDisplay::clearDisplay(void)
{
  for(int i = 0; i < 7; i++) {
    if(inverted) {
      mcp_chip->digitalWrite(pins[i], HIGH);
    } else {
      mcp_chip->digitalWrite(pins[i], LOW);
    }
  }
}
