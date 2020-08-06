#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include "KD_ardu_button.h"

/* Hardware configuration */

// HW
#define BUTTON_DEBOUNCE 20                          // button debounce (ms)
#define NO_OF_BUTTONS 8

// ADC
#define AUX_VREF 2.56
#define ADC_CHANNELS 1024

// I2C
#define I2C_ADDR 0x20
#define REG_HIGH_BYTE 0x00
#define REG_LOW_BYTE 0xA3

/* Define debugging level */

#define DEBUG_LEVEL 5
#define POST_INTERVAL 5000
#define SERIAL_PLOTTER 0
#define POLL_DELAY 50                               // Serial plotter poll delay

/* Serial port */

#define SERIAL_BAUD_RATE 57600                      // 57600 max with internal 8 MHz oscillator
#define READ_BUFFER_SIZE 32

/***********************************************************************************************************************/
/*                                                  pin definitions                                                    */
/***********************************************************************************************************************/

class I2C_MomentaryButton 
  {
  private:
    Adafruit_MCP23017 *_mcp_chip;
    int _button_pin;
    int _command;
    bool _state_old;
    bool _state;
    int _button_illumination_pin;
    
  public: 
    I2C_MomentaryButton(Adafruit_MCP23017 *mcp_chip, int buttonpin, int command);
    I2C_MomentaryButton(void);
    int init(void);
    int readstate(void);
    int sendstate(void);
};

  I2C_MomentaryButton::I2C_MomentaryButton(Adafruit_MCP23017 *mcp_chip, int buttonpin, int command)
  {
    _button_pin = buttonpin;
    _button_illumination_pin = 0;
    _command = command;
    _state_old = HIGH;
    _state = LOW;
    _state_old = LOW;
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

/* Arduino digital */
const int bufferedSW1 = 6;
const int bufferedSW2 = 4;
const int bufferedSW3 = 12;
const int bufferedSW4 = 11;
const int bufferedSW5 = 7;
const int bufferedSW6 = 19;
const int bufferedSW7 = 18;
const int bufferedSW8 = 17;

const int MSGEQ7_STROBE = 5;
const int MSGEQ7_RESET = 8;

// const int MCP23017_RESET = 0;                    // The pin won't work with Arduino

const int LED_UC_X = 13;                            // Same as debug LED
const int LED_UC_B = 10;
const int LED_UC_D = 9;

const int TIP_SWITCH = 16;

const int debugLED = 13;

/* Arduino analog */
const int audioLevel = 15;
const int MSGEQ7analogOut = 14;

/* MCP23017 */

// GPIO A:
//const unsigned char mcp_A0 = 0;                   // Not in use
const unsigned char segG1 = 0;
const unsigned char segF1 = 1;
const unsigned char segE1 = 2;
const unsigned char segD1 = 3;
const unsigned char segC1 = 4;
const unsigned char segB1 = 5;
const unsigned char segA1 = 6;

// GPIO B:
//const unsigned char mcp_B0 = 8;                   // Not in use
const unsigned char segG2 = 9;
const unsigned char segF2 = 10;
const unsigned char segE2 = 11;
const unsigned char segD2 = 12;
const unsigned char segC2 = 13;
const unsigned char segB2 = 14;
const unsigned char segA2 = 15;

/***********************************************************************************************************************/
/*                                                   VARIABLES                                                         */
/***********************************************************************************************************************/

/* Other variables */

Adafruit_MCP23017 mcp;

I2C_MomentaryButton X_AXIS_SELECT_BUTTON = I2C_MomentaryButton(&mcp, 8, 177);

// pin, pullup, inverted
//Button button1(bufferedSW1, LOW, HIGH);
//Button button1(bufferedSW2, LOW, HIGH);
//Button button1(bufferedSW3, LOW, HIGH);
//Button button1(bufferedSW4, LOW, HIGH);
//Button button1(bufferedSW5, LOW, HIGH);
//Button button1(bufferedSW6, LOW, HIGH);
//Button button1(bufferedSW7, LOW, HIGH);
//Button button1(bufferedSW8, LOW, HIGH);

Button UIButton[NO_OF_BUTTONS];

bool b_pressed[NO_OF_BUTTONS];

// Debugging
unsigned long _last_post_time = 0;
byte cycle_count = 0;

// enum type
enum machine_state_list {
  music,
  generic,
  off,
  fault,
  unknown
};

machine_state_list machine_state = unknown;

/***********************************************************************************************************************/
/*                                                   FUNCTIONS                                                         */
/***********************************************************************************************************************/

void machine_state1();

/* Analog in */
float read_voltage(int);

/* Functions */
byte I2C_command(byte _command);

/***********************************************************************************************************************/
/*                                                   SETUP LOOP                                                        */
/***********************************************************************************************************************/

void setup() {

  // ADC
  analogReference(INTERNAL);

  /* Pin settings on Arduino */

  // LEDs
  pinMode(debugLED, OUTPUT);
  digitalWrite(debugLED, LOW);

  // I2C
  Wire.begin();
  Wire.setClock(100000);
  
  /* Pin settings on MCP23017 */

  mcp.begin();

  // Buttons
  UIButton[0].setPin(bufferedSW1);
  UIButton[1].setPin(bufferedSW2);
  UIButton[2].setPin(bufferedSW3);
  UIButton[3].setPin(bufferedSW4);
  UIButton[4].setPin(bufferedSW5);
  UIButton[5].setPin(bufferedSW6);
  UIButton[6].setPin(bufferedSW7);
  UIButton[7].setPin(bufferedSW8);
  
  for (int i = 0; i < NO_OF_BUTTONS; i++){
    b_pressed[i] = false;
  }

  for (int i = 0; i < NO_OF_BUTTONS; i++){
    UIButton[i].setPullup(HIGH);
    UIButton[i].setInverted(HIGH);
  }
  
  X_AXIS_SELECT_BUTTON.init();

  /* Serial communications setup */
  
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial){
    ;                                               // Needed for Atmega32U4
  }  

#if DEBUG_LEVEL > 1
  Serial.println(F("Serial port opened"));
  Serial.print(F("Serial baud rate: "));
  Serial.print(SERIAL_BAUD_RATE);
  Serial.println(F(" bit/s"));
#endif

  /* Setup complete */

  // Delay with blinking
  for(int i = 0; i < 5; i++) {
    digitalWrite(debugLED, HIGH);
    delay(100);
    digitalWrite(debugLED, LOW);
    delay(100);
  }

#if DEBUG_LEVEL > 1
  Serial.println(F("Setup complete"));
#endif
  
}

/***********************************************************************************************************************/
/*                                                   MAIN LOOP                                                         */
/***********************************************************************************************************************/

void loop() { 

  // State machine
  machine_state = generic;
  
  switch(machine_state) {
    case music:
      machine_state1();
      break;

    case generic:
      break;

    case fault:
      break;

    case unknown:
      break;
    
    default:
      break;
  }

// Debugging
#if DEBUG_LEVEL > 4
    if((_last_post_time + POST_INTERVAL) < millis())
    {
      // Debug info
      Serial.println(F("debugging info:"));
      Serial.print(F("Cycle: "));
      Serial.println(cycle_count);

      _last_post_time = millis();
      cycle_count++;
      Serial.println("------");
  }
#endif
} // void loop()

/***********************************************************************************************************************/
/*                                             STATE MACHINE 1:                                                        */
/***********************************************************************************************************************/

void machine_state1() {
  if(UIButton[0].pressed()){
    return;
  } else {
    return;
  }
}

/***********************************************************************************************************************/
/*                                             STATE MACHINE 2:                                                        */
/***********************************************************************************************************************/

/***********************************************************************************************************************/
/*                                             STATE MACHINE fault                                                     */
/***********************************************************************************************************************/

/***********************************************************************************************************************/
/*                                             STATE MACHINE unknown                                                   */
/***********************************************************************************************************************/

/***********************************************************************************************************************/
/*                                                   FUNCTIONS                                                         */
/***********************************************************************************************************************/

/* Analog in */
float read_voltage(int _pin) {
  float _result = 0;
  _result = AUX_VREF * analogRead(_pin) / ADC_CHANNELS;
  return _result;
}

/* I2C */

byte I2C_command(byte _command) {
  byte _reading = 0x00;
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(byte(REG_HIGH_BYTE));
  Wire.write(byte(REG_LOW_BYTE));
  Wire.write(_command);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDR, 1);
  if (Wire.available()) {
    _reading = Wire.read();
  }
  return _reading;
}
