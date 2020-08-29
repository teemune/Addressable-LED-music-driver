#include <FastLED.h>
#include <Wire.h>
#include <fix_fft.h>
#include "Adafruit_MCP23017.h"
#include "KD_ardu_button.h"
#include "KD_MCP23017Button.h"
#include "KD_MCP23017_7seg.h"

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
#define POST_INTERVAL 200
#define SERIAL_PLOTTER 0
#define POLL_DELAY 50                               // Serial plotter poll delay

/* Serial port */

#define SERIAL_BAUD_RATE 57600                      // 57600 max with internal 8 MHz oscillator
#define READ_BUFFER_SIZE 32

/* LEDs */

const unsigned int NUM_LEDS = 100;

/* FFT */

const int FFT_DATA_SIZE = 128;

/***********************************************************************************************************************/
/*                                                  pin definitions                                                    */
/***********************************************************************************************************************/

/* Arduino digital */

const int bufferedSW1 = 6;
const int bufferedSW2 = 4;
const int bufferedSW3 = 12;
const int bufferedSW4 = 11;
const int bufferedSW5 = 7;
const int bufferedSW6 = 23;
const int bufferedSW7 = 22;
const int bufferedSW8 = 21;

const int MSGEQ7_STROBE = 5;
const int MSGEQ7_RESET = 8;

const int MCP23017_RESET = 30;                      // The pin won't work with Arduino

const int LED_UC_X = 13;                            // Same as debug LED
const int LED_UC_B = 9;
const int LED_UC_D = 10;

const int TIP_SWITCH = 16;

const int debugLED = 13;

/* Arduino analog */
const int audioSignalIn = 15;
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

/* FFT */

int8_t FFTdata[FFT_DATA_SIZE], im[FFT_DATA_SIZE];

/* LEDs */

CRGB leds[NUM_LEDS];

uint8_t brightness, red, green, blue = 0;

/* 7 segment */

MCP23017SevenSegDisplay sevenSeg[2];

/* Buttons */

Adafruit_MCP23017 mcp;

Button UIButton[NO_OF_BUTTONS];

/* Other */

// Debugging
unsigned long _last_post_time = 0;
byte cycle_count = 0;

// enum type
enum machine_state_list {
  music,
  amplitude,
  off,
  fault,
  unknown
};

machine_state_list machine_state = unknown;

/***********************************************************************************************************************/
/*                                                   FUNCTIONS                                                         */
/***********************************************************************************************************************/

void machine_state1();
void machine_state2();

/* Analog in */
float readAmplitude(int _pin);

/* Functions */
byte I2C_command(byte _command);
void readButtons(void);

/* FFT */

void FFTsample(void);

/* LED */
void setLedColor(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);
void rotateLeds();

/***********************************************************************************************************************/
/*                                                   SETUP LOOP                                                        */
/***********************************************************************************************************************/

void setup() {
  
  /* Serial communications setup */
  
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial){
    ;                                               // Needed for Atmega32U4
  }
  
  // LEDs
  FastLED.addLeds<WS2813, LED_UC_D, GRB>(leds, NUM_LEDS);

  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();

  // ADC
  analogReference(DEFAULT);

  /* Pin settings on Arduino */

  // LEDs
  pinMode(debugLED, OUTPUT);
  digitalWrite(debugLED, LOW);

  // I2C
  Wire.begin();
  Wire.setClock(100000);
  
  /* Pin settings on MCP23017 */

  // Release from reset, not in use
//  pinMode(MCP23017_RESET, OUTPUT);
//  digitalWrite(MCP23017_RESET, HIGH);

  mcp.begin();

  sevenSeg[0].init(&mcp,7,6,5,4,3,2,1,0);
  sevenSeg[1].init(&mcp,15,14,13,12,11,10,9,8);
  sevenSeg[0].setInverted(HIGH);
  sevenSeg[1].setInverted(HIGH);

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
    UIButton[i].setInverted(HIGH);
  }

  // ADC prescaler 101 -> 32 prescale -> 40 kHz sample rate
  // ADC prescaler 110 -> 64 prescale -> 20 kHz sample rate
  bitWrite(ADCSRA,2,1);
  bitWrite(ADCSRA,1,1);
  bitWrite(ADCSRA,0,0);

//#if DEBUG_LEVEL > 1
//  Serial.println(F("Serial port opened"));
//  Serial.print(F("Serial baud rate: "));
//  Serial.print(SERIAL_BAUD_RATE);
//  Serial.println(F(" bit/s"));
//#endif

  /* Setup complete */

  // Delay with blinking
//  for(int i = 0; i < 5; i++) {
//    digitalWrite(debugLED, HIGH);
//    delay(100);
//    digitalWrite(debugLED, LOW);
//    delay(100);
//  }

//#if DEBUG_LEVEL > 1
//  Serial.println(F("Setup complete"));
//#endif
  
}

/***********************************************************************************************************************/
/*                                                   MAIN LOOP                                                         */
/***********************************************************************************************************************/

void loop() { 

  // State machine
  machine_state = amplitude;
  
  switch(machine_state) {
    case music:
      machine_state1();
      break;

    case amplitude:
      machine_state2();
      break;

    case fault:
      break;

    case unknown:
      break;
    
    default:
      break;
  }

  readButtons();

// Debugging
#if DEBUG_LEVEL > 4
  if((_last_post_time + POST_INTERVAL) < millis())
  {
    // Debug info
//    Serial.println(F("debugging info:"));
//    Serial.print(F("Cycle: "));
//    Serial.println(cycle_count);
   
    rotateLeds();

    _last_post_time = millis();
    cycle_count++;
//    Serial.println("------");
  }
#endif
} // void loop()

/***********************************************************************************************************************/
/*                                             STATE MACHINE 1:                                                        */
/***********************************************************************************************************************/

void machine_state1() {

  int _resultArray[FFT_DATA_SIZE];

  FFTsample();

  Serial.println(100);
  Serial.println(0);
  Serial.println(0);
  Serial.println(0);
  for (int i = 0; i < FFT_DATA_SIZE; i++) {
    Serial.println(FFTdata[i]);
  }

  if(fix_fft(FFTdata, im, 7, 0) < 0) { // FFT processing
    Serial.println(F("Error in FFT"));
    return;
  }

  long _doubleArray[FFT_DATA_SIZE];

  for (int i = 0; i < FFT_DATA_SIZE; i++) {
    _doubleArray[i] = sqrt((long)FFTdata[i] * (long)FFTdata[i] + (long)im[i] * (long)im[i]);
  }
  
  Serial.println(0);
  Serial.println(0);
  Serial.println(0);
  Serial.println(0);
  Serial.println(0);
  Serial.println(0);
  for (int i = 0; i < FFT_DATA_SIZE; i++) {
    Serial.println(_doubleArray[i]);
  }
  Serial.println(0);
  Serial.println(0);
  Serial.println(0);
  Serial.println(100);
  
//  for (i = 0; i < 8; i++) {
//    // Average values
//    j = i << 3;
//    data_avgs[i] = data[j] + data[j + 1] + data[j + 2] + data[j + 3]
//      + data[j + 4] + data[j + 5] + data[j + 6] + data[j + 7];
//    if (i == 0)
//      data_avgs[i] >>= 1;  // KK: De-emphasize first audio band (too sensitive)
//    data_avgs[i] = map(data_avgs[i], 0, maxExpectedAudio, 0, 7); // Map for output to 8x8 display
//  }

//  red += 5;
//  green -= 5;
//  blue += 10;
//  brightness = 5;
//
//  Serial.println(red);
//  Serial.println(green);
//  Serial.println(blue);
//  Serial.println(brightness);
    
  return;
}

/***********************************************************************************************************************/
/*                                             STATE MACHINE 2:                                                        */
/***********************************************************************************************************************/

void machine_state2() {
  return;
}

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
float readAmplitude(int _pin) {
  float _result = 0;
  _result = AUX_VREF/ADC_CHANNELS * analogRead(_pin) ;
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

void readButtons() {

    if(UIButton[0].pressed())
    {
      Serial.print(F("Pressed button: "));
      Serial.println(0);
      //sevenSeg[0].setNumber(0);
      sevenSeg[1].setNumber(0);
      for(int j=0;j<NUM_LEDS;j++){
        leds[j] = CRGB::Red;
      }
    }
    if(UIButton[1].pressed())
    {
        machine_state1();

//      Serial.print(F("Pressed button: "));
//      Serial.println(1);
//      //sevenSeg[0].setNumber(1);
//      sevenSeg[1].setNumber(1);
//      for(int j=0;j<NUM_LEDS;j++){
//        leds[j] = CRGB::Green;
//      }
    }
    if(UIButton[2].pressed())
    {
      Serial.print(F("Pressed button: "));
      Serial.println(2);
      //sevenSeg[0].setNumber(2);
      sevenSeg[1].setNumber(2);
      for(int j=0;j<NUM_LEDS;j++){
        leds[j] = CRGB::Blue;
      }
    }
    if(UIButton[3].pressed())
    {
      Serial.print(F("Pressed button: "));
      Serial.println(3);
      //sevenSeg[0].setNumber(3);
      sevenSeg[1].setNumber(3);
      for(int j=0;j<NUM_LEDS;j++){
        leds[j] = CRGB::White;
      }
    }
    if(UIButton[4].pressed())
    {
      Serial.print(F("Pressed button: "));
      Serial.println(4);
      sevenSeg[1].setNumber(4);
      for(int j=0;j<NUM_LEDS;j++){
        leds[j] = CRGB::Yellow;
        delay(10);
        FastLED.show();
      }
    }
    if(UIButton[5].pressed())
    {
      Serial.print(F("Pressed button: "));
      Serial.println(5);
      //sevenSeg[0].setNumber(i);
      sevenSeg[1].setNumber(5);
      int _dimming_value = 100;
      for(int k=0;k<60;k++) {
        for(int j=0;j<NUM_LEDS;j++){
        //leds[j] = CRGB::Black;
        //leds[j].nscale8_video(100 - 10 * j);
        leds[j].fadeToBlackBy(k);
        }
        FastLED.show();
        delay(10);
      }
    }
    FastLED.show();
}

void FFTsample(void) {

  int rawResults[FFT_DATA_SIZE];

  for (int i = 0; i < FFT_DATA_SIZE; i++) {
    rawResults[i] = 0;
    im[i] = 0;
  }

  for (int i = 0; i < FFT_DATA_SIZE; i++) {
    rawResults[i] = analogRead(A1);
  }

  // This could be combined with earlier to save up the space of rawResults[FFT_DATA_SIZE]
  // With a small cost to sample rate
  for (int i = 0; i < FFT_DATA_SIZE; i++) {
    FFTdata[i] = map(rawResults[i], 0, 1023, 0, 255) - 128;
  }
}

void setLedColorBuffer(uint8_t _brightness, uint8_t _red, uint8_t _green, uint8_t _blue) {
  brightness = _brightness;
  red = _red;
  green = _green;
  blue = _blue;
}

void rotateLeds() {

  for(int i = NUM_LEDS - 1; i >= 1; i--){
        leds[i] = leds[i-1];
      }
  
  leds[0] = CRGB(red, green, blue);
  leds[0].fadeLightBy(brightness);
  
  FastLED.show();
}
