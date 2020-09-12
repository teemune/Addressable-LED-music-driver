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
#define ADC_PRESCALE_32                             // 32 = ~40 kHz sampling
//#define ADC_PRESCALE_64                             // ~20 kHz sampling

// I2C
#define I2C_ADDR 0x20
#define REG_HIGH_BYTE 0x00
#define REG_LOW_BYTE 0xA3

/* Define debugging level */

#define DEBUG_LEVEL 1
#define POST_INTERVAL 300
#define SERIAL_PLOTTER 0
#define POLL_DELAY 50                               // Serial plotter poll delay

/* Serial port */

#define SERIAL_BAUD_RATE 57600                      // 57600 max with internal 8 MHz oscillator
#define READ_BUFFER_SIZE 32

/* FFT */

const int FFT_DATA_SIZE = 128;
const int FFT_NOISE_FLOOR = 3;                      // Anything less than 3 will show some yellow on low f

/* LEDs */

const unsigned int FFT_FADING_DELAY = 1;           // How fast the peaks fade in peak detection mode, bigger value -> less fading
const unsigned int FFT_FADING_AMOUNT = 5;
const unsigned int MAX_AMPLITUDE_FADING_FACTOR = 3; // How fast the max amplitude value fades, bigger value -> less fading
const unsigned int LOW_PEAK_SUPPRESSION = 4;        // Suppress the signal from first FFT bin
const unsigned int REFRESH_INTERVAL = 10;           // How often to light up a new LED (ms)
const unsigned int COLOR_SENSITIVITY = 4;          // Overall sensitivity
const unsigned int RED_SENSITIVITY = 1;             // Sensitivity adjustment for red
const unsigned int GREEN_SENSITIVITY = 5;           // Sensitivity adjustment for green
const unsigned int BLUE_SENSITIVITY = 9;           // Sensitivity adjustment for blue
const unsigned int NUM_LEDS = 100;

const unsigned int BIN_ONE_TH_HZ = 0;               // Frequencies below this will be ignored
const unsigned int BIN_TWO_TH_HZ = 400;             // f below this will light red leds, above green
const unsigned int BIN_THREE_TH_HZ = 800;           // f above this will light blue leds

/* Effects */

// React to large amplitude signals, even if the event is too short for FFT
// Triggers very often with music
//#define AMPLITUDE_DETECTION

// Hold peaks in FFT spectrum, makes the effect less flashy
// Can have adverse effects in noisy environment
#define FFT_PEAK_HOLD

/* Other */

//#ifdef ADC_PRESCALE_32
//  const unsigned int hzPerBin = 35000 / FFT_DATA_SIZE;
//#endif
//
//#ifdef ADC_PRESCALE_64
//  const unsigned int hzPerBin = 17000 / FFT_DATA_SIZE;
//#endif

//const unsigned int BIN_ONE_TH = BIN_ONE_TH_HZ / hzPerBin;
//const unsigned int BIN_TWO_TH = BIN_TWO_TH_HZ / hzPerBin;
//const unsigned int BIN_THREE_TH = BIN_THREE_TH_HZ / hzPerBin;

const unsigned int BIN_ONE_TH = 0;
const unsigned int BIN_TWO_TH = 2;
const unsigned int BIN_THREE_TH = 7;

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

/* ADC */

uint16_t maxAmplitude = 0;
uint8_t maxAmplitudeFading = 0;

/* FFT */

int8_t FFTdata[FFT_DATA_SIZE], im[FFT_DATA_SIZE];
uint16_t calculatedValueArray[FFT_DATA_SIZE/2];
uint8_t fadingFactorArray[FFT_DATA_SIZE/2];

/* LEDs */

CRGB leds[NUM_LEDS];
unsigned long _last_refresh_time = 0;

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
uint8_t colorMap(uint32_t _value);

/* FFT */

void FFTsample(void);

/* LED */
//void setLedColor(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);
void rotateLeds();

/***********************************************************************************************************************/
/*                                                   SETUP LOOP                                                        */
/***********************************************************************************************************************/

void setup() {

  /* Pin settings on Arduino */

  // I2C
  Wire.begin();
  Wire.setClock(100000);
  
  /* Serial communications setup */
  
  Serial.begin(SERIAL_BAUD_RATE);
//  while(!Serial){
//    ;                                               // If you want the code to only start when you open serial monitor
//  }
  
  /* LEDs */
  FastLED.addLeds<WS2813, LED_UC_D, GRB>(leds, NUM_LEDS);

  // Check the LEDs
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(25, 0, 0);
  }
  FastLED.show();
  delay(300);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(0, 25, 0);
  }
  FastLED.show();
  delay(300);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(0, 0, 25);
  }
  FastLED.show();
  delay(300);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(15, 15, 15);
  }
  FastLED.show();
  delay(300);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();

  /* FFT */

  // Zero the averaging / peak detection array
  for (int i = 0; i < FFT_DATA_SIZE/2; i++){
    calculatedValueArray[i] = 0;
    fadingFactorArray[i] = 0;
  }

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

  /* ADC */

  analogReference(DEFAULT);

  // ADC conversion takes 14.5 clock cycles
  
  // ADC prescaler 101 -> 32 prescale -> 35 kHz sample rate
  // ADC clock 500 kHz
#ifdef ADC_PRESCALE_32
  bitWrite(ADCSRA,2,1);
  bitWrite(ADCSRA,1,1);
  bitWrite(ADCSRA,0,0);
#endif

  // ADC prescaler 110 -> 64 prescale -> 17 kHz sample rate
  // ADC clock 250 kHz
#ifdef ADC_PRESCALE_64
  bitWrite(ADCSRA,2,1);
  bitWrite(ADCSRA,1,1);
  bitWrite(ADCSRA,0,0);
#endif

#if DEBUG_LEVEL > 2
  Serial.println(F("Serial port opened"));
  Serial.print(F("Serial baud rate: "));
  Serial.print(SERIAL_BAUD_RATE);
  Serial.println(F(" bit/s"));
#endif

  /* Setup complete */

#if DEBUG_LEVEL > 2
  Serial.println(F("Setup complete"));
#endif
  
}

/***********************************************************************************************************************/
/*                                                   MAIN LOOP                                                         */
/***********************************************************************************************************************/

void loop() { 

  // State machine
  machine_state = music;
  
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

  if((_last_refresh_time + REFRESH_INTERVAL) < millis())
  {
    // Causes distortion in analog signal with max gain
    rotateLeds();
    _last_refresh_time = millis();
  }

// Debugging
#if DEBUG_LEVEL > 0
  if((_last_post_time + POST_INTERVAL) < millis())
  {
    // Debug info

//#if DEBUG_LEVEL > 4
//    Serial.print("LEDs: ");
//    Serial.print(red);
//    Serial.print(", ");
//    Serial.print(green);
//    Serial.print(", ");
//    Serial.println(blue);
//#endif

//#if DEBUG_LEVEL > 3
//    for (int i = 0; i < FFT_DATA_SIZE/2; i++) {
//      Serial.println(calculatedValueArray[i]);
//    }
//#endif

    _last_post_time = millis();
//    cycle_count++;
//    Serial.println("------");
  }
#endif
} // void loop()

/***********************************************************************************************************************/
/*                                             STATE MACHINE 1:                                                        */
/***********************************************************************************************************************/

void machine_state1() {

  uint16_t _redValue = 0;
  uint16_t _greenValue = 0;
  uint16_t _blueValue = 0;
  uint16_t _brightnessValue = 0;

  FFTsample();

  // FFT with 2^7 bins
  if(fix_fft(FFTdata, im, 7, 0) < 0) { // FFT processing
#if DEBUG_LEVEL > 4
    Serial.println(F("Error in FFT"));
#endif
    return;
  }

  uint16_t _absoluteValueArray[FFT_DATA_SIZE/2];
//  uint8_t _SignalLevel = 0;
//  uint8_t _highestBin = 129;

  for (int i = BIN_ONE_TH; i < FFT_DATA_SIZE/2; i++) {
    _absoluteValueArray[i] = sqrt((long)FFTdata[i] * (long)FFTdata[i] + (long)im[i] * (long)im[i]);
    //_absoluteValueArray[i] = (long)FFTdata[i] * (long)FFTdata[i] + (long)im[i] * (long)im[i];
  }

  // Suppress the low frequency peak
  if(_absoluteValueArray[0] < LOW_PEAK_SUPPRESSION) {
      _absoluteValueArray[0] = 0;
  } else {
      _absoluteValueArray[0] = _absoluteValueArray[0] - LOW_PEAK_SUPPRESSION;
  }

#ifdef FFT_PEAK_HOLD
  // Peak detection and fading
  for (int i = BIN_ONE_TH; i < FFT_DATA_SIZE/2; i++) {
    if (calculatedValueArray[i] < _absoluteValueArray[i]) {
      calculatedValueArray[i] = _absoluteValueArray[i];
      fadingFactorArray[i] = 0;
    } 
    else if (fadingFactorArray[i] >= FFT_FADING_DELAY) {
      if (calculatedValueArray[i] > FFT_FADING_AMOUNT) {
        calculatedValueArray[i] -= FFT_FADING_AMOUNT;
      } else {
        calculatedValueArray[i] = 0;
      }
      fadingFactorArray[i] = 0;
    } else {
      fadingFactorArray[i]++;
    }
  }
#endif

//  // No processing
//  for (int i = BIN_ONE_TH; i < FFT_DATA_SIZE/2; i++) {
//    calculatedValueArray[i] = _absoluteValueArray[i];  
//  } 

//  //Integrate over full spectrum
//  // Value for red LED
//  for (int i = BIN_ONE_TH; i < BIN_TWO_TH; i++) {
//    if(calculatedValueArray[i] > FFT_NOISE_FLOOR) {
//      _redValue += calculatedValueArray[i];
//    }
//  }
//
//  // Value for green LED
//  for (int i = BIN_TWO_TH; i < BIN_THREE_TH; i++) {
//    if(calculatedValueArray[i] > FFT_NOISE_FLOOR) {
//      _greenValue += calculatedValueArray[i];
//    }
//  }
//
//  // Value for blue LED
//  for (int i = BIN_THREE_TH; i < FFT_DATA_SIZE/2; i++) {
//    if(calculatedValueArray[i] > FFT_NOISE_FLOOR) {
//      _blueValue += calculatedValueArray[i];
//    }
//  }

  // Value for red LED
  for (int i = BIN_ONE_TH; i < BIN_TWO_TH; i++) {
    if(calculatedValueArray[i] > (_redValue + FFT_NOISE_FLOOR)) {
      _redValue = calculatedValueArray[i] * RED_SENSITIVITY * COLOR_SENSITIVITY;
    }
  }

  // Value for green LED
  for (int i = BIN_TWO_TH; i < BIN_THREE_TH; i++) {
    if(calculatedValueArray[i] > (_greenValue + FFT_NOISE_FLOOR)) {
      _greenValue = calculatedValueArray[i] * GREEN_SENSITIVITY * COLOR_SENSITIVITY;
    }
  }

  // Value for blue LED
  for (int i = BIN_THREE_TH; i < FFT_DATA_SIZE/2; i++) {
    if(calculatedValueArray[i] > (_blueValue + FFT_NOISE_FLOOR)) {
      _blueValue = calculatedValueArray[i] * BLUE_SENSITIVITY * COLOR_SENSITIVITY;
    }
  }

  red = _redValue;
  green = _greenValue;
  blue = _blueValue;

#ifdef AMPLITUDE_DETECTION
Serial.println("DISABLED");
  if (maxAmplitudeFading >= MAX_AMPLITUDE_FADING_FACTOR) {
    if (maxAmplitude > 10) {
        maxAmplitude -= 10;
      } else {
        maxAmplitude = 0;
      }
      maxAmplitudeFading = 0;
    } else {
      maxAmplitudeFading++;
    }

  if (((red + blue + green) < 5) && (maxAmplitude > 10)) {
    red += maxAmplitude*2;
    green += maxAmplitude*2;
    blue += maxAmplitude*2;
  }
#endif
    
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
#if DEBUG_LEVEL > 3
    Serial.print(F("Pressed button: "));
    Serial.println(0);
#endif
    //sevenSeg[0].setNumber(0);
    sevenSeg[1].setNumber(0);
    for(int j=0;j<NUM_LEDS;j++){
      leds[j] = CRGB::Red;
    }
    FastLED.show();
  }
  if(UIButton[1].pressed())
  {
#if DEBUG_LEVEL > 3
    Serial.print(F("Pressed button: "));
    Serial.println(1);
#endif

    int _absoluteValueArray[FFT_DATA_SIZE];

    for (int i = 0; i < FFT_DATA_SIZE; i++) {
      _absoluteValueArray[i] = sqrt((long)FFTdata[i] * (long)FFTdata[i] + (long)im[i] * (long)im[i]);
    }

#if DEBUG_LEVEL > 1
    for (int i = 0; i < FFT_DATA_SIZE/2; i++) {
      
/* Formatting for plotter */
Serial.println(_absoluteValueArray[i]);

///* Formatting for serial monitor */
//      Serial.print(i);
//      Serial.print(": ");
//      Serial.print(_absoluteValueArray[i]);
//      Serial.print(", ");
//      delay(10);

    }
//    Serial.println("");
#endif

    sevenSeg[1].setNumber(1);

    FastLED.show();
  }
  if(UIButton[2].pressed())
  {
#if DEBUG_LEVEL > 3
    Serial.print(F("Pressed button: "));
    Serial.println(2);
#endif
    //sevenSeg[0].setNumber(2);
    sevenSeg[1].setNumber(2);
    for(int j=0;j<NUM_LEDS;j++){
      leds[j] = CRGB::Blue;
    }
    FastLED.show();
  }
  if(UIButton[3].pressed())
  {
#if DEBUG_LEVEL > 3
    Serial.print(F("Pressed button: "));
    Serial.println(3);
#endif
    //sevenSeg[0].setNumber(3);
    sevenSeg[1].setNumber(3);
    for(int j=0;j<NUM_LEDS;j++){
      leds[j] = CRGB::White;
    }
    FastLED.show();
  }
  if(UIButton[4].pressed())
  {
#if DEBUG_LEVEL > 3
    Serial.print(F("Pressed button: "));
    Serial.println(4);
#endif
    sevenSeg[1].setNumber(4);
    for(int j=0;j<NUM_LEDS;j++){
      leds[j] = CRGB::Yellow;
      delay(10);
      FastLED.show();
    }
  }
  if(UIButton[5].pressed())
  {
#if DEBUG_LEVEL > 3
    Serial.print(F("Pressed button: "));
    Serial.println(5);
#endif
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
    if (abs(FFTdata[i]) > maxAmplitude) {
      maxAmplitude = abs(FFTdata[i]);
    }
  }
}

void rotateLeds() {

  for(int i = NUM_LEDS - 1; i >= 1; i--){
        leds[i] = leds[i-1];
      }
      
#if DEBUG_LEVEL > 4
//  Serial.print("LEDs: ");
  Serial.print(red);
  Serial.print(", ");
  Serial.print(green);
  Serial.print(", ");
  Serial.print(blue);
  Serial.print(", ");
  Serial.println(maxAmplitude);
#endif

  leds[0] = CRGB(red, green, blue);
  
  FastLED.show();
}

uint8_t colorMap(uint32_t _value) {
  return 100;
}
