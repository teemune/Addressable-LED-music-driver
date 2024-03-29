#include <FastLED.h>
#include <EEPROM.h>
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

// EEPROM
const unsigned int MODE_SETTING_ADDRESS = 0;
const unsigned int COLOR_INDEX_SETTING_ADDRESS = 4;
const unsigned int BRIGHTNESS_SETTING_ADDRESS = 8;
const unsigned int STRIP_TYPE_SETTING_ADDRESS = 12;

bool modeSettingChanged = LOW;
bool colorSettingChanged = LOW;
bool brightnessSettingChanged = LOW;
bool stripTypeChanged = LOW;

/* Define debugging level */

#define DEBUG_LEVEL 3
#define POST_INTERVAL 300
#define SERIAL_PLOTTER 0
#define POLL_DELAY 50                               // Serial plotter poll delay

/* Serial port */

#define SERIAL_BAUD_RATE 57600                      // 57600 max with internal 8 MHz oscillator
#define READ_BUFFER_SIZE 32

/* FFT */

const int FFT_DATA_SIZE = 128;
const int FFT_NOISE_FLOOR = 4;                      // Anything less than 3 will show some yellow on low f

/* LEDs */

unsigned int FFT_FADING_DELAY = 1;                  // How fast the peaks fade in peak detection mode, bigger value -> less fading
unsigned int FFT_FADING_AMOUNT = 5;
const unsigned int MAX_AMPLITUDE_FADING_FACTOR = 3; // How fast the max amplitude value fades, bigger value -> less fading
const unsigned int LOW_PEAK_SUPPRESSION = 4;        // Suppress the signal from first FFT bin
unsigned int REFRESH_INTERVAL = 1;                 // How often to light up a new LED (ms)
const unsigned int COLOR_SENSITIVITY = 5;           // Overall sensitivity, default 10
const unsigned int RED_SENSITIVITY = 3;             // Sensitivity adjustment for red, default 3
const unsigned int GREEN_SENSITIVITY = 5;           // Sensitivity adjustment for green, default 5
const unsigned int BLUE_SENSITIVITY = 5;            // Sensitivity adjustment for blue, default 5
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

const unsigned int SEGMENT_REFRESH_INTERVAL = 500;  // (ms) Update rate for 7-segment display
const unsigned int NUMBER_OF_MODES = 3;
const unsigned int COLOR_QUANTITY = 21;
const unsigned int MAX_BRIGHTNESS = 10;
const unsigned int LED_STRIP_TYPES_SUPPORTED = 3;

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
const unsigned int BIN_TWO_TH = 4;
const unsigned int BIN_THREE_TH = 8;

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

const int MCP23017_RESET = 30;                      // The pin won't work with Arduino

const int LED_UC_X = 13;                            // Same as debug LED
const int LED_UC_B = 9;
const int LED_UC_D = 10;

const int TIP_SWITCH = 16;

/* Arduino analog */
const int audioSignalIn = 15;

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

uint8_t maxAmplitude = 0;
uint8_t maxAmplitudeFading = 0;

/* FFT */

int8_t FFTdata[FFT_DATA_SIZE], im[FFT_DATA_SIZE];
uint16_t calculatedValueArray[FFT_DATA_SIZE/2];
uint8_t fadingFactorArray[FFT_DATA_SIZE/2];

uint8_t redFadingCounter = 0;
uint8_t greenFadingCounter = 0;
uint8_t blueFadingCounter = 0;

/* LEDs */

CRGB leds[NUM_LEDS];
unsigned long _last_refresh_time = 0;

uint8_t red, green, blue = 0;
uint8_t brightness = 105;
uint8_t brightnessSelection = 5;

bool ledUpdate = LOW;
uint8_t staticColorSelection = 1;

uint8_t stripTypeSelection = 2;

/* 7 segment */

MCP23017SevenSegDisplay sevenSeg[2];
unsigned long _last_7seg_refresh_time = 0;
uint8_t displayPausedSequences = 4;

/* Buttons */

Adafruit_MCP23017 mcp;
Button UIButton[NO_OF_BUTTONS];

/* Other */

// Debugging
unsigned long _last_post_time = 0;
byte cycle_count = 0;

uint8_t operationMode = 1;
uint8_t operationModeSelection = 1;
/***********************************************************************************************************************/
/*                                                   FUNCTIONS                                                         */
/***********************************************************************************************************************/

void byteFFTanalysis();
void machine_state2();

/* Analog in */
float readAmplitude(int _pin);

/* Functions */
byte I2C_command(byte _command);
void readButtons(void);
void setStaticColor(uint8_t _colorIndex);
void set7SegNumber (uint8_t _numberToDisplay);
void set7SegFlatLine(void);

/* FFT */
void FFTsample(void);

/* LED */
//void setLedColor(uint8_t brightness, uint8_t red, uint8_t green, uint8_t blue);
void rotateLeds(void);
void setStripColor(void);
void testLeds(void);
void dimmLedsBy(uint8_t _dimBy);
void brightenLedsBy(uint8_t _brightenBy);
void setLedBrightness(uint8_t _brightness);

/***********************************************************************************************************************/
/*                                                   SETUP LOOP                                                        */
/***********************************************************************************************************************/

void setup() {

  /* Pin settings on Arduino */

  // I2C
  Wire.begin();
  Wire.setClock(100000);
  
  /* Serial communications setup */

#if DEBUG_LEVEL > 1
  Serial.begin(SERIAL_BAUD_RATE);
//  while(!Serial){
//    ;                                               // If you want the code to only start when you open serial monitor
//  }
#endif

  /* LEDs */
  FastLED.addLeds<WS2813, LED_UC_D, GRB>(leds, NUM_LEDS);
  testLeds();

  /* FFT */

  // Zero the averaging / peak detection array
  for (int i = 0; i < FFT_DATA_SIZE/2; i++){
    calculatedValueArray[i] = 0;
    fadingFactorArray[i] = 0;
  }

  /* Pin settings on MCP23017 */

//  // Release from reset, not in use
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
    UIButton[i].setPullup(HIGH);
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

#if DEBUG_LEVEL > 1
  Serial.println(F("Serial port opened"));
  Serial.print(F("Serial baud rate: "));
  Serial.print(SERIAL_BAUD_RATE);
  Serial.println(F(" bit/s"));
#endif

  // Set initial states
  uint8_t _EEPROMsetting = EEPROM.read(MODE_SETTING_ADDRESS);
  if( _EEPROMsetting >= 1 && _EEPROMsetting <= NUMBER_OF_MODES) {
    operationMode = _EEPROMsetting;
    operationModeSelection = _EEPROMsetting;
  }
  _EEPROMsetting = EEPROM.read(COLOR_INDEX_SETTING_ADDRESS);
  if( _EEPROMsetting >= 1 && _EEPROMsetting <= COLOR_QUANTITY) {
    staticColorSelection = _EEPROMsetting;
  }
  _EEPROMsetting = EEPROM.read(BRIGHTNESS_SETTING_ADDRESS);
  if( _EEPROMsetting >= 0 && _EEPROMsetting <= MAX_BRIGHTNESS) {
    brightnessSelection = _EEPROMsetting;
    brightness = brightnessSelection * 25;
    if(brightnessSelection == 10) {
      brightness = 255;
    }
  }
  _EEPROMsetting = EEPROM.read(STRIP_TYPE_SETTING_ADDRESS);
  if( _EEPROMsetting >= 0 && _EEPROMsetting <= LED_STRIP_TYPES_SUPPORTED) {
    stripTypeSelection = _EEPROMsetting;
  }

#if DEBUG_LEVEL > 1
  Serial.print(F("Mode: "));
  Serial.println(operationModeSelection);
  Serial.print(F("Color: "));
  Serial.println(staticColorSelection);
  Serial.print(F("Brightness: "));
  Serial.println(brightnessSelection);
#endif

  /* Setup complete */

#if DEBUG_LEVEL > 1
  Serial.println(F("Setup complete"));
#endif
  
}

/***********************************************************************************************************************/
/*                                                   MAIN LOOP                                                         */
/***********************************************************************************************************************/

void loop() { 

  bool runningLeds = HIGH;
  readButtons();

  // State machine
  switch(operationMode) {
    // Music - running
    case 1:
      FFT_FADING_DELAY = 1;
      FFT_FADING_AMOUNT = 30;
      byteFFTanalysis();
      runningLeds = HIGH;
      ledUpdate = HIGH;
      break;

    // Music - Full strip
    case 2:
      FFT_FADING_DELAY = 20;
      FFT_FADING_AMOUNT = 5;
      byteFFTanalysis();
      runningLeds = LOW;
      ledUpdate = HIGH;
      break;

    // Static Color
    case 3:
      setStaticColor(staticColorSelection);
      //setLedBrightness(brightness);
      ledUpdate = LOW;
      FastLED.show();
      operationMode = 97;
      break;

    // wait for new commands
    case 97:
      break;
    // fault
    case 98:
      break;
    // unkown
    case 99:
      break;
    
    default:
      break;
  }

  if((_last_refresh_time + REFRESH_INTERVAL) < millis() && ledUpdate)
  {
    if(runningLeds) {
      rotateLeds();
    } else {
      setStripColor();
    }
    _last_refresh_time = millis();
  }

  // Update 7-seg display
  // Same function is used for writing EEPROM, as it is called when a certain time
  // has elapsed from last keypress
  if((_last_7seg_refresh_time + SEGMENT_REFRESH_INTERVAL) < millis())
  {
    if(displayPausedSequences < 1) {
      // 7 segment update
      if (operationModeSelection == 1 || operationModeSelection == 2) {
        uint8_t _amplitudeDisplay = (uint8_t)((float)maxAmplitude / 128.0 * 99.0);
        set7SegNumber(_amplitudeDisplay);
      } else {
        set7SegNumber(operationModeSelection);
      }


      // EEPROM
      if(modeSettingChanged) {
        modeSettingChanged = LOW;
        EEPROM.write(MODE_SETTING_ADDRESS, operationModeSelection);
        #if DEBUG_MODE > 1
          Serial.println(F("modeSettingChanged"));
        #endif
      }
      if(colorSettingChanged) {
        colorSettingChanged = LOW;
        EEPROM.write(COLOR_INDEX_SETTING_ADDRESS, staticColorSelection);
        #if DEBUG_MODE > 1
          Serial.println(F("colorSettingChanged"));
        #endif
      }
      if(brightnessSettingChanged) {
        brightnessSettingChanged = LOW;
        EEPROM.write(BRIGHTNESS_SETTING_ADDRESS, brightnessSelection);
        #if DEBUG_MODE > 1
          Serial.println(F("brightnessSettingChanged"));
        #endif
      }
      if(stripTypeChanged) {
        stripTypeChanged = LOW;
        EEPROM.write(STRIP_TYPE_SETTING_ADDRESS, stripTypeSelection);
        #if DEBUG_MODE > 1
          Serial.println(F("stripTypeChanged"));
        #endif
      }
    } else {
      displayPausedSequences--;
    }
    _last_7seg_refresh_time = millis();
  }

// Debugging
  #if DEBUG_LEVEL > 0
  if((_last_post_time + POST_INTERVAL) < millis())
  {
    // Debug info

    #if DEBUG_LEVEL > 4
    Serial.print("LEDs: ");
    Serial.print(red);
    Serial.print(", ");
    Serial.print(green);
    Serial.print(", ");
    Serial.println(blue);
    #endif

    #if DEBUG_LEVEL > 4
    for (int i = 0; i < FFT_DATA_SIZE/2; i++) {
      Serial.println(calculatedValueArray[i]);
    }
    #endif

    _last_post_time = millis();
  }
  #endif
} // void loop()

/***********************************************************************************************************************/
/*                                             STATE MACHINE 1:                                                        */
/***********************************************************************************************************************/

void byteFFTanalysis() {

  uint16_t _redValue = 0;
  uint16_t _greenValue = 0;
  uint16_t _blueValue = 0;
  uint16_t _brightnessValue = 0;

  FFTsample();

  // FFT with 2^7 bins
  if(fix_fft(FFTdata, im, 7, 0) < 0) { // FFT processing
    #if DEBUG_LEVEL > 1
    Serial.println(F("Error in FFT"));
    #endif
    return;
  }

  uint16_t _absoluteValueArray[FFT_DATA_SIZE/2];

  for (int i = BIN_ONE_TH; i < FFT_DATA_SIZE/2; i++) {
    _absoluteValueArray[i] = sqrt((uint32_t)FFTdata[i] * (uint32_t)FFTdata[i] + (uint32_t)im[i] * (uint32_t)im[i]);
  }

  // Suppress the low frequency peak
  if(_absoluteValueArray[0] < LOW_PEAK_SUPPRESSION) {
      _absoluteValueArray[0] = 0;
  } else {
      _absoluteValueArray[0] = _absoluteValueArray[0] - LOW_PEAK_SUPPRESSION;
  }

  #ifdef FFT_PEAK_HOLD
  //  // Peak detection and fading
  //  for (int i = BIN_ONE_TH; i < FFT_DATA_SIZE/2; i++) {
  //    if (calculatedValueArray[i] < _absoluteValueArray[i]) {
  //      calculatedValueArray[i] = _absoluteValueArray[i];
  //      fadingFactorArray[i] = 0;
  //    } 
  //    else if (fadingFactorArray[i] >= FFT_FADING_DELAY) {
  //      if (calculatedValueArray[i] > FFT_FADING_AMOUNT) {
  //        calculatedValueArray[i] -= FFT_FADING_AMOUNT;
  //      } else {
  //        calculatedValueArray[i] = 0;
  //      }
  //      fadingFactorArray[i] = 0;
  //    } else {
  //      fadingFactorArray[i]++;
  //    }
  //  }
  
    // Highest value for red LED
    for (int i = BIN_ONE_TH; i < BIN_TWO_TH; i++) {
      if(_absoluteValueArray[i] > (_redValue + FFT_NOISE_FLOOR)) {
        _redValue = _absoluteValueArray[i] * RED_SENSITIVITY * COLOR_SENSITIVITY;
      }
    }
  
    // Highest value for green LED
    for (int i = BIN_TWO_TH; i < BIN_THREE_TH; i++) {
      if(_absoluteValueArray[i] > (_greenValue + FFT_NOISE_FLOOR)) {
        _greenValue = _absoluteValueArray[i] * GREEN_SENSITIVITY * COLOR_SENSITIVITY;
      }
    }
  
    // Highest value for blue LED
    for (int i = BIN_THREE_TH; i < FFT_DATA_SIZE/2; i++) {
      if(_absoluteValueArray[i] > (_blueValue + FFT_NOISE_FLOOR)) {
        _blueValue = _absoluteValueArray[i] * BLUE_SENSITIVITY * COLOR_SENSITIVITY;
      }
    }
  
    if (_redValue > 255) {
      red = 255;
    } else if(_redValue > red) {
      red = _redValue;
      redFadingCounter = 0;
    } else if(redFadingCounter >= FFT_FADING_DELAY) {
        if (red > FFT_FADING_AMOUNT) {
          red -= FFT_FADING_AMOUNT;
        } else {
          red = 0;
        }
    } else {
      redFadingCounter++;
    }
  
    if (_greenValue > 255) {
      green = 255;
    } else if(_greenValue > green) {
      green = _greenValue;
      greenFadingCounter = 0;
    } else if(greenFadingCounter >= FFT_FADING_DELAY) {
        if (green > FFT_FADING_AMOUNT) {
          green -= FFT_FADING_AMOUNT;
        } else {
          green = 0;
        }
    } else {
      greenFadingCounter++;
    }
    
    if (_blueValue > 255) {
      blue = 255;
    } else if(_blueValue > blue) {
      blue = _blueValue;
      blueFadingCounter = 0;
    } else if(blueFadingCounter >= FFT_FADING_DELAY) {
        if (blue > FFT_FADING_AMOUNT) {
          blue -= FFT_FADING_AMOUNT;
        } else {
          blue = 0;
        }
    } else {
      blueFadingCounter++;
    }
  #else
    // No processing
    for (int i = BIN_ONE_TH; i < FFT_DATA_SIZE/2; i++) {
      calculatedValueArray[i] = _absoluteValueArray[i];  
    }
    
    //Integrate over full spectrum
    // Value for red LED
    for (int i = BIN_ONE_TH; i < BIN_TWO_TH; i++) {
      if(calculatedValueArray[i] > FFT_NOISE_FLOOR) {
        _redValue += calculatedValueArray[i];
      }
    }
  
    // Value for green LED
    for (int i = BIN_TWO_TH; i < BIN_THREE_TH; i++) {
      if(calculatedValueArray[i] > FFT_NOISE_FLOOR) {
        _greenValue += calculatedValueArray[i];
      }
    }
  
    // Value for blue LED
    for (int i = BIN_THREE_TH; i < FFT_DATA_SIZE/2; i++) {
      if(calculatedValueArray[i] > FFT_NOISE_FLOOR) {
        _blueValue += calculatedValueArray[i];
      }
    }
  
    if (_redValue < 255 ) {
      red = _redValue;
    } else {
      red = 255;
    }
    
    if (_greenValue < 255 ) {
      green = _greenValue;
    } else {
      green = 255;
    }
    
    if (_blueValue < 255 ) {
      blue = _blueValue;
    } else {
      blue = 255;
    }
  #endif

  #ifdef AMPLITUDE_DETECTION
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
  
  // Button 0 - Mode up
  if(UIButton[0].pressed())
  {
    modeSettingChanged = HIGH;
    #if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(0);
    #endif
    if(operationModeSelection < NUMBER_OF_MODES) {
      operationModeSelection++;  
    }
    operationMode = operationModeSelection;
    set7SegNumber(operationMode);
    // Display a demo to visualize selection
    if(operationModeSelection == 1 || operationModeSelection == 2) {
      red = 90;
    }
    #if DEBUG_LEVEL > 1
      Serial.print(F("Operation mode: "));
      Serial.println(operationMode);
    #endif
    displayPausedSequences = 8;
    delay(250);
  }
  
  // Button 1 - Mode down
  if(UIButton[1].pressed())
  {
    modeSettingChanged = HIGH;
    #if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(1);
    #endif
    if(operationModeSelection > 1) {
      operationModeSelection--;  
    }
    operationMode = operationModeSelection;
    set7SegNumber(operationMode);
    if(operationModeSelection == 1 || operationModeSelection == 2) {
      red = 90;
    }
    #if DEBUG_LEVEL > 1
      Serial.print(F("Operation mode: "));
      Serial.println(operationMode);
    #endif
    displayPausedSequences = 8;
    delay(250);
  }
  
  // Button 2 - Color up
  if(UIButton[2].pressed())
  {
    colorSettingChanged = HIGH;
    #if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(2);
    #endif
    if(operationModeSelection == 3 || operationModeSelection == 4) {
      operationMode = operationModeSelection;
      if(staticColorSelection < COLOR_QUANTITY) {
        staticColorSelection++;
      }
      set7SegNumber(staticColorSelection);
      #if DEBUG_LEVEL > 1
        Serial.print(F("Color: "));
        Serial.println(staticColorSelection);
      #endif
      displayPausedSequences = 8;
    } else {
      set7SegFlatLine();
      displayPausedSequences = 2;
    }
    delay(250);
  }
  
  // Button 3 - Color down
  if(UIButton[3].pressed())
  {
    colorSettingChanged = HIGH;
    #if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(3);
    #endif
    if(operationModeSelection == 3 || operationModeSelection == 4) {
      operationMode = operationModeSelection;
      if(staticColorSelection > 1) {
        staticColorSelection--;
      }
      set7SegNumber(staticColorSelection);
      #if DEBUG_LEVEL > 1
        Serial.print(F("Color: "));
        Serial.println(staticColorSelection);
      #endif
      displayPausedSequences = 8;
    } else {
      set7SegFlatLine();
      displayPausedSequences = 2;
    }
    delay(250);
  }
  
  // Button 4 - Brightness up
  if(UIButton[4].pressed())
  {
    brightnessSettingChanged = HIGH;
    #if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(4);
    #endif
    if(operationModeSelection == 3 || operationModeSelection == 4) {
      if(brightnessSelection < MAX_BRIGHTNESS) {
        brightnessSelection++;
        brightness = brightnessSelection * 25;
      }
      if(brightnessSelection == 10) {
        brightness = 255;
      }
      // Update LEDs to change brightness
      operationMode = operationModeSelection;
      set7SegNumber(brightnessSelection);
      #if DEBUG_LEVEL > 1
        Serial.print(F("Brightness: "));
        Serial.println(brightness);
      #endif
      displayPausedSequences = 8;
    } else {
      set7SegFlatLine();
      displayPausedSequences = 2;
    }
    delay(250);
  }

  // Button 5 - Brightness down
  if(UIButton[5].pressed())
  {
    brightnessSettingChanged = HIGH;
    #if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(5);
    #endif
    if(operationModeSelection == 3 || operationModeSelection == 4) {
      if(brightnessSelection > 0) {
        brightnessSelection--;
        brightness = brightnessSelection * 25;
        // Update LEDs to change brightness
        operationMode = operationModeSelection;
      }
      set7SegNumber(brightnessSelection);
      #if DEBUG_LEVEL > 1
        Serial.print(F("Brightness: "));
        Serial.println(brightness);
      #endif
      displayPausedSequences = 8;
    } else {
      displayPausedSequences = 2;
      set7SegFlatLine();
    }
    delay(250);
  }
  // Button 6
  if(UIButton[6].pressed())
  {
    stripTypeChanged = HIGH;
#if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(6);
#endif
    if(stripTypeSelection < LED_STRIP_TYPES_SUPPORTED) {
      stripTypeSelection++;  
    }
    changeStripType(stripTypeSelection);
    operationMode = operationModeSelection;
    displayPausedSequences = 8;
    delay(250);
  }
  // Button 7
  if(UIButton[7].pressed())
  {
    stripTypeChanged = HIGH;
    #if DEBUG_LEVEL > 1
      Serial.print(F("Pressed button: "));
      Serial.println(7);
    #endif
    if(stripTypeSelection > 1) {
      stripTypeSelection--;  
    }
    changeStripType(stripTypeSelection);
    operationMode = operationModeSelection;
    displayPausedSequences = 8;
    delay(250);
  }
}

void FFTsample(void) {

  int rawResults[FFT_DATA_SIZE];

  maxAmplitude = 0;

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
//    Serial.print("LEDs: ");
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

void setStripColor() {
  for(int i = NUM_LEDS - 1; i >= 0; i--){
    leds[i] = CRGB(red, green, blue);
  }
  FastLED.show();
}

void setStaticColor(uint8_t _colorIndex) {
  switch(_colorIndex) {
    
    case 1:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Indigo;
      }
    break;

    case 2:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Magenta;
      }
    break;

    case 3:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Brown;
      }
    break;

    case 4:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::DeepPink;
      }
    break;

    case 5:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Crimson;
      }
    break;

    case 6:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Red;
      }
    break;

    case 7:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Chocolate;
      }
    break;

    case 8:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::OrangeRed;
      }
    break;

    case 9:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Orange;
      }
    break;

    case 10:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Gold;
      }
    break;

    case 11:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::GreenYellow;
      }
    break;

    case 12:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Chartreuse;
      }
    break;
    
    case 13:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Lime;
      }
    break;

    case 14:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Turquoise;
      }
    break;

    case 15:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Blue;
      }
    break;

    case 16:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::MediumAquamarine;
      }
    break;

    case 17:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::MediumPurple;
      }
    break;

    case 18:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::IndianRed;
      }
    break;

    case 19:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::HotPink;
      }
    break;

    case 20:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::SandyBrown;
      }
    break;

    case 21:
      for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::White;
      }
    break;
    
    default:
    for(int i = NUM_LEDS - 1; i >= 0; i--){
      leds[i] = CRGB::Black;
      }
    break;
  }
  // Set brightenss
  for(int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i].fadeToBlackBy(255 - brightness);
    //leds[i].fadeLightBy(255 - brightness);
    
  }
  return;
}

void dimmLedsBy(uint8_t _dimBy) {
  for(int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i].fadeToBlackBy(_dimBy);
  }
  FastLED.show();
}

void brightenLedsBy(uint8_t _brightenBy) {
  for(int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i].fadeToBlackBy(_brightenBy);
    leds[i].maximizeBrightness();
  }
  FastLED.show();
}

void setLedBrightness(uint8_t _brightness) {
  for(int i = NUM_LEDS - 1; i >= 0; i--) {
    leds[i].maximizeBrightness();
    leds[i].fadeToBlackBy(_brightness);
  }
  FastLED.show();
}

void testLeds() {
  // Check the LEDs
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(25, 0, 0);
  }
  FastLED.show();
  delay(150);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(0, 25, 0);
  }
  FastLED.show();
  delay(150);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(0, 0, 25);
  }
  FastLED.show();
  delay(150);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(15, 15, 15);
  }
  FastLED.show();
  delay(150);
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

void set7SegNumber(uint8_t _numberToDisplay) {
  uint8_t _leftNumber, _rightNumber = 0;

  _leftNumber = _numberToDisplay / 10;
  _rightNumber = _numberToDisplay % 10;
  
  sevenSeg[0].setNumber(_leftNumber);
  sevenSeg[1].setNumber(_rightNumber);
}

void set7SegFlatLine() {
  sevenSeg[0].flatLine();
  sevenSeg[1].flatLine();
  displayPausedSequences = 2;
}

// For testing purposes
// Not very functional as the inital configuration in the beginning is missing
void changeStripType(uint8_t _stripTypeSelection) {
  switch(_stripTypeSelection) {
  case 1:
    set7SegNumber(12);
    FastLED.addLeds<WS2812, LED_UC_D, GRB>(leds, NUM_LEDS);
    testLeds();
    break;

  case 2:
    set7SegNumber(13);
    FastLED.addLeds<WS2813, LED_UC_D, GRB>(leds, NUM_LEDS);
    testLeds();
    break;

  case 3:
    set7SegNumber(15);
    FastLED.addLeds<WS2812, LED_UC_D, GRB>(leds, NUM_LEDS);
    testLeds();
    break;

  default:
    break;
  }
}
