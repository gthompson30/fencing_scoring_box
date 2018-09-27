//===========================================================================//
//                                                                           //
//  Desc:    Arduino Code to implement a fencing scoring apparatus           //
//  Dev:     Wnew                                                            //
//  Date:    Nov  2012                                                       //
//  Updated: Sept 2015                                                       //
//  Notes:   1. Basis of algorithm from digitalwestie on github. Thanks Mate //
//           2. Used uint8_t instead of int where possible to optimise       //
//           3. Set ADC prescaler to 16 faster ADC reads                     //
//                                                                           //
//  To do:   1. Could use shift reg on lights and mode LEDs to save pins     //
//           2. Implement short circuit LEDs (already provision for it)      //
//           3. Set up debug levels correctly                                //
//                                                                           //
//===========================================================================//

// Fencers are refered to as Green and Red. Green on the left and red on the right.
// The pins of the connectors are refered to as A-B--C
// Where A is: the return path for Epee, the Lame for Foil and Sabre
// Where B is: live for all three weapons, it connects to the blade (there is debate whether A or B is used in Sabre) 
// Where C is: ground for both Epee, Foil and Sabre. Connected to the guard of all 3 weapons.
// The order of the weapons is in alphabetical order, epee, foil sabre.

//============
// #includes
//============
#include <Adafruit_NeoPixel.h>  // adafruit's library for neopixel displays

//============
// #defines
//============
//TODO: set up debug levels correctly
#define DEBUG 0
//#define TEST_LIGHTS       // turns on lights for a second on start up
//#define TEST_ADC_SPEED    // used to test sample rate of ADCs
//#define REPORT_TIMING     // prints timings over serial interface
#define NEOPIXELS         // if this is set then sketch uses the neopixel display, if not then individual leds per pin are assumed.
#define BUZZERTIME  1000  // length of time the buzzer is kept on after a hit (ms)
#define LIGHTTIME   3000  // length of time the lights are kept on after a hit (ms)
#define BAUDRATE   57600  // baudrate of the serial debug interface

#define LED_PIN        8  // neopixels data pin
#define NUMPIXELS     40  // number of NeoPixels on display


// initialise the neopixel class
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

//============
// Pin Setup
//============
//const uint8_t shortLEDGrn  =  8;    // Short Circuit A Light
const uint8_t onTargetGrn  =  9;    // On Target A Light
const uint8_t offTargetGrn = 10;    // Off Target A Light
const uint8_t offTargetRed = 11;    // Off Target B Light
const uint8_t onTargetRed  = 12;    // On Target B Light
const uint8_t shortLEDRed  = 13;    // Short Circuit A Light

// The micro only has 4 analog pins
//const uint8_t groundPinGrn = A0;    // Ground A pin - Analog
const uint8_t weaponPinGrn = A2;    // Weapon A pin - Analog
const uint8_t lamePinGrn   = A3;    // Lame   A pin - Analog (Epee return path)
const uint8_t lamePinRed   = A0;    // Lame   B pin - Analog (Epee return path)
const uint8_t weaponPinRed = A1;    // Weapon B pin - Analog
//const uint8_t groundPinRed = A5;    // Ground B pin - Analog

const uint8_t modePin    =  2;        // Mode change button interrupt pin 0 (digital pin 2)
const uint8_t buzzerPin  =  3;        // buzzer pin
const uint8_t modeLeds[] = {4, 5, 6}; // LED pins to indicate weapon mode selected {e f s}

//=========================
// values of analog reads
//=========================
int grnA = 0;
int redA = 0;
int grnB = 0;
int redB = 0;
int grnC = 0;
int redC = 0;

//=======================
// depress and timeouts
//=======================
long depressGrnTime = 0;
long depressRedTime = 0;
bool lockedOut      = false;

//==========================
// Lockout & Depress Times
//==========================
// the lockout time between hits for foil is 300ms +/-25ms
// the minimum amount of time the tip needs to be depressed for foil 14ms +/-1ms
// the lockout time between hits for epee is 45ms +/-5ms (40ms -> 50ms)
// the minimum amount of time the tip needs to be depressed for epee 2ms
// the lockout time between hits for sabre is 120ms +/-10ms
// the minimum amount of time the tip needs to be depressed (in contact) for sabre 0.1ms -> 1ms
// These values are stored as micro seconds for more accuracy
//                         foil   epee   sabre
const long lockout [] = {300000,  45000, 120000};  // the lockout time between hits
const long depress [] = { 14000,   2000,   1000};  // the minimum amount of time the tip needs to be depressed



//=================
// mode constants
//=================
const uint8_t EPEE_MODE  = 0;
const uint8_t FOIL_MODE  = 1;
const uint8_t SABRE_MODE = 2;

uint8_t currentMode = FOIL_MODE;

bool modeJustChangedFlag = false;

//=========
// states
//=========
boolean depressedGrn  = false;
boolean depressedRed  = false;
boolean hitOnTargGrn  = false;
boolean hitOffTargGrn = false;
boolean hitOnTargRed  = false;
boolean hitOffTargRed = false;

#ifdef TEST_ADC_SPEED
long now;
long loopCount = 0;
bool done = false;
#endif


//================
// Configuration
//================
void setup() {

   Serial.begin(BAUDRATE);
   while (!Serial);
   Serial.println("3 Weapon Scoring Box");
   Serial.println("====================");
   Serial.print  ("Mode : ");
   Serial.println(currentMode);

   // set the internal pullup resistor on modePin
   pinMode(modePin, INPUT_PULLUP);

   // add the interrupt to the mode pin (pin 2 is interrupt0 on the Uno and interrupt1 on the Micro)
   // change to modePin-2 for the Uno
   attachInterrupt(modePin-1, changeMode, FALLING);
   pinMode(modeLeds[0], OUTPUT);
   pinMode(modeLeds[1], OUTPUT);
   pinMode(modeLeds[2], OUTPUT);

   // set the light pins to outputs
   pinMode(offTargetGrn, OUTPUT);
   pinMode(offTargetRed, OUTPUT);
   pinMode(onTargetGrn,  OUTPUT);
   pinMode(onTargetRed,  OUTPUT);
   //pinMode(shortLEDGrn,  OUTPUT);
   pinMode(shortLEDRed,  OUTPUT);
   pinMode(buzzerPin,  OUTPUT);

   digitalWrite(modeLeds[currentMode], HIGH);

   // initialise the LED display
   pixels.begin();

#ifdef TEST_LIGHTS
   testLights();
#endif

   // this optimises the ADC to make the sampling rate quicker
   //adcOpt();

   setModeLeds();
}


//=============
// ADC config
//=============
void adcOpt() {

   // the ADC only needs a couple of bits, the atmega is an 8 bit micro
   // so sampling only 8 bits makes the values easy/quicker to process
   // unfortunately this method only works on the Due.
   //analogReadResolution(8);

   // Data Input Disable Register
   // disconnects the digital inputs from which ever ADC channels you are using
   // an analog input will be float and cause the digital input to constantly
   // toggle high and low, this creates noise near the ADC, and uses extra 
   // power Secondly, the digital input and associated DIDR switch have a
   // capacitance associated with them which will slow down your input signal
   // if you’re sampling a highly resistive load 
   DIDR0 = 0x7F;

   // set the prescaler for the ADCs to 16 this allowes the fastest sampling
   bitClear(ADCSRA, ADPS0);
   bitClear(ADCSRA, ADPS1);
   bitSet  (ADCSRA, ADPS2);
}


//============
// Main Loop
//============
void loop() {
   // use a while as a main loop as the loop() has too much overhead for fast analogReads
   // we get a 3-4% speed up on the loop this way
   while(1) {
      checkIfModeChanged();
      // read analog pins
      grnA = analogRead(weaponPinGrn);
      redA = analogRead(weaponPinRed);
      grnB = analogRead(lamePinGrn);
      redB = analogRead(lamePinRed);
      signalHits();
      if      (currentMode == EPEE_MODE)
         epee();
      else if (currentMode == FOIL_MODE)
         foil();
      else if (currentMode == SABRE_MODE)
         sabre();

#ifdef TEST_ADC_SPEED
      if (loopCount == 0) {
         now = micros();
      }
      loopCount++;
      if ((micros()-now >= 1000000) && done == false) {
         Serial.print(loopCount);
         Serial.println(" readings in 1 sec");
         done = true;
      }
#endif
   }
}

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

//=====================
// Mode pin interrupt
//=====================
void changeMode() {
   Serial.println("Button Pressed");
   if ((millis() - lastDebounceTime) >  debounceDelay && digitalRead(modePin) == LOW) {
      modeJustChangedFlag = true;
   } else {
      lastDebounceTime = millis();
      modeJustChangedFlag = false;
   }
}


//============================
// Sets the correct mode led
//============================
void setModeLeds() {
   if (currentMode == EPEE_MODE) {
      ledEpee();
      digitalWrite(onTargetGrn, HIGH);
   } else {
      if (currentMode == FOIL_MODE) {
         ledFoil();
         digitalWrite(onTargetRed, HIGH);
      } else {
         if (currentMode == SABRE_MODE){
            ledSabre();
            digitalWrite(onTargetGrn, HIGH);
            digitalWrite(onTargetRed, HIGH);
         }
      }
   }
   long now = millis();
   while (millis() < now + 1000) {}
   //delay(500);
   digitalWrite(onTargetGrn, LOW);
   digitalWrite(onTargetRed, LOW);
   clear();
}


//========================
// Run when mode changed
//========================
void checkIfModeChanged() {
   if (modeJustChangedFlag) {
      if (currentMode == 2)
         currentMode = 0;
      else
         currentMode++;

      setModeLeds();
      modeJustChangedFlag = false;
#ifdef DEBUG
      Serial.print("Mode changed to: ");
      Serial.println(currentMode);
#endif
   }
}


//===================
// Main foil method
//===================
void foil() {

   long now = micros();
   if (((hitOnTargGrn || hitOffTargGrn) && (depressGrnTime + lockout[0] < now)) || 
       ((hitOnTargRed || hitOffTargRed) && (depressRedTime + lockout[0] < now))) {
      lockedOut = true;
   }

   // weapon Grn
   if (hitOnTargGrn == false && hitOffTargGrn == false) { // ignore if Grn has already hit
      // off target
      if (900 < grnA && redB < 100) {
         if (!depressedGrn) {
            depressGrnTime = micros();
            depressedGrn   = true;
         } else {
            if (depressGrnTime + depress[0] <= micros()) {
               hitOffTargGrn = true;
            }
         }
      } else {
      // on target
         if (400 < grnA && grnA < 600 && 400 < redB && redB < 600) {
            if (!depressedGrn) {
               depressGrnTime = micros();
               depressedGrn   = true;
            } else {
               if (depressGrnTime + depress[0] <= micros()) {
                  hitOnTargGrn = true;
               }
            }
         } else {
            // reset these values if the depress time is short.
            depressGrnTime = 0;
            depressedGrn   = 0;
         }
      }
   }

   // weapon Red
   if (hitOnTargRed == false && hitOffTargRed == false) { // ignore if Red has already hit
      // off target
      if (900 < redA && grnB < 100) {
         if (!depressedRed) {
            depressRedTime = micros();
            depressedRed   = true;
         } else {
            if (depressRedTime + depress[0] <= micros()) {
               hitOffTargRed = true;
            }
         }
      } else {
      // on target
         if (400 < redA && redA < 600 && 400 < grnB && grnB < 600) {
            if (!depressedRed) {
               depressRedTime = micros();
               depressedRed   = true;
            } else {
               if (depressRedTime + depress[0] <= micros()) {
                  hitOnTargRed = true;
               }
            }
         } else {
            // reset these values if the depress time is short.
            depressRedTime = 0;
            depressedRed   = 0;
         }
      }
   }
}


//===================
// Main epee method
//===================
void epee() {
   long now = micros();
   if ((hitOnTargGrn && (depressGrnTime + lockout[1] < now)) || (hitOnTargRed && (depressRedTime + lockout[1] < now))) {
      lockedOut = true;
   }

   // weapon Grn
   //  no hit for Grn yet    && weapon depress    && opponent lame touched
   if (hitOnTargGrn == false) {
      if (400 < grnA && grnA < 600 && 400 < grnB && grnB < 600) {
         if (!depressedGrn) {
            depressGrnTime = micros();
            depressedGrn   = true;
         } else {
            if (depressGrnTime + depress[1] <= micros()) {
               hitOnTargGrn = true;
            }
         }
      } else {
         // reset these values if the depress time is short.
         if (depressedGrn == true) {
            depressGrnTime = 0;
            depressedGrn   = 0;
         }
      }
   }

   // weapon Red
   //  no hit for Red yet    && weapon depress    && opponent lame touched
   if (hitOnTargRed == false) {
      if (400 < redA && redA < 600 && 400 < redB && redB < 600) {
         if (!depressedRed) {
            depressRedTime = micros();
            depressedRed   = true;
         } else {
            if (depressRedTime + depress[1] <= micros()) {
               hitOnTargRed = true;
            }
         }
      } else {
         // reset these values if the depress time is short.
         if (depressedRed == true) {
            depressRedTime = 0;
            depressedRed   = 0;
         }
      }
   }
}


//===================
// Main sabre method
//===================
void sabre() {

   long now = micros();
   if (((hitOnTargGrn || hitOffTargGrn) && (depressGrnTime + lockout[2] < now)) || 
       ((hitOnTargRed || hitOffTargRed) && (depressRedTime + lockout[2] < now))) {
      lockedOut = true;
   }

   // weapon Grn
   if (hitOnTargGrn == false && hitOffTargGrn == false) { // ignore if Grn has already hit
      // on target
      if (400 < grnA && grnA < 600 && 400 < redB && redB < 600) {
         if (!depressedGrn) {
            depressGrnTime = micros();
            depressedGrn   = true;
         } else {
            if (depressGrnTime + depress[2] <= micros()) {
               hitOnTargGrn = true;
            }
         }
      } else {
         // reset these values if the depress time is short.
         depressGrnTime = 0;
         depressedGrn   = 0;
      }
   }

   // weapon Red
   if (hitOnTargRed == false && hitOffTargRed == false) { // ignore if Red has already hit
      // on target
      if (400 < redA && redA < 600 && 400 < grnB && grnB < 600) {
         if (!depressedRed) {
            depressRedTime = micros();
            depressedRed   = true;
         } else {
            if (depressRedTime + depress[2] <= micros()) {
               hitOnTargRed = true;
            }
         }
      } else {
         // reset these values if the depress time is short.
         depressRedTime = 0;
         depressedRed   = 0;
      }
   }
}


//==============
// Signal Hits
//==============
void signalHits() {
   // non time critical, this is run after a hit has been detected
   if (lockedOut) {
      if (hitOnTargGrn) {
         ledOnTargGrn();
      }
      if (hitOffTargGrn) {
         ledOffTargGrn();
      }
      if (hitOffTargRed) {
         ledOffTargRed();
      }
      if (hitOnTargRed) {
         ledOnTargRed();
      }
      digitalWrite(onTargetGrn,  hitOnTargGrn);
      digitalWrite(offTargetGrn, hitOffTargGrn);
      digitalWrite(offTargetRed, hitOffTargRed);
      digitalWrite(onTargetRed,  hitOnTargRed);
      digitalWrite(buzzerPin,  HIGH);
#ifdef DEBUG
      String serData = String("hitOnTargGrn  : ") + hitOnTargGrn  + "\n"
                            + "hitOffTargGrn : "  + hitOffTargGrn + "\n"
                            + "hitOffTargRed : "  + hitOffTargRed + "\n"
                            + "hitOnTargRed  : "  + hitOnTargRed  + "\n"
                            + "Locked Out  : "  + lockedOut   + "\n";
      Serial.println(serData);
#endif
      resetValues();
   }
}


//======================
// Reset all variables
//======================
void resetValues() {
   long now = millis();
   while (millis() < now + BUZZERTIME);        // wait before turning off the buzzer
   digitalWrite(buzzerPin,  LOW);
   while (millis() < now + LIGHTTIME-BUZZERTIME);
   //delay(LIGHTTIME-BUZZERTIME);   // wait before turning off the lights
   digitalWrite(onTargetGrn,  LOW);
   digitalWrite(offTargetGrn, LOW);
   digitalWrite(offTargetRed, LOW);
   digitalWrite(onTargetRed,  LOW);
   //digitalWrite(shortLEDGrn,  LOW);
   digitalWrite(shortLEDRed,  LOW);
   clear();

   lockedOut      = false;
   depressGrnTime = 0;
   depressedGrn   = false;
   depressRedTime = 0;
   depressedRed   = false;

   hitOnTargGrn  = false;
   hitOffTargGrn = false;
   hitOnTargRed  = false;
   hitOffTargRed = false;

   delay(100);
}


//==============
// Test lights
//==============
void testLights() {
   ledOnTargGrn();
   ledOnTargRed();
   digitalWrite(onTargetGrn,  HIGH);
   digitalWrite(onTargetRed, HIGH);
   delay(1000);
   clear();
   ledOffTargGrn();
   ledOffTargRed();
   digitalWrite(offTargetGrn, HIGH);
   digitalWrite(offTargetRed,  HIGH);
   delay(1000);
   clear();
   ledShortGrn();
   ledShortRed();
   //digitalWrite(shortLEDGrn,  HIGH);
   digitalWrite(shortLEDRed,  HIGH);
   delay(1000);
   clear();
   resetValues();
}

void clear() {
  for(int i=0;i<8;i++){
    for(int j=0;j<9;j++){
      pixels.setPixelColor(j*8+i, pixels.Color(0,0,0));
    }
  }
  pixels.show();
}

void ledEpee() {
   pixels.setPixelColor( 2, pixels.Color(23,33,178));
   pixels.setPixelColor( 3, pixels.Color(23,33,178));
   pixels.setPixelColor( 4, pixels.Color(23,33,178));
   pixels.setPixelColor(10, pixels.Color(23,33,178));
   pixels.setPixelColor(18, pixels.Color(23,33,178));
   pixels.setPixelColor(19, pixels.Color(23,33,178));
   pixels.setPixelColor(26, pixels.Color(23,33,178));
   pixels.setPixelColor(34, pixels.Color(23,33,178));
   pixels.setPixelColor(35, pixels.Color(23,33,178));
   pixels.setPixelColor(36, pixels.Color(23,33,178));
   pixels.show();
}

void ledFoil() { 
   pixels.setPixelColor( 2, pixels.Color(23,33,178));
   pixels.setPixelColor( 3, pixels.Color(23,33,178));
   pixels.setPixelColor( 4, pixels.Color(23,33,178));
   pixels.setPixelColor(10, pixels.Color(23,33,178));
   pixels.setPixelColor(18, pixels.Color(23,33,178));
   pixels.setPixelColor(19, pixels.Color(23,33,178));
   pixels.setPixelColor(26, pixels.Color(23,33,178));
   pixels.setPixelColor(34, pixels.Color(23,33,178));
   pixels.show();
}

void ledSabre() { 
   pixels.setPixelColor( 2, pixels.Color(23,33,178));
   pixels.setPixelColor( 3, pixels.Color(23,33,178));
   pixels.setPixelColor( 4, pixels.Color(23,33,178));
   pixels.setPixelColor(10, pixels.Color(23,33,178));
   pixels.setPixelColor(18, pixels.Color(23,33,178));
   pixels.setPixelColor(19, pixels.Color(23,33,178));
   pixels.setPixelColor(20, pixels.Color(23,33,178));
   pixels.setPixelColor(28, pixels.Color(23,33,178));
   pixels.setPixelColor(34, pixels.Color(23,33,178));
   pixels.setPixelColor(35, pixels.Color(23,33,178));
   pixels.setPixelColor(36, pixels.Color(23,33,178));
   pixels.show();
}

void ledOnTargGrn() {
  for(int i=4;i<8;i++){
    for(int j=0;j<9;j++){
      pixels.setPixelColor(j*8+i, pixels.Color(50,0,0));
    }
  }
  pixels.show();
}

void ledOnTargRed() {
  for(int i=0;i<4;i++){
    for(int j=0;j<9;j++){
      pixels.setPixelColor(j*8+i, pixels.Color(0,50,0));
    }
  }
  pixels.show();
}

void ledOffTargGrn() {
  for(int i=5;i<6;i++){
    for(int j=0;j<9;j++){
      pixels.setPixelColor(j*8+i, pixels.Color(50,50,50));
    }
  }
  pixels.setPixelColor(14, pixels.Color(50,50,50));
  pixels.setPixelColor(22, pixels.Color(50,50,50));
  pixels.setPixelColor(23, pixels.Color(50,50,50));
  pixels.setPixelColor(30, pixels.Color(50,50,50));
  pixels.show();
}

void ledOffTargRed() {
  for(int i=2;i<3;i++){
    for(int j=0;j<9;j++){
      pixels.setPixelColor(j*8+i, pixels.Color(50,50,50));
    }
  }
  pixels.setPixelColor( 9, pixels.Color(50,50,50));
  pixels.setPixelColor(16, pixels.Color(50,50,50));
  pixels.setPixelColor(17, pixels.Color(50,50,50));
  pixels.setPixelColor(25, pixels.Color(50,50,50));
  pixels.show();
}

void ledShortGrn() {
  for(int i=0;i<2;i++){
    for(int j=3;j<5;j++){
      pixels.setPixelColor(j*8+i, pixels.Color(100,50,0));
    }
  }
  pixels.show();
}

void ledShortRed() {
  for(int i=6;i<8;i++){
    for(int j=3;j<5;j++){
      pixels.setPixelColor(j*8+i, pixels.Color(100,50,0));
    }
  }
  pixels.show();
}