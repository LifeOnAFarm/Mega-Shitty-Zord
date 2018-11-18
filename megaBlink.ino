/*
 * Description:   Shitty MegaZord code for the blinky-ness and flashy flashy. 
 *                Attiny-84 and 5 PCF8574As were used for this board 
 * Name:          Seamus de Cleir - sdecleir@gmail.com
 * Date:          06/11/2018
 */

// Needed for SoftI2CMaster
#define SDA_PORT PORTA
#define SDA_PIN 6       // = A6 - Pin 7 - SDA
#define SCL_PORT PORTA
#define SCL_PIN 4       // = A4 - Pin 9 - SCL

#define I2C_PULLUP 1    // Helps with the low 3 volts

#include <SoftI2CMaster.h>
#include <PinChangeInterrupt.h>

#define  TORSO  0x3E            /// High High Low - Address of MegaZord's Body
#define  LEFTARM  0x39          /// Low Low High - Address of MegaZord's Left Arm
#define  LEFTLEG  0x3A          /// Low High Low - Address of MegaZord's Left Leg
#define  RIGHTLEG  0x3C         /// High Low Low - Address of MegaZord's Right Leg
#define  RIGHTARM  0x38         /// Low Low Low - Address of MegaZord's Right Arm

#define TORSO_RED B01110101     /// Red LEDS of the MegaZord's Body
#define LEFTARM_RED B11100001   /// Red LEDS of the MegaZord's Left Arm
#define LEFTLEG_RED B01101011   /// Red LEDS of the MegaZord's Left Leg
#define RIGHTLEG_RED B01101011  /// Red LEDS of the MegaZord's Right Leg
#define RIGHTARM_RED B10000111  /// Red LEDS of the MegaZord's Right Arm

// Directions for the lightRun function
#define FORWARD true
#define BACK false

// Button Pin
#define inPin 9

// Variables for lightRun funtion
int ledsOn = 1;
boolean direction = FORWARD;


int boardArray[] = {TORSO, LEFTARM, LEFTLEG, RIGHTLEG, RIGHTARM};   
int boardRed[] = {TORSO_RED, LEFTARM_RED, LEFTLEG_RED, RIGHTLEG_RED, RIGHTARM_RED}; 

int pause = 0;
unsigned long prevMillis;
unsigned long currentMillis;
int reading;
int previous;
unsigned long timeLast;
int stage;
int maxStages = 7;


void setup()
{
  i2c_init();
  allOff();
  pinMode(inPin, INPUT);
}

void loop()
{
  // Reset Timer
  currentMillis = millis();

  // Button attached to pin 9
  reading = digitalRead(inPin);

  // Checks for a button press
  if (reading == HIGH && previous == LOW && millis() - timeLast) {
    
    timeLast = millis();
    
    // Increments the stage
    stage++;
    
    // Reset the stages
    if (stage > maxStages) stage = 0;
  }
  
  stageSelect(stage);
  previous = reading;
  
}


// ***** Functions ***** 

// Controls button presses to select LED sequence
void stageSelect(int stage) {
  
  if (millis() - prevMillis > pause) {
    
    switch (stage) {
      case 0:
        allOn();
        break;
      case 1:
        lightRun();
        break;
      case 2:
        redToYellow(1000);
        break;
      case 3:
        alternate(1000);
        break;
      case 4:
        flasher(1000);
        break;
      case 5:
        allRed();
        break;
      case 6:
        allYellow();
        break;
      case 7:
        sequence(4000);
        break;
    }
    
    prevMillis = millis();
    
  }
} //End stageSelect



// Sends addresses to pins
void transmit(int devAddress, int dataSent){
  i2c_start((devAddress<<1)|I2C_WRITE);
  i2c_write(dataSent);
  i2c_stop();
}// End transmit


// Turns all red LEDs on and then all yellow LEDs
void redToYellow(int speedDelay){
  for (int i = 0; i < 5; i++){
    
    static unsigned long _timer;
    
    if((millis() - _timer) >= 0 && (millis() - _timer) < speedDelay){
      transmit(boardArray[i], boardRed[i]);   // Red LEDs on
    } 
    else if(((millis() - _timer) >= speedDelay) && ((millis() - _timer) < speedDelay*2)){
      transmit(boardArray[i], ~boardRed[i]);  // Yellow LEDs on
    }
    else{
      _timer = millis();
    }
  }
}// End redToYellow


// Alternates every second pin on and off
void alternate(int speedDelay){
  for (int i = 0; i < 5; i++){
    
    static unsigned long _timer;
    
    if((millis() - _timer) >= 0 && (millis() - _timer) < speedDelay){
      transmit(boardArray[i], 0xAA);          // Binary 10101010
    } 
    else if(((millis() - _timer) >= speedDelay) && ((millis() - _timer) < speedDelay*2)){
      transmit(boardArray[i], 0x55);          // // Binary 01010101
    }
    else{
      _timer = millis();
    }
 }
}// End alternate


// Flashes LEDs on and off
void flasher(int speedDelay){
  for (int i = 0; i < 5; i++){
    
    static unsigned long _timer;
    
    if((millis() - _timer ) >= 0 && (millis() - _timer) < speedDelay){
      transmit(boardArray[i], 0x00);
    } 
    else if(((millis() - _timer) >= speedDelay) && ((millis() - _timer) < speedDelay*2)){
      transmit(boardArray[i], 0xFF);
    } 
    else{
      _timer = millis();
    }

    
 }
}// End flasher


// Light Chaser
void lightRun(){
  
  static unsigned long _timer;
  int interval;
    
    for (int i = 0; i < 5; i++){
      
      transmit(boardArray[i], ~ledsOn);

      // Randomizes the speed
      interval = random(50,500);
      
      if ((millis() - _timer) >= interval) {
        
        // Resets the timer 
        _timer = millis();

        // If the direction is forward bit shift to the left
        if(direction == FORWARD){
          
          ledsOn = ledsOn << 1;
          
          if (ledsOn >= 128){
            direction = BACK;
          }
          
        // If the direction is back bit shift to the right
        } else{
          
          ledsOn = ledsOn >> 1;
          
          if (ledsOn == 0) {
            ledsOn = 8;
            direction = FORWARD;
          }
        }
      }
   }
} // End lightRun

// Turns on all Red Leds
void allRed(){
  for (int i = 0; i < 5; i++){
  transmit(boardArray[i], boardRed[i]);
 }
}

// Turns on all Yellow Leds
void allYellow(){
  for (int i = 0; i < 5; i++){
  transmit(boardArray[i], ~boardRed[i]);
 }
}

// Turns all the pins off
void allOff(){
 for (int i = 0; i < 5; i++){
  transmit(boardArray[i], 255);
 }
}// End allOff


// Turns all the pins on
void allOn(){
 for (int i = 0; i < 5; i++){
  transmit(boardArray[i], 0);
 }
}// End allOn

// Uses all light functions
void sequence(int speedDelay){
    
    static unsigned long _timer;
    
    if((millis() - _timer ) >= 0 && (millis() - _timer) < speedDelay){
      lightRun();
    } 
    else if(((millis() - _timer) >= speedDelay) && ((millis() - _timer) < speedDelay*2)){
      alternate(1000);
    } 
    else if(((millis() - _timer) >= speedDelay) && ((millis() - _timer) < speedDelay*3)){
      flasher(1000);
    }
    else if(((millis() - _timer) >= speedDelay) && ((millis() - _timer) < speedDelay*4)){
      redToYellow(1000);
    }
    else{
      _timer = millis();
    }
    
}

