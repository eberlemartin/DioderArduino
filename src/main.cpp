/****************************************
* Arduino Dioder von IKEA Hack
*
* PIC Ports (Removed) but visible on PCB:
*         _______
*    +5V  |  o  | GND
*    Buzz |     | Red
*    1k pd|     | Blue
*    Btn  |     | Green
*         -------
*
* Arduino:      Dioder:       PIR:
* 9     ->      Green
* 10    ->      Blue
* 11    ->      Red
* +5V   ->      +5V
* GND   ->      GND
* 3     ->      Button
* 2                   ->      DATA
* +5V                 ->      VCC
* GND                 ->      GND
*
  TODO: Fix confusion between lastStateSwitch & lastMovementDetected! + overrun protection
  TODO: Mixed up motion detection of Fade and Off
  TODO: Brightness Fade for turning off and on (and switching states)
  TODO: Celan up Code in general
*****************************************/

#include "Arduino.h"

#define GREEN_PIN 9
#define BLUE_PIN 10
#define RED_PIN 11
#define FADER_TIME 10
#define BUTTON_PIN 3
#define PIR_PIN 2

enum MachineState { fade, red, green, blue, white, off };
MachineState arduState, nextArduState = fade;

unsigned long lastStateSwitch = 0;
unsigned long lastMovementDetected = 0;
unsigned long lastButtonPress = 0;
unsigned int rgbColor[3];
boolean movement = false;
boolean lightOn = false;
boolean manual = true;

void setGreen(unsigned int g){
  analogWrite(GREEN_PIN, g);
}

void setBlue(unsigned int b){
  analogWrite(BLUE_PIN, b);
}

void setRed(unsigned int r){
  analogWrite(RED_PIN, r);
}

void setRGB(unsigned int r, unsigned int g, unsigned int b){
  setRed(r);
  setGreen(g);
  setBlue(b);
}

void buttonPressed(){
  Serial.println("Button pressed, switching to next State");
  if(millis() - lastButtonPress >= 1000){
    lastButtonPress = millis();
    manual = true;
    //switch state to which on next iteration of loop we will jump
    switch (arduState) {
      case fade: nextArduState = red; break;
      case red: nextArduState = green; break;
      case green: nextArduState = blue; break;
      case blue: nextArduState = white; break;
      case white: nextArduState = off; break;
      case off: nextArduState = fade; break;
    }
  }
}

void motionDetected(){
  //only switch state if light is previously off
  movement = true;
  lastMovementDetected = millis();
  if(!lightOn && !manual){
    nextArduState = fade;
  }
}

void setup(){
  Serial.begin(115200);

  Serial.println("Setup");

  //config light pins
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);

  //set pin of PIR Sensor to input
  pinMode(PIR_PIN, INPUT);

  //interrupt for the Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), motionDetected, RISING); //set up interrupt attachInterrupt(digital...(pin), ISR, MODE)

  //set all lights to 0 (OFF!)
  Serial.println("All off");
  analogWrite(GREEN_PIN, 0);
  analogWrite(BLUE_PIN, 0);
  analogWrite(RED_PIN, 0);

  //Setup color fader
  rgbColor[0] = 255;
  rgbColor[1] = 0;
  rgbColor[2] = 0;
}

void loop(){
  Serial.println("Loop start");

  //PIR stuff
  if(movement){
    if(digitalRead(PIR_PIN) == 1){
      movement = true;
      Serial.println("Movement detected");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      movement = false;
      digitalWrite(LED_BUILTIN, LOW);
    }
  }

  if(!movement && !manual && lightOn && millis() - lastStateSwitch >= (30*1000)){   //turn off lights after 30 seconds
    Serial.println("Time over, preparing to switch off");
    nextArduState = off;
  }

  if(lastStateSwitch > millis()){
    Serial.println("Overrun in millis() corrected");
    lastStateSwitch = millis();  //replace time, in case overrun happens before reset
  }

  //only change State if previous State is more than 1 Second old to avoid Bouncing from switch
  if((arduState != nextArduState)){
    lastStateSwitch = millis();
    arduState = nextArduState;
    Serial.println("Switching ArduState");
  }

  switch (arduState)
  {
    case fade:
      Serial.println("State: Fade");
      lightOn = true;

      //Choose colors to increment and decrement
      for(int decColor = 0; decColor < 3; decColor++){
        int incColor = decColor == 2 ? 0 : decColor + 1;

        //cross fade the two colors
        for(int i = 0; i < 255; i++){
          rgbColor[decColor]--;
          rgbColor[incColor]++;

          setRGB(rgbColor[0], rgbColor[1], rgbColor[2]);
          delay(FADER_TIME);
        }
      }
      break;

    case red:
      Serial.println("State: Red");
      lightOn = true;
      setRGB(255,0,0);
      break;

    case green:
      Serial.println("State: Green");
      lightOn = true;
      setRGB(0,255,0);
      break;

    case blue:
      Serial.println("State: Blue");
      lightOn = true;
      setRGB(0,0,255);
      break;

    case white:
      Serial.println("State: White");
      lightOn = true;
      setRGB(255,255,255);
      break;

    case off:
	//just to commit
      Serial.println("State: Off");
      //shows light is off
      lightOn = false;
      //enables PIR
      manual = false;
      setRGB(0,0,0);
      break;
  }

  //make debug readable
  //delay(100);
}
