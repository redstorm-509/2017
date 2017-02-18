#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#define PIN 12

Adafruit_NeoPixel strip = Adafruit_NeoPixel(30, PIN);

const int waitwipe = 17;

int colorValue = 1;
char receivedValue;
int CURRENT_STATE = 1;
uint16_t RAINBOW_VAL = 0;
int flashingvalue = 0;
int flashingstate = 1;
int currentteam = 0;

int defaultwhite = 0;
int rainbowvalue = 1;
int redteam = 2;
int blueteam = 3;
int loading = 4;
int shooting = 5;
int endactions = 6;
int hastarget = 7;
int notarget = 8;
bool targetvisible = false;
bool lasttargetvisible = false;



void setup() {

   Wire.begin(1);
   Wire.onReceive(Input);
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setBrightness(128);
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, 100, 100, 100);
  }
  strip.show();

}

void loop(){
  if (CURRENT_STATE == rainbowvalue) {
    rainbow();
  }
  if (CURRENT_STATE == shooting) {
    flash(strip.Color(0,245,0));
  }

  if (targetvisible != lasttargetvisible) {
    if (CURRENT_STATE == endactions) {
      lasttargetvisible = targetvisible;
      if (currentteam == 1 && targetvisible == false) {
        colorWipe(strip.Color(245,0,0),waitwipe); // red
      }
      else if (currentteam == 2  && targetvisible == false) {
        colorWipe(strip.Color(0,0,245),waitwipe); // blue
      }
      else if (targetvisible == true){
        colorWipe(strip.Color(0,245,0), waitwipe); // green
      }
      else {
        InstantColor(strip.Color(100,100,100)); // white
      }
    }
  }
  
  if (CURRENT_STATE != colorValue) {
    if (colorValue == defaultwhite) {
      InstantColor(strip.Color(100,100,100)); // white
    }
    else if (colorValue == redteam) {
      //colorWipe(strip.Color(245, 0, 0), waitwipe); // red
      currentteam = 1;
    }
    else if (colorValue == blueteam) {
      //colorWipe(strip.Color(0, 0, 245), waitwipe); // blue
      currentteam = 2;
    }
    else if (colorValue == loading) {
      InstantColor(strip.Color(245, 0, 245)); // purple
    }
    else if (colorValue == shooting) {
      InstantColor(strip.Color(0,245,0)); // green
    }
    
    else if (colorValue == endactions) {
      lasttargetvisible = targetvisible;
      if (currentteam == 1 && targetvisible == false) {
        colorWipe(strip.Color(245,0,0),waitwipe); // red
      }
     else if (currentteam == 2  && targetvisible == false) {
        colorWipe(strip.Color(0,0,245),waitwipe); // blue
      }
      else if (targetvisible == true){
        colorWipe(strip.Color(0,245,0), waitwipe); // green
      }
     else {
        InstantColor(strip.Color(100,100,100)); // white
      }
    }
    
    CURRENT_STATE = colorValue;
  }
  delay(20);
}

void Input(int RoboValue){

  if (Wire.available()){

    int newValue = Wire.read();

    if (newValue == hastarget) {
      targetvisible = true;
    }
    else if (newValue == notarget) {
      targetvisible = false;
    }
    else {
      colorValue = newValue;
    }
    
/*
    Serial.print("colorValue");
    Serial.println(colorValue);
    Serial.print("RoboValue");
    Serial.println(RoboValue);
    */
    
  }
}

void InstantColor(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
    strip.show();
}
 
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
   
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void flash(uint32_t c) {
  flashingvalue = (flashingvalue + 1);
  if (flashingvalue >= 10) {
    flashingvalue = 0;
    if (flashingstate == 1) {
      InstantColor(strip.Color(0,0,0)); // off
      Serial.println("changing to 0");
      flashingstate = 0;
    }
    else {
      InstantColor(c); // on
      Serial.println("changing to 1");
      flashingstate = 1;
    }
  }
}

void rainbow() {
  uint16_t k;
  RAINBOW_VAL = RAINBOW_VAL + 1;
  if (RAINBOW_VAL > 255) {
    RAINBOW_VAL = 0;
  }
  for(k=0; k<strip.numPixels(); k++) {
      strip.setPixelColor(k, Wheel((k+RAINBOW_VAL) & 255));
  }
  strip.show();
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
