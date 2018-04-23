  
// Include ROS libraries
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int8.h>

// Include timer libraries
#include <Event.h>
#include <Timer.h>

// LED Dotstar libraries
#include <Adafruit_DotStar.h>
#include <SPI.h>         
#include <math.h>

#define NUMPIXELS 72   // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN1    8
#define CLOCKPIN1   4
Adafruit_DotStar strip1 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN1, CLOCKPIN1, DOTSTAR_BRG);

#define DATAPIN2    6
#define CLOCKPIN2   7
Adafruit_DotStar strip2 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN2, CLOCKPIN2, DOTSTAR_BRG);

#define DATAPIN3    9
#define CLOCKPIN3   10
Adafruit_DotStar strip3 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN3, CLOCKPIN3, DOTSTAR_BRG);

#define DATAPIN4    12
#define CLOCKPIN4   13
Adafruit_DotStar strip4 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN4, CLOCKPIN4, DOTSTAR_BRG);


// Creation of variables & functions

// Subscriber callback function declaration
int emotion_status = 0;
int emotion_prev = 0;
int count = 0;

int color_green = 1;
int color_red   = 1;
int color_blue  = 1;
void EmotionUpdate(const std_msgs::Int8& msg);


Timer t; // Timer variable

ros::NodeHandle nh; // ROS variables
ros::Subscriber<std_msgs::Int8> sub_emotion_status("/emotion_status", EmotionUpdate);


void setup() {

  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
    clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
  #endif

  Serial.begin(57600);

  nh.initNode();
  nh.subscribe(sub_emotion_status);

  strip1.begin(); // Initialize pins for output
  strip1.clear();
  strip1.show();  // Turn all LEDs off ASAP

  strip2.begin(); // Initialize pins for output
  strip2.clear();
  strip2.show();  // Turn all LEDs off ASAP

  strip3.begin(); // Initialize pins for output
  strip3.clear();
  strip3.show();  // Turn all LEDs off ASAP

  strip4.begin(); // Initialize pins for output
  strip4.clear();
  strip4.show();  // Turn all LEDs off ASAP

  t.every(20, breatheLED);
}      

void loop() {
  t.update();
  nh.spinOnce();
  delay(50);
}

int i = 0;
uint32_t color = 0x000000;
uint32_t red = 0;
uint32_t green = 0; 
uint32_t blue = 0; 

void breatheLED(){
  
  if (emotion_prev == emotion_status){
    count++;
  }
  else{
    count = 0;
  }

  if (count > 4){
  
    float val = abs(-(exp(sin(millis()/3000.0*PI)) - 0.36787944)*108.0)/5;
  
    green =  color_green*val;
    red   =  color_red  *val;
    blue  =  color_blue *val;
  
    color = (green << 16) | (red << 8) | (blue);
  
    for (int i=0; i<=NUMPIXELS; i++){
      strip1.setPixelColor(i, color);
      strip2.setPixelColor(i, color);
      strip3.setPixelColor(i, color);
      strip4.setPixelColor(i, color);
    }
    strip1.show();
    strip2.show();
    strip3.show();
    strip4.show();
  }
}

void EmotionUpdate(const std_msgs::Int8& msg){
  emotion_prev = emotion_status;
  emotion_status = msg.data;

  switch (emotion_status){
    case 0: { // Disgust
      color_red   = 1;
      color_green = 0;
      color_blue  = 1;
      break;
    }
    case 1: { // Angry
      color_red   = 1;
      color_green = 0;
      color_blue  = 0;
      break;
    }
    case 2: { // Sad
      color_red   = 0;
      color_green = 0;
      color_blue  = 1;
      break;
    }
    case 3: { // Neutral
      color_red   = 1;
      color_green = 1;
      color_blue  = 1;
      break;
    }
    case 4: { // Happy
      color_red   = 0;
      color_green = 1;
      color_blue  = 0;
      break;
    }
    default: { // Neutral
      color_red   = 1;
      color_green = 1;
      color_blue  = 1;
      break;
    }
  }
}

