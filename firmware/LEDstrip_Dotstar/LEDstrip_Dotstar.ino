  
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
#define CLOCKPIN1   12
Adafruit_DotStar strip1 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN1, CLOCKPIN1, DOTSTAR_BRG);

#define DATAPIN2    3
#define CLOCKPIN2   7
Adafruit_DotStar strip2 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN2, CLOCKPIN2, DOTSTAR_BRG);

#define DATAPIN3    46
#define CLOCKPIN3   40
Adafruit_DotStar strip3 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN3, CLOCKPIN3, DOTSTAR_BRG);

#define DATAPIN4    28
#define CLOCKPIN4   24
Adafruit_DotStar strip4 = Adafruit_DotStar(
  NUMPIXELS, DATAPIN4, CLOCKPIN4, DOTSTAR_BRG);


// Creation of variables & functions
void breathLoop();


// Subscriber callback function declaration
int emotion_status = 0;

int green = 1;
int red   = 1;
int blue  = 1;
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
  delay(10);
}


void breatheLED(){  
  
  static int i = 0;
  static bool flag=true;

  if (flag){
    if(i++ > 200) {
      flag = false;
    }else{
      for(int j=0; j<strip1.numPixels(); j++) {
        strip1.setPixelColor(j, strip1.Color(green*i, red*i, blue*i));
        strip2.setPixelColor(j, strip2.Color(green*i, red*i, blue*i));
        strip3.setPixelColor(j, strip3.Color(green*i, red*i, blue*i));
        strip4.setPixelColor(j, strip4.Color(green*i, red*i, blue*i));
      }
    }
  }else{
    if(i-- < 1) {
      flag = true;
    }else{
      for(int j=0; j<strip1.numPixels(); j++) {
        strip1.setPixelColor(j, strip1.Color(green*i, red*i, blue*i));
        strip2.setPixelColor(j, strip2.Color(green*i, red*i, blue*i));
        strip3.setPixelColor(j, strip3.Color(green*i, red*i, blue*i));
        strip4.setPixelColor(j, strip4.Color(green*i, red*i, blue*i));
      }
    }
  }
  
  strip1.show();
  strip2.show();
  strip3.show();
  strip4.show();
}

void EmotionUpdate(const std_msgs::Int8& msg){

  emotion_status = msg.data;

  switch (emotion_status){
    case 0: { // Disgust
      red   = 1;
      green = 0;
      blue  = 1;
      break;
    }
    case 1: { // Angry
      red   = 1;
      green = 0;
      blue  = 0;
      break;
    }
    case 2: { // Sad
      red   = 0;
      green = 0;
      blue  = 1;
      break;
    }
    case 3: { // Neutral
      red   = 1;
      green = 1;
      blue  = 1;
      break;
    }
    case 4: { // Happy
      red   = 0;
      green = 1;
      blue  = 0;
      break;
    }
    default: { // Neutral
      red   = 1;
      green = 1;
      blue  = 1;
      break;
    }
  }
}


