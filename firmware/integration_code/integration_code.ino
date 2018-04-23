#include <Timer.h>
#include <Event.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoHardware.h>

#include <ros.h>
#include <std_msgs/Int8.h>

// --------------------------------------------------
// Sensor variables
Timer t;


//LED
#define PIXEL_PIN    9
#define PIXEL_COUNT 56

#define FLASH_F 2.0 //hz

void Update_status(const std_msgs::Int8& msg);
void update_light();
void breathGreen();
void theaterChaseRainbow(); 

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

int led_mode_= 0;

const byte touchpinright = 2;
const byte touchpinleft = 3;

volatile byte countright = 0;
volatile byte countleft = 0;

int stateright = 0;
int stateleft = 0;

int changeright = 0;
int changeleft = 0;

// --------------------------------------------------
// ROS variables
ros::NodeHandle  nh;

std_msgs::Int8 value;
ros::Publisher pub("touch_sensors", &value);
ros::Subscriber<std_msgs::Int8> sub_status("/emotion", &Update_status );

// --------------------------------------------------
// setup & loop functions
void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub);

  pinMode(touchpinright, INPUT_PULLUP);
  pinMode(touchpinleft, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(touchpinright), touchright, RISING);
  attachInterrupt(digitalPinToInterrupt(touchpinleft), touchleft, RISING);
  t.every(1000, touchchange);
  t.every(10,update_light);
}

void loop() {
  
  t.update();
  nh.spinOnce();
}

// --------------------------------------------------
// Interrupts & timers

void touchright(){
  countright++;
}

void touchleft(){
  countleft++;
}

void touchchange()
{
  noInterrupts();
  stateright = countright;
  countright = 0;
  stateleft = countleft;
  countleft = 0;
  interrupts();

  // ---------------------------------------------
  // First conditions
  if (stateright>0){
    if (stateleft < 1) {
      if (changeleft == 1) {changeright = 1; changeleft = 0;}
      else changeright = 1-changeright;
    }
    else {
      if (changeright == 1 && changeleft == 1){changeright = 0; changeleft = 0;}
      else if (changeright == 1) changeleft = 1;
      else if (changeleft == 1) changeright = 1;
      else {changeright = 1; changeleft = 1;}
    }
  }
  else {
    if (stateleft > 0) {
      if (changeright == 1) {changeright = 0; changeleft = 1;}
      else changeleft = 1-changeleft;    
    }
  }

  // ---------------------------------------------
  // Value setting
//  Serial.println(stateleft);
//  Serial.println(changeleft);
  if (changeright == 1 && changeleft == 1)
  {
    if (stateright == 1) {
      value.data =3;
      // Turn red value
    }
    else if (stateright > 1){
//      Serial.println("Hey");
      value.data = 4; // PARTY MODE
    }
    
  }
  else if (changeright == 1)
  {
    if (stateright == 1) {
      value.data = 1;
      // Turn right value
    }
    else if (stateright > 1){
//      Serial.println("Hey");
      value.data = 4; // PARTY MODE
    }
  }
  else if (changeleft == 1)
  {
    if (stateleft == 1) {
      value.data = 2;
      // Turn left value
    }
    else if (stateleft > 1){
//      Serial.println("Hey");
      value.data = 4; // PARTY MODE
    }
  }
  else
  {
    if (stateright>0 || stateleft>0)
      value.data = 0;      
  }
  Serial.println(value.data);
  pub.publish(&value);
  
}
//----------------------------------------------
void update_light() {
  LEDupdate_light(led_mode_);
  strip.show(); 
}

void Update_status(const std_msgs::Int8& msg){

  led_mode_ = msg.data;
}

void LEDupdate_light(int &mode)
{
  static boolean flash = true;
  static int count = 0;
  switch(5){
    case 0: // neutral
      for(int i=0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, strip.Color(0,0,0));
      }
      break;
    case 1: // happy
      breathGreen();
      break;
    case 2: // angry
      for(int i=0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, strip.Color(200,0,0));
      }
      break;
    case 3: // sad
      for(int i=0; i < strip.numPixels(); i++)
      {
        strip.setPixelColor(i, strip.Color(0,0,200));
      }
      break;
    case 4: // surprised
      for(int i=0; i < strip.numPixels(); i++)
      {
        if (flash){
          strip.setPixelColor(i, strip.Color(200,200,0));
        }else{
          strip.setPixelColor(i, strip.Color(0,0,0));
        }
        if (count > (1.0/FLASH_F)*100){
          flash = !flash;
          count = 0;
        }
        strip.setPixelColor(19, strip.Color(0,0,0));
      }
      break;
    case 5: // patry
      theaterChaseRainbow();
      break;
  }
  count++;
}

void breathGreen() { 
  static int i = 0;
  static bool flag=true;
  int l = 1; 
  if (flag){
    if(i++ > 200) {
      flag = false;
    }else{
      for(int j=0; j<strip.numPixels(); j++) {  
        strip.setPixelColor(j, strip.Color(0,i,0));     
      }
    }
  }else{
    if(i-- < 1) {
      flag = true;
    }else{
      for(int j=0; j<strip.numPixels(); j++) {   
        strip.setPixelColor(j, strip.Color(0,i,0));
      }
    }
  }
}

void theaterChaseRainbow() {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      strip.show();
      delay(50);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
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

