#include <Adafruit_DotStar.h>

#include <ros.h>
#include <ros/time.h>

#include <Timer.h>

#include <std_msgs/Int8.h>

//LED
#define PIXEL_PIN    9
#define PIXEL_COUNT 56


#define FLASH_F 2.0 //hz

void EmotionUpdate(const std_msgs::Int8& msg);
void TouchUpdate(const std_msgs::Int8& msg);
void update_light();
void breathGreen();
void theaterChaseRainbow();

// Ros
ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int8> sub_emotion_status("/emotion_status", &EmotionUpdate );
ros::Subscriber<std_msgs::Int8> sub_touch_status("/touch_status", &TouchUpdate );

//LED
Timer t_led;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

int emotion_mode_= 0;
int touch_mode_= 0;

void setup() {

  //LED
  strip.begin();
  LEDupdate_light(emotion_mode_, touch_mode_);
  strip.show(); // Initialize all pixels to 'off'
  t_led.every(10, update_light);


  // ROS node
  nh.initNode();
  nh.subscribe(sub_emotion_status);
  nh.subscribe(sub_touch_status);

}


void loop() {
  t_led.update();
  nh.spinOnce();

}

void update_light() {
  LEDupdate_light(emotion_mode_, touch_mode_);
  strip.show();
}

void EmotionUpdate(const std_msgs::Int8& msg){

  emotion_mode_ = msg.data;
}

void TouchUpdate(const std_msgs::Int8& msg){

  touch_mode_ = msg.data;
}

void LEDupdate_light(int &emotion_mode, int &touch_mode)
{
  static boolean flash = true;
  static int count = 0;
  if(touch_mode == 0){
    switch(emotion_mode){
      case 3: // neutral
        for(int i=0; i < strip.numPixels(); i++)
        {
          strip.setPixelColor(i, strip.Color(200,200,200));
        }
        break;
      case 2: // happy
        breathGreen();
        break;
      case 0: // angry
        for(int i=0; i < strip.numPixels(); i++)
        {
          strip.setPixelColor(i, strip.Color(200,0,0));
        }
        break;
      case 4: // sad
        for(int i=0; i < strip.numPixels(); i++)
        {
          strip.setPixelColor(i, strip.Color(0,0,200));
        }
        break;
      case 1: // surprised
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
      }
    }else if(touch_mode == 1){ //right turu
      for(int i=0; i < 28; i++)
      {
        if (flash){
          strip.setPixelColor(strip.numPixels()-(i+1), strip.Color(200,180,0));
        }else{
          strip.setPixelColor(strip.numPixels()-(i+1), strip.Color(0,0,0));
        }
       if (count > (1.0/FLASH_F)*100){
          flash = !flash;
          count = 0;
        }
        strip.setPixelColor(i, strip.Color(0,0,0));
      }
    }else if(touch_mode == 2){ //left turu
       for(int i=0; i < 28; i++)
      {
        if (flash){
          strip.setPixelColor(i, strip.Color(200,180,0));
        }else{
          strip.setPixelColor(i, strip.Color(0,0,0));
        }
        if (count > (1.0/FLASH_F)*100){
          flash = !flash;
          count = 0;
        }
        strip.setPixelColor(strip.numPixels()-(i+1), strip.Color(0,0,0));
      }
    }else if(touch_mode == 3){ //brake
      for(int i=0; i < strip.numPixels(); i++)
        {
          if (flash){
            strip.setPixelColor(i, strip.Color(200,0,0));
          }else{
            strip.setPixelColor(i, strip.Color(0,0,0));
          }
          if (count > (1.0/FLASH_F)*100){
            flash = !flash;
            count = 0;
          }
          strip.setPixelColor(19, strip.Color(0,0,0));
        }

    }else if(touch_mode == 4){ //party
      theaterChaseRainbow();
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
