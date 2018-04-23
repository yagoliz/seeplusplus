#include <Timer.h>
#include <Event.h>

#include <ros.h>
#include <std_msgs/Int8.h>

// --------------------------------------------------
// Sensor variables
Timer t;

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
ros::Publisher pub("/touch_status", &value);


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
}

void loop() {
  nh.spinOnce();
  t.update();
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
