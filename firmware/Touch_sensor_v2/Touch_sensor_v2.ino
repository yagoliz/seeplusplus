#include <Timer.h>
#include <Event.h>

const byte TouchPin=2;
const int ledPin=13;

Timer t;

int state = 0;
volatile byte change = LOW;
volatile int count = 0;
String message[] = {"Once","Twice"};

void setup() {
  Serial.begin(57600);
  pinMode(TouchPin, INPUT_PULLUP);
  pinMode(ledPin,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(TouchPin), touch, RISING);
  t.every(1000, changestate);
}

void loop() {
  if(state >= 2 && change == HIGH)
  {
    digitalWrite(ledPin,change);
  }
  else if (state == 1 && change == change)
  {
    digitalWrite(ledPin,HIGH);
  }
  else
  {
    digitalWrite(ledPin,change);
  }
  t.update();
}

void touch()
{
  count++;
}

void changestate()
{
  noInterrupts();
  state = count;
  count = 0;
  interrupts();
  if (state > 0) {
    change = !change;
    if (state==1) Serial.println(message[0]);
    else Serial.println(message[1]);
  }
}


