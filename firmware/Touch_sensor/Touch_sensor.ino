const byte TouchPin=2;
const int ledPin=13;

volatile byte state = LOW;

void setup() {
    pinMode(TouchPin, INPUT_PULLUP);
    pinMode(ledPin,OUTPUT);
    attachInterrupt(digitalPinToInterrupt(TouchPin), touch, RISING);
}

void loop() {
    if(state == HIGH)
    {
        digitalWrite(ledPin,HIGH);
    }
    else
    {
        digitalWrite(ledPin,LOW);
    }
}

void touch()
{
  state = !state;
}


