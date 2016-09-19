#include "Manchester.h"

#define RX_PIN 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  man.setupReceive(RX_PIN, MAN_38400);
  man.beginReceive();

}

void loop() {
  if (man.receiveComplete()) {
    uint16_t m = man.getMessage();
    Serial.println(m);
    man.beginReceive(); //start listening for next message right after you retrieve the message
  }

}
