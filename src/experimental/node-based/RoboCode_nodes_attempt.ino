//mbed library
#include <mbed.h>
#include <cmath>

#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"

//to use the mbed library types without writing mbed::DigitalOut every time.
using namespace mbed;

void setup() {
  Serial.begin(9600);
  hw_init();
  delay(200);
  map_init();
}

void loop() {
  callibration();
  if(!trnng && !mvng_frwrd){
    if(traversing){
      traverse();
    }else{
      fsm(robot);
    }
  }
}
