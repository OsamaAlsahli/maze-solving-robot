//mbed library
#include <mbed.h>
#include <cmath>

#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"

//to use the mbed library types without writing mbed::DigitalOut every time.
using namespace mbed;
extern GridMap grid_map;
void setup() {
  Serial.begin(9600);
  hw_init();
  delay(200);
  grid_init(grid_map);
  Serial.println("=== ROBOT READY ===");
  Serial.println("Starting maze exploration...");
  delay(1000);
}

void loop() {
  static bool exploration_done = false;
  if(!exploration_done){
    explore_maze(grid_map);
    exploration_done = true;
  }
  stop();
  delay(1000);
}