#pragma once
#include <mbed.h>

extern float side_sensor_th;
extern float front_sensor_th;
extern float front_dis;
extern float right_dis;
extern float left_dis;
extern unsigned long last_print;
extern unsigned long last_pid;
extern float steer;
extern unsigned long lastsns;


void callibration();

enum orientation {
  up = 0,
  rt = 1,
  dn = 2,
  lt = 3,
};

void callibration();
void wait_until_done();
void avoid_obstacle();