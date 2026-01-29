#pragma once
#include <mbed.h>

extern float side_sensor_th;
extern float front_sensor_th;
extern float front_dis;
extern float right_dis;
extern float left_dis;
extern int  stt_switch;
extern unsigned long last_print;
extern unsigned long last_pid;
extern float steer;
extern bool traversing;
extern int nodehits;
extern unsigned long lastsns;
extern int node_cnt;
extern int crnt_node;
extern float last_l_val;
extern float last_r_val;

void callibration();

enum orientation {
  up = 0,
  rt = 1,
  dn = 2,
  lt = 3,
};

struct Pose {
  int x;
  int y;
  orientation o;
};

enum Robostates{
  stt_stop = -1,
  stt_forward = 0,
  stt_turn_right = 1,
  stt_turn_back = 2,
  stt_turn_left = 3,
};

extern Pose robot;
extern Robostates state;

void callibration();
void wait_until_done();
void turn_right(Pose &r);
void turn_left(Pose &r);
void turn_away(Pose &r);
void avoid_obstacle();
void safe_move();
void move_to_node_center();
void traverse();
void wtg();
void fsm(Pose &r);
