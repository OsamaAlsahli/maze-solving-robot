#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"
#include <cmath>

using namespace mbed;

//pwm control for the left motor (motor A).
PwmOut pwma(P0_27);
//pwm control for motor A.
DigitalOut dira(P0_4);
//direction control for right motor (motor B).
PwmOut pwmb(P1_2);
//direction control for motor B.
DigitalOut dirb(P0_5);
//Encoder pins (Distance control) for motors A and B
InterruptIn tica(P1_11);
InterruptIn ticb(P1_12);
//front ultrasonic sensor pins
DigitalInOut us_front(P0_23);
//timer for us sensor
Timer us_t;
I2C i2c(P0_31, P0_2);

//left/right encoder count variables
volatile long int enca = 0;
volatile long int encb = 0;

bool mvng_frwrd = false;
bool trnng = false;
//target tick count
long int tt = 0;

float lcs = 0.0;
float rcs = 0.0;
//left/right motor speed variables
float l_speed = 0;
float r_speed = 0;
//left/rightmotor directions
int l_dir = +1;
int r_dir = +1;
//distance per encoder tick (cm)
float dpt = 0.048;
//distance moved by each motor
float disa = 0;
float disb = 0;
//base speed
float b_speed = 0.3;

float speed = 0.4f;

float dis = 0;

float k = 0.025;
//tickss per degree
float tpd = 3.15;

float dis_wall = 0;

float diff = 0;

float us_r = 0;

void l_cnt() {
  enca = enca + l_dir;
}

void r_cnt() {
  encb = encb + r_dir;
}

void distance() {
  disa = abs(enca) * dpt;
  disb = abs(encb) * dpt;
}

//to control the robot wheels individually
void wheels(float l_speed, float r_speed) {

  //left motor direcetion: if speed is positive, move forward
  if (l_speed > 0) {
    dira = 1;
    l_dir = +1;
    //otherwise, move backward
  } else if (l_speed < 0) {
    dira = 0;
    l_dir = -1;
  } else {
    l_dir = 0;
  }
  //same for right motor
  if (r_speed > 0) {
    dirb = 1;
    r_dir = +1;
  } else if (r_speed < 0) {
    dirb = 0;
    r_dir = -1;
  } else {
    r_dir = 0;
  }
  //using the absolute value of motor speeds, rotate at that speed
  pwma.write(fabs(l_speed));
  pwmb.write(fabs(r_speed));
}

void stop() {
  wheels(0, 0);
  mvng_frwrd = false;
  trnng = false;
}
//to move forward at a certain speed for a certain distnce
void move_frwrd(float dis, float spd) {
  enca = 0;
  encb = 0;
  tt = dis / dpt;
  speed = spd;
  mvng_frwrd = true;
}

void move_back(float dis, float spd){
  enca = 0;
  encb = 0;
  tt = dis / dpt;
  speed = -spd;
  mvng_frwrd = true;
}

void keepstraight() {
  //to keep the robot straight
  diff = abs(enca) - abs(encb);
  lcs = speed - k * diff;
  rcs = speed + k * diff;
  if (lcs < 0) {
    lcs = 0;
  } else if (lcs > 1) {
    lcs = 1;
  } else {
    lcs = lcs;
  }
  if (rcs < 0) {
    rcs = 0;
  } else if (rcs > 1) {
    rcs = 1;
  } else {
    rcs = rcs;
  }
}
//to rotate the robot a set amount of degrees at a certain speed
void turn(float deg, float spd) {
  enca = 0;
  encb = 0;
  tt = tpd * deg;
  speed = spd;
  lcs = spd;
  rcs = spd;
  trnng = true;
}
//to red the front ultr
float read_us() {
  us_front.output();
  us_t.reset();
  us_front = 0;
  wait_us(5);
  us_front = 1;
  wait_us(10);
  us_front = 0;
  us_front.input();
  us_t.reset();
  us_t.start();
  while (us_front.read() == 0 && us_t.read_us() < 30000) {}
  if (us_front.read() == 0) {
    return 1000;
  }
  us_t.reset();
  us_t.start();
  while (us_front.read() == 1 && us_t.read_us() < 30000) {}
  us_t.stop();
  float pw_us = us_t.read_us();
  us_r = pw_us / 58;
  return us_r;
}

float read_l_ir() {
  const char mux_cmd = 0x02;
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  unsigned int raw_l = (cmd[0] << 4) | (cmd[1] >> 4);
  float l_wall_dis = raw_l / 64.0f;
  return l_wall_dis * 0.866;
}

float read_r_ir() {
  const char mux_cmd = 0x04;
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  unsigned int raw_r = (cmd[0] << 4) | (cmd[1] >> 4);
  float r_wall_dis = raw_r / 64.0f;
  return r_wall_dis * 0.866;
}

float read_fl_ir(){
  const char mux_cmd = 0x01;
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  unsigned int raw_fl = (cmd[0] << 4) | (cmd[1] >> 4);
  float fl_wall_dis = raw_fl / 64.0f;
  return fl_wall_dis;
}

float read_fr_ir(){
  const char mux_cmd = 0x08;
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  unsigned int raw_fr = (cmd[0] << 4) | (cmd[1] >> 4);
  float fr_wall_dis = raw_fr / 64.0f;
  return fr_wall_dis;
}

void hw_init(){
    //period for pwm
  pwma.period_us(2000);
  pwmb.period_us(2000);
  tica.rise(&l_cnt);
  ticb.rise(&r_cnt);
  i2c.frequency(100000);
}

float centering(){
  
  float left = read_l_ir();
  float right = read_r_ir();
  const float Kp = 0.02;
  const float max_steer =0.15;
  float error = 0.0;
  if(left < side_sensor_th && right < side_sensor_th){
    error = left - right;
  }else if(left > side_sensor_th){
    error = (10 - right);
  }else if(right < side_sensor_th){
    error = (left - 10);
  }else{error = 0.0;}
  
  float cntrng = error*Kp;
  
  if(cntrng > max_steer){
    cntrng = max_steer;
  }else if(cntrng < -max_steer){
    cntrng = -max_steer;
  }
  return cntrng;
}
