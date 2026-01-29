#pragma once
#include <mbed.h>


//left/right encoder count variables
extern volatile long int enca;
extern volatile long int encb;
extern bool mvng_frwrd;
extern bool trnng;
//target tick count
extern long int tt;
extern float lcs;
extern float rcs;
extern float disa;
extern float disb;
extern float diff;
extern float b_speed;
// motor control
void wheels(float l_speed, float r_speed);
void stop();
// motion primitives
void move_frwrd(float dis, float spd);
void move_back(float dis, float spd);
void keepstraight();
void turn(float deg, float spd);
void distance();
// sensors
float read_us();
float read_l_ir();
float read_r_ir();
float read_fl_ir();
float read_fr_ir();
void hw_init();
float centering();