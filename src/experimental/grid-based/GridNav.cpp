#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"


// threshold for the front and side sensors
float side_sensor_th = 15; //cm
float front_sensor_th = 5;//cm

unsigned long last_print = 0;
unsigned long last_pid = 0;
float steer = 0.0;
unsigned long lastsns;

void callibration(){
  long int avg_enc = (labs(enca) + labs(encb))/2;
  float f_val=read_us();
  if(mvng_frwrd){
    if(f_val < front_sensor_th && f_val > 2){
      Serial.print("Real Wall stop at: "); Serial.println(f_val);
      stop();
      mvng_frwrd = false;
      return;
    }
    keepstraight();
    if(millis()-last_pid > 50){
      steer = centering();
      last_pid = millis();
    }
    if(avg_enc < tt){
      wheels(lcs - steer, rcs + steer);
    } else{
      stop();
      mvng_frwrd = false;
      steer = 0;
    }
  } else if(trnng){
    if(avg_enc < tt){
      wheels(lcs*l_dir, rcs*r_dir);
    }else{
      stop();
      trnng = false;
    }
  }
}

void wait_until_done(){
  while(mvng_frwrd || trnng){
    callibration();
    wait_us(100);
  }
}

void turn_right(GridMap &m) {
  Serial.println("Action: Turning Right");
  turn(90,0.4);
  wait_until_done();
  m.robot_o = (orientation)((m.robot_o + 1) % 4);
}

void turn_left(GridMap &m) {
  Serial.println("Action: Turning Left");
  turn(-90, 0.4);
  wait_until_done();
  m.robot_o = (orientation)((m.robot_o + 3) % 4);
}

void turn_away(GridMap &m) {
  Serial.println("Action: Turning Back (180)");
  turn(180, 0.4);
  wait_until_done();
  m.robot_o = (orientation)((m.robot_o + 2) % 4);
}

void avoid_obstacle(){
  if((read_fl_ir() < 10 && read_fr_ir() < 10) || read_us() < 20){
    return;
  }else if(read_fl_ir() < 10){
    turn(30, 0.3);
    wait_until_done();
    move_frwrd(5, b_speed);
    wait_until_done();
    turn(-30, 0.3);
    wait_until_done();
    return;
  }else if(read_fr_ir() < 10){
    turn(-30, 0.3);
    wait_until_done();
    move_frwrd(5, b_speed);
    wait_until_done();
    turn(30, 0.3);
    wait_until_done();
    return;
  }
}
