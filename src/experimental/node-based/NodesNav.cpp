#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"

float front_dis;
float right_dis;
float left_dis;

Pose robot = { 0, 0, up };

// threshold for the front and side sensors
float side_sensor_th = 15; //cm
float front_sensor_th = 5;//cm

Robostates state = stt_forward;

unsigned long last_print = 0;
unsigned long last_pid = 0;
float steer = 0.0;
bool traversing = false;
int  nodehits = 0;
unsigned long lastsns;
int crnt_node = 0;
int chosen_dir = -1;
float last_l_val = 0;
float last_r_val = 0;

void callibration(){
  long int avg_enc = (labs(enca) + labs(encb))/2;

  if(mvng_frwrd){
    if(read_us() < front_sensor_th && read_us() > 2){
      Serial.print("Real Wall stop at: "); Serial.println(read_us());
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
    if(tt > 0 && avg_enc < tt){
      wheels(lcs, -rcs);
    }else if(tt < 0 && avg_enc < -tt){
      wheels(-lcs, rcs);
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

void turn_right(Pose &r) {
  Serial.println("Action: Turning Right");
  turn(90,0.4);
  wait_until_done();
  r.o = (orientation)((r.o + 1) % 4);
}

void turn_left(Pose &r) {
  Serial.println("Action: Turning Left");
  turn(-90, 0.4);
  wait_until_done();
  r.o = (orientation)((r.o + 3) % 4);
}

void turn_away(Pose &r) {
  Serial.println("Action: Turning Back (180)");
  turn(180, 0.4);
  wait_until_done();
  r.o = (orientation)((r.o + 2) % 4);
}

void move_to_node_center(){
  Serial.print("Centering at node. Front dist: ");
  Serial.println(read_us());
  if(read_us() < (front_sensor_th + 5)){
    move_frwrd(5,b_speed);
  }else{
    move_frwrd(14,b_speed);
  }
  wait_until_done();
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

void safe_move() {
  Serial.println("Action: Safe Move (12cm)");
  enca = 0;
  encb = 0;
  move_frwrd(22, b_speed);
  while(mvng_frwrd){
    callibration();
    //avoid_obstacle();
    long int avg_enc = (labs(enca) + labs(encb))/2;
    if(avg_enc > 300){
      if(read_l_ir() < side_sensor_th || read_r_ir() < side_sensor_th){
        break;
      }  
    }
  }
  stop();
  Serial.println("Junction Cleared: Sensors see walls.");
}

void traverse(){
  keepstraight();
  float steer = centering();  
  wheels(lcs - steer, rcs + steer);
  long int avg_enc = (labs(enca) + labs(encb))/2;
  //avoid_obstacle();
  unsigned long now = millis();
  if(now - lastsns < 60UL){
    return;
  }
  lastsns = now;
  float f_val = read_us();
  float r_val = read_r_ir();
  float l_val = read_l_ir();

  bool front = (f_val > front_sensor_th);
  bool right = (r_val > side_sensor_th);
  bool left = (l_val > side_sensor_th);
  bool node_detected = ((!front) && f_val > 2) ||
                       (left && l_val > 2 && avg_enc > 180) || 
                       (right && r_val > 2 && avg_enc > 180);

  Serial.print("TRAV | F:"); Serial.print(f_val);
  Serial.print(" R:"); Serial.print(r_val);
  Serial.print(" L:"); Serial.print(l_val);
  Serial.print(" Hits:"); Serial.println(nodehits);
  
  if(node_detected){
    nodehits++;
  }else{
    nodehits = 0;
  }

  if(nodehits > 2){
    Serial.println(">>> NODE DETECTED <<<");
    stop();
    wait_until_done();
    move_frwrd(22,b_speed);
    wait_until_done();
    traversing = false;
    nodehits = 0;
    return;
  }
}

//wtg : where to go
void wtg(){
  chosen_dir = -1;
  for(int i = 0; i <= 3; i++){
    if(nodes[crnt_node].open[i] == true && nodes[crnt_node].explored[i] == false){
      chosen_dir = i;
      break;
    }
  }  
  if(chosen_dir != -1){
    nodes[crnt_node].explored[chosen_dir] = true;
  }
  if(chosen_dir == -1 && nodes[crnt_node].parent != -1){
      chosen_dir = nodes[crnt_node].parent_dir;
  }
  Serial.print("WTG: Chosen direction index is "); Serial.println(chosen_dir);
}

void fsm(Pose &r){
  Serial.println("--- Entering FSM ---");
  walls(r);
  wtg();
  new_node();
  if(chosen_dir == -1){
    state = stt_stop;
    return;
  }
  int delta = ((chosen_dir - r.o + 4) %4);
  switch (delta){
    case 0: state = stt_forward;    Serial.println("State: Forward"); break;
    case 1: state = stt_turn_right; Serial.println("State: Turn Right"); break;
    case 2: state = stt_turn_back;  Serial.println("State: Turn Back"); break;
    case 3: state = stt_turn_left;  Serial.println("State: Turn Left"); break;
  }
  switch (state){
    case stt_forward:
      depart_node = crnt_node;
      depart_dir = chosen_dir;
      traversing = true;
      break;
    case stt_stop:
      Serial.println("State: Stop (Finished)");
      stop();
      break;
    case stt_turn_right: 
      turn_right(r);
      safe_move();
      Serial.println("rstt");
      traversing = true;
      break;
    case stt_turn_left:  
      turn_left(r);
      safe_move();
      Serial.println("lstt");
      traversing = true;
      break;
    case stt_turn_back:
      turn_away(r);
      safe_move();
      Serial.println("astt");
      traversing = true;
      break;
  }
}
