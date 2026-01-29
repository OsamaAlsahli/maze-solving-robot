#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"
#include <cmath>

Node nodes[max_nodes];

int node_cnt = 0;
int target_dir = -1;
int depart_node = -1;
int depart_dir = -1;

void init_grid()

void walls(Pose &r){
  for(int i = 0; i <= 3; i++){
  nodes[crnt_node].open[i] = false;
  }
  bool front = read_us() > front_sensor_th;
  bool left = read_l_ir() > side_sensor_th;
  bool right = read_r_ir() > side_sensor_th;
  if(r.o == up){
    nodes[crnt_node].open[0] = front;
    nodes[crnt_node].open[1] = right;
    nodes[crnt_node].open[3] = left;
  }else if(r.o == rt){
    nodes[crnt_node].open[0] = left;
    nodes[crnt_node].open[1] = front;
    nodes[crnt_node].open[2] = right;
  }else if(r.o == dn){
    nodes[crnt_node].open[1] = left;
    nodes[crnt_node].open[2] = front;
    nodes[crnt_node].open[3] = right;
  }else if(r.o == lt){
    nodes[crnt_node].open[0] = right;
    nodes[crnt_node].open[2] = left;
    nodes[crnt_node].open[3] = front;
  }
  if(nodes[crnt_node].parent != -1){
    nodes[crnt_node].open[nodes[crnt_node].parent_dir] = true;
  }
}

void init_node(int id){
  for(int i = 0; i <= 3; i++){
    nodes[id].open[i] = false;
    nodes[id].explored[i] = false;
    nodes[id].link[i] = -1;
  }
  nodes[id].parent = -1;
  nodes[id].parent_dir = -1;
}

void new_node(){
  if (node_cnt == 1 && crnt_node == 0 && chosen_dir == -1){
    return;
  }
  if(chosen_dir != nodes[crnt_node].parent_dir){
    depart_node = crnt_node;
    crnt_node = node_cnt;
    node_cnt++;
    init_node(crnt_node);
    nodes[depart_node].link[chosen_dir] = crnt_node;
    nodes[crnt_node].parent = depart_node;

    nodes[crnt_node].parent_dir = (chosen_dir +2)%4;
    nodes[crnt_node].explored[nodes[crnt_node].parent_dir] = true;
  }else{
    crnt_node= nodes[crnt_node].parent;
  }
}

void map_init(){
  init_node(0);
  node_cnt = 1;
  crnt_node = 0;
}
