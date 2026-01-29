#pragma once
#include <mbed.h>

struct Node{
  bool open [4];
  bool explored [4];
  int link [4];
  int parent;
  int parent_dir;
};

constexpr int max_nodes = 50;
extern Node nodes[max_nodes];

extern int chosen_dir;
extern int target_dir;
extern int depart_node;
extern int depart_dir;

void init_node(int id);
void map_init();
void walls(Pose &r);
void new_node();
