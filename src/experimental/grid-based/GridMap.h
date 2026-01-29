#pragma once
#include <mbed.h>

const int grid_cols=7;
const int grid_rows=6;

const int cell_unknown=0;
const int cell_free=1;
const int cell_blocked=2;
const int cell_visited=3;

struct GridMap{
  uint8_t cells[grid_rows][grid_cols];
  int robot_x;
  int robot_y;
  orientation robot_o;
  int last_x;
  int last_y;
};

extern GridMap grid_map;

void grid_init(GridMap &m);
void scan_cell(GridMap &m);
void check_dir(GridMap &m, int check_dir, float dis, int dx[], int dy[]);
bool choose_next_cell(GridMap &m, int &target_x, int &target_y);
bool choose_next_cell_backtracking(GridMap &m, int &target_x, int &target_y);
bool move_to_cell(GridMap &m, int target_x, int target_y);
void explore_maze(GridMap &m);

