#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"
#include <cmath>

GridMap grid_map;

void grid_init(GridMap &m){
  for(int row=0; row < grid_rows; row++){
    for(int col=0; col < grid_cols; col++){
      m.cells[row][col] = cell_unknown;
    }
  }
  
  m.robot_x = 3;
  m.robot_y = 0;
  m.robot_o = up;
  m.cells[0][3] = cell_visited;
  m.last_x = 3;
  m.last_y = -1;

  Serial.println("Grid initialized!");
  Serial.print("Robot at: (");
  Serial.print(m.robot_x);
  Serial.print(", ");
  Serial.print(m.robot_y);
  Serial.println(")");
}

void check_dir(GridMap &m, int check_dir, float dis, int dx[], int dy[]){
  int map_bearing = (m.robot_o + check_dir) %4; //north=0, east=1, south=2, west=3
  int check_x = m.robot_x + dx[map_bearing];
  int check_y = m.robot_y + dy[map_bearing];
  Serial.print("Checking direction ");
  Serial.print(map_bearing);
  Serial.print(" → cell (");
  Serial.print(check_x);
  Serial.print(", ");
  Serial.print(check_y);
  Serial.print("): ");
  if(check_x < 0 || check_x >= grid_cols || check_y < 0 || check_y >= grid_rows){
    Serial.println("OUT OF BOUNDS");
    return;
  }
  bool is_blocked;
  if(check_dir == 0){
    is_blocked = (dis < 10);
  }else{
    is_blocked = (dis < 10);
  }

  if(m.cells[check_y][check_x] == cell_visited){
    Serial.println("Already VISITED");
    return;
  }
  if(is_blocked){
    m.cells[check_y][check_x] = cell_blocked;
    Serial.println("BLOCKED");
  }else{
    m.cells[check_y][check_x] = cell_free;
    Serial.println("FREE");
  }
}

void scan_cell(GridMap &m){
  Serial.println("=== SCANNING ===");
  Serial.print("Position: (");
  Serial.print(m.robot_x);
  Serial.print(", ");
  Serial.print(m.robot_y);
  Serial.print(") heading: ");
  Serial.println(m.robot_o);

  float f_val = read_us();
  float l_val = read_l_ir();
  float r_val = read_r_ir();
  Serial.print("Front: "); Serial.print(f_val); Serial.println("cm");
  Serial.print("Left: "); Serial.print(l_val); Serial.println("cm");
  Serial.print("Right: "); Serial.print(r_val); Serial.println("cm");

  int dx[4] = {0, 1, 0, -1};
  int dy[4] = {1, 0, -1, 0};

  check_dir(m, 0, f_val, dx, dy);
  check_dir(m, 3, l_val, dx, dy);
  check_dir(m, 1, r_val, dx, dy);

}

bool choose_next_cell(GridMap &m, int &target_x, int &target_y){
  Serial.println("=== CHOOSING NEXT CELL ===");
  int dx[4] = {0, 1, 0, -1};
  int dy[4] = {1, 0, -1, 0};
  Serial.println("Checking for FREE adjacent cells...");
  for(int dir=0; dir<4; dir++){
    int check_x = m.robot_x + dx[dir];
    int check_y = m.robot_y + dy[dir];
    if(check_x < 0 || check_x >= grid_cols || check_y < 0 || check_y >= grid_rows) {
      continue;
    }

    if(m.cells[check_y][check_x] == cell_free){
      target_x = check_x;
      target_y = check_y;
       Serial.print("Found FREE cell at (");
        Serial.print(target_x);
        Serial.print(", ");
        Serial.print(target_y);
        Serial.println(")");
        return true;
    }
  }
  Serial.println("No FREE cells. Checking for UNKNOWN adjacent cells...");
  for(int dir=0; dir<4; dir++){
    int check_x = m.robot_x + dx[dir];
    int check_y = m.robot_y + dy[dir];
    if(check_x < 0 || check_x >= grid_cols || 
        check_y < 0 || check_y >= grid_rows) {
        continue;
    }

    if(m.cells[check_y][check_x] == cell_unknown) {
      target_x = check_x;
      target_y = check_y;
      
      Serial.print("Found UNKNOWN cell at (");
      Serial.print(target_x);
      Serial.print(", ");
      Serial.print(target_y);
      Serial.println(")");
      
      return true;
    }
  }
  Serial.println("No valid adjacent cells! Need to backtrack.");
  return false;
}

bool choose_next_cell_backtracking(GridMap &m, int &target_x, int &target_y){
  if(choose_next_cell(m, target_x, target_y)){
    return true;
  }
  Serial.println("BACKTRACKING: Looking for visited cells...");
  int dx[4] = {0, 1, 0, -1};
  int dy[4] = {1, 0, -1, 0};
  for(int dir=0; dir<4; dir++){
    int check_x = m.robot_x + dx[dir];
    int check_y = m.robot_y + dy[dir];
    if(check_x < 0 || check_x >= grid_cols || check_y < 0 || check_y >= grid_rows) {
      continue;
    }
    if(m.cells[check_y][check_x] == cell_visited){
      target_x = check_x;
      target_y = check_y;
      Serial.print("Backtracking to (");
      Serial.print(target_x);
      Serial.print(", ");
      Serial.print(target_y);
      Serial.println(")");
      return true;
    }
  }
  Serial.println("ERROR: Completely stuck, no moves available!");
  return false;
}

void explore_maze(GridMap &m){
  while(m.robot_y < grid_rows - 1){
    scan_cell(m);
    wait_us(500000);
    int target_x, target_y;
    bool found  = choose_next_cell_backtracking(m, target_x, target_y);
    if(!found){
      Serial.println("No path found! Stopping.");
      stop();
      return;
    }

    bool success = move_to_cell(m, target_x, target_y);
    if(success){
      m.robot_x = target_x;
      m.robot_y = target_y;
      m.cells[target_y][target_x] = cell_visited;

      Serial.println("Position updated on map");
    }else{
      m.cells[target_y][target_x] = cell_blocked;
      Serial.println("Cell marked as BLOCKED");
    }
    wait_us(500000);
  }
  Serial.println("=== REACHED FINISH LINE! ===");
  stop();
}


bool move_to_cell(GridMap &m, int target_x, int target_y){
  Serial.println("=== MOVING TO TARGET CELL ===");
  Serial.print("From (");
  Serial.print(m.robot_x);
  Serial.print(", ");
  Serial.print(m.robot_y);
  Serial.print(") to (");
  Serial.print(target_x);
  Serial.print(", ");
  Serial.print(target_y);
  Serial.println(")");

  int dx = target_x - m.robot_x;
  int dy = target_y - m.robot_y;
  int map_bearing;
  if(dy == 1){
    map_bearing = up;
    Serial.println("Need to face: UP");
  }else if(dx == 1){
    map_bearing = rt;
    Serial.println("Need to face: RIGHT");
  }else if(dy == -1){
    map_bearing = dn;
    Serial.println("Need to face: DOWN");
  }else if(dx == -1){
    map_bearing = lt;
    Serial.println("Need to face: LEFT");
  }else{
    Serial.println("ERROR: Target cell not adjacent!");
    Serial.print("dx="); Serial.print(dx);
    Serial.print(", dy="); Serial.println(dy);
    return false;
  }
  int delta = ((map_bearing - m.robot_o + 4) %4);
  switch(delta){
    case 0:
      Serial.println("Already facing target direction");
      break;
    case 1:
        // Turn right 90°
        Serial.println("Executing: Turn RIGHT 90°");
        turn(90, 0.3);           // Your existing function
        wait_until_done();       // Wait for turn to complete
        m.robot_o = (orientation)map_bearing;  // Update heading
        break;
    case 2:
        // Turn around 180°
        Serial.println("Executing: Turn AROUND 180°");
        turn(180,0.3);
        wait_until_done();
        m.robot_o = (orientation)map_bearing;
        break;
    case 3:
        // Turn left 90° (same as turn right 270°, but we use -90)
        Serial.println("Executing: Turn LEFT 90°");
        turn(-90,0.3);
        wait_until_done();
        m.robot_o = (orientation)map_bearing;
        break;  
  }
  Serial.println("Starting forward movement (30cm)...");
  move_frwrd(25, 0.3);
  unsigned long move_start = millis();
  while(mvng_frwrd){
    callibration();

    float f_val = read_us();
    if(f_val < 8 && f_val > 2){
      stop();
      Serial.print("ABORT! Front obstacle at ");
      Serial.print(f_val);
      Serial.println("cm");
      return false;
    }
    if(millis() - move_start > 300){
      float fl_val = read_fl_ir();
      if(fl_val < 5 && fl_val > 1) {  // Close obstacle on front-left
        stop();
        Serial.print("ABORT! Front-left obstacle at ");
        Serial.print(fl_val);
        Serial.println("cm");
        return false;
      }
      // Check front-right IR sensor (30° sensor)
      float fr_val = read_fr_ir();
      if(fr_val < 5 && fr_val > 1) {  // Close obstacle on front-right
        stop();
        Serial.print("ABORT! Front-right obstacle at ");
        Serial.print(fr_val);
        Serial.println("cm");
        return false;
      }
    }
    float l_val = read_l_ir();
    float r_val = read_r_ir();
    if(l_val < 3 || r_val < 3){
      stop();
      Serial.println("ABORT! Too close to side wall");
      return false;
    }
    wait_us(100);
  }
  Serial.println("Movement completed successfully!");
  return true;
}