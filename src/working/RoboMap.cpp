/*
 * RoboMap.cpp - Maze Solving and Path Recording Implementation
 * 
 * This is the core maze-solving module implementing a modified Bug2 algorithm.
 * 
 * Algorithm Overview:
 * ==================
 * The Bug algorithm aims to reach a goal by:
 * 1. Moving directly toward the goal (north in this case)
 * 2. When an obstacle is hit, navigating around it
 * 3. Resuming direct travel when the goal direction is clear
 * 
 * Modifications for this maze:
 * - Uses multiple sensors to make smarter turn decisions
 * - Prefers the direction with more open space
 * - Tracks total north progress to detect finish line
 * - Records path for return journey (Phase 4 requirement)
 * 
 * Path Recording:
 * ==============
 * Every move is stored in a path[] array. After reaching the finish,
 * the robot can reverse this path to return to start efficiently.
 * The key insight is that:
 * - Forward moves stay forward (robot has turned around)
 * - LEFT turns become RIGHT turns
 * - RIGHT turns become LEFT turns
 * 
 * Author: fwm509
 * Module: ELE00098H Robotics Design and Construction
 * Date: January 2026
 */

#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"
#include <cmath>

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
int bearing = 0;                  // Current heading: 0=N, 1=E, 2=S, 3=W
int path_len = 0;                 // Number of moves recorded
bool maze_finished = false;       // True when finish line reached

MoveType path[MAX_PATH];          // Array to store recorded moves

//=============================================================================
// PATH RECORDING FUNCTIONS
//=============================================================================

/**
 * Record a move to the path array
 * 
 * @param m The move type (M_FORWARD, M_LEFT, etc.)
 * 
 * Moves are stored sequentially and replayed in reverse
 * during the return journey.
 */
void record(MoveType m) {
  if(path_len < MAX_PATH) {
    path[path_len] = m;
    path_len++;
  }
  // Note: If path is full, moves are silently dropped
  // In practice, MAX_PATH=100 should be sufficient
}

/**
 * Print the recorded path to Serial monitor
 * 
 * Output format:
 *   0: FWD
 *   1: LEFT
 *   2: FWD
 *   etc.
 */
void print_path() {
  Serial.println("=== RECORDED PATH ===");
  Serial.print("Total moves: ");
  Serial.println(path_len);
  
  for(int i = 0; i < path_len; i++) {
    Serial.print(i);
    Serial.print(": ");
    switch(path[i]) {
      case M_FORWARD:       Serial.println("FWD"); break;
      case M_FORWARD_SHORT: Serial.println("FWD_S"); break;
      case M_RIGHT:         Serial.println("RIGHT"); break;
      case M_LEFT:          Serial.println("LEFT"); break;
      case M_AWAY:          Serial.println("AWAY"); break;
    }
  }
  Serial.println("=====================");
}

/**
 * Return to start by reversing the recorded path
 * 
 * Algorithm:
 * 1. Turn 180° to face back toward start
 * 2. Iterate through path[] in reverse order
 * 3. For each move:
 *    - Forward moves: execute forward (same distance)
 *    - Left turns: execute RIGHT (inverted)
 *    - Right turns: execute LEFT (inverted)
 *    - 180° turns: execute 180° (same)
 * 
 * This works because after turning 180°, what was "left"
 * relative to the original direction is now "right".
 */
void return_to_start() {
  Serial.println("=== RETURNING TO START ===");
  
  // First turn around to face south (back toward start)
  turn(180, return_speed);
  wait_until_done();
  
  // Process moves in reverse order
  for(int i = path_len - 1; i >= 0; i--) {
    Serial.print("Return step ");
    Serial.print(path_len - i);
    Serial.print("/");
    Serial.println(path_len);
    
    switch(path[i]) {
      case M_FORWARD:
        // Forward stays forward (we've already turned around)
        return_forward();
        break;
        
      case M_FORWARD_SHORT:
        return_forward_short();
        break;
        
      case M_RIGHT:
        // Right becomes left (mirror image)
        return_left();
        break;
        
      case M_LEFT:
        // Left becomes right (mirror image)
        return_right();
        break;
        
      case M_AWAY:
        // 180° turn stays 180° turn
        turn(180, return_speed);
        wait_until_done();
        break;
    } 
  }
  
  Serial.println("=== BACK AT START ===");
}

//=============================================================================
// BUG2 MAZE SOLVING ALGORITHM (Main Implementation)
//=============================================================================

/**
 * Bug2 maze solver with path recording
 * 
 * This is the main navigation algorithm used to solve the maze.
 * 
 * State Variables:
 * - bearing: Current compass direction (0=N, 1=E, 2=S, 3=W)
 * - total_north: Distance traveled north (cm)
 * - moves_since_turn: Prevents immediate re-turning
 * - turn_cnt: Legacy counter (not actively used)
 * 
 * Decision Logic:
 * 
 * 1. IF facing north AND front clear:
 *    → Move forward, add to north progress
 * 
 * 2. IF facing east AND left opens up:
 *    → Turn left to face north, check if actually clear
 *    → If not clear, turn back and continue east
 * 
 * 3. IF facing west AND right opens up:
 *    → Turn right to face north, check if actually clear
 *    → If not clear, turn back and continue west
 * 
 * 4. IF facing south:
 *    → Try to turn toward north (left or right)
 * 
 * 5. IF front blocked:
 *    → Compare left and right sensor readings
 *    → Turn toward the more open direction
 *    → Move forward short to clear the corner
 * 
 * 6. ELSE (front clear, not facing north):
 *    → Move forward, track north progress if N/S
 */
void bug2_try() {
  Serial.println("==========BUG2 TRIAL==========");
  
  // Initialize state
  bearing = 0;                    // Start facing north
  int moves_since_turn = 0;       // Prevent immediate re-turning
  int turn_cnt = 0;               // Turn counter (legacy)
  float total_north = 0;          // Total northward progress (cm)
  
  // Main navigation loop
  while (true) {
    // Read all relevant sensors
    float f_val = read_us();      // Front ultrasonic
    float l_val = read_l_ir();    // Left IR
    float r_val = read_r_ir();    // Right IR
    
    // Debug output - sensor readings
    Serial.print("B:");
    Serial.print(bearing);
    Serial.print(" F:");
    Serial.print(f_val);
    Serial.print(" L:");
    Serial.print(l_val);
    Serial.print(" R:");
    Serial.println(r_val);
    
    //=========================================================================
    // CASE 1: Facing north and front is clear - MOVE NORTH!
    //=========================================================================
    if(bearing == 0 && f_val > 15) {
      go_forward();
      record(M_FORWARD);
      total_north += move_dis;
      
      Serial.print("North progress: ");
      Serial.println(total_north + north_avoid);
      
      // Check if we've reached the finish line
      if(total_north + north_avoid >= 190) {
        Serial.println("=== REACHED FINISH LINE! ===");
        stop();
        maze_finished = true;
        return;                   // Exit maze solving
      }
      
      moves_since_turn = 0;
      continue;
    }
    
    //=========================================================================
    // CASE 2: Facing east - check if we can turn back north
    //=========================================================================
    else if(bearing == 1 && l_val > 15 && moves_since_turn >= 1) {
      turn_left();                // Turn to face north
      record(M_LEFT);
      
      float check = read_us();    // Check if north is actually clear
      if(check < 10) {
        // North is blocked, turn back to face east
        turn_right();
        record(M_RIGHT);
        go_forward_short();       // Move away to avoid getting stuck
        record(M_FORWARD_SHORT);
      }
      
      moves_since_turn = 0;
      continue;
    }
    
    //=========================================================================
    // CASE 3: Facing west - check if we can turn back north
    //=========================================================================
    else if(bearing == 3 && r_val > 15 && moves_since_turn >= 1) {
      turn_right();               // Turn to face north
      record(M_RIGHT);
      
      float check = read_us();    // Check if north is actually clear
      if(check < 10) {
        // North is blocked, turn back to face west
        turn_left();
        record(M_LEFT);
        go_forward_short();       // Move away to avoid getting stuck
        record(M_FORWARD_SHORT);
      }
      
      moves_since_turn = 0;
      turn_cnt++;
      continue;
    }
    
    //=========================================================================
    // CASE 4: Facing south - try to turn toward north
    //=========================================================================
    else if(bearing == 2 && moves_since_turn >= 1) {
      if(l_val > 15) {
        // Left is open - turn left (toward east, then can go north)
        turn_left();
        record(M_LEFT);
        moves_since_turn = 0;
        continue;
      } else if(r_val > 15) {
        // Right is open - turn right (toward west, then can go north)
        turn_right();
        record(M_RIGHT);
        moves_since_turn = 0;
        turn_cnt++;
        continue;
      }
    }
    
    //=========================================================================
    // CASE 5: Front is blocked - need to turn
    //=========================================================================
    if(f_val < 15) {
      // Smart direction choice: compare left and right openness
      if(l_val > 15 && r_val > 15) {
        // Both sides open - choose the MORE open one
        if(r_val > l_val) {
          turn_right();
          record(M_RIGHT);
        } else {
          turn_left();
          record(M_LEFT);
        }
      } else if(l_val > 15) {
        // Only left is open
        turn_left();
        record(M_LEFT);
      } else if(r_val > 15) {
        // Only right is open
        turn_right();
        record(M_RIGHT);
      } else {
        // Both sides blocked - dead end, turn around
        turn_away();
        record(M_AWAY);
      }
      
      moves_since_turn = 0;
      
      // Move forward short to clear the corner
      go_forward_short();
      record(M_FORWARD_SHORT);
      
      // Update north tracking for short moves
      if(bearing == 2) {
        total_north -= 10;        // Moving south
      } else if(bearing == 0) {
        total_north += 10;        // Moving north
      }
    }
    
    //=========================================================================
    // CASE 6: Front is clear but not facing north - move forward
    //=========================================================================
    else {
      go_forward();
      record(M_FORWARD);
      
      // Update north tracking
      if(bearing == 2) {
        total_north -= move_dis;  // Moving south
      } else if(bearing == 0) {
        total_north += move_dis;  // Moving north
      }
      
      moves_since_turn++;
    }
    
    wait_us(100000);              // Brief pause before next iteration
  }
}