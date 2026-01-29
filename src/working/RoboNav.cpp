/*
 * RoboNav.cpp - Navigation and Motion Control Implementation
 * 
 * This module implements high-level navigation for the maze robot.
 * 
 * Key Features:
 *   - Blocking motion commands with obstacle avoidance
 *   - Real-time obstacle detection during movement
 *   - Automatic avoidance maneuvers for diagonal obstacles
 *   - Separate speeds for exploration (slow) and return (fast)
 * 
 * The callibration() function is the heart of the motion control system.
 * It runs continuously during any motion to:
 *   1. Check for obstacles
 *   2. Perform avoidance if needed
 *   3. Apply straight-line correction
 *   4. Stop when target distance reached
 * 
 * Author: fwm509
 * Module: ELE00098H Robotics Design and Construction
 * Date: January 2026
 */

#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
unsigned long last_print = 0;       // For debug output timing
unsigned long last_pid = 0;         // For PID update timing
float steer = 0.0;                  // Steering correction
unsigned long lastsns;              // Last sensor timestamp
bool need_to_avoid = false;         // Avoidance in progress flag
float north_avoid = 0;              // North distance from avoidance maneuvers
float return_speed = 0.6f;          // Return journey speed (faster than exploration)

//=============================================================================
// HELPER FUNCTIONS
//=============================================================================

/**
 * Track north progress during obstacle avoidance
 * 
 * When the robot performs an avoidance maneuver, it may move
 * slightly north or south. This function updates north_avoid
 * to keep the total north distance accurate.
 */
void add_north_component() {
  if(bearing == 0) {              // Facing north
    north_avoid += 7;             // Add ~7cm north progress
  } else if(bearing == 2) {       // Facing south
    north_avoid -= 7;             // Subtract north progress
  }
}

//=============================================================================
// MAIN MOTION CONTROL FUNCTION
//=============================================================================

/**
 * Core motion control loop - handles all active motion
 * 
 * This function should be called continuously (via wait_until_done)
 * while any motion is in progress. It handles:
 * 
 * 1. FORWARD MOTION (mvng_frwrd == true):
 *    - Check front ultrasonic for walls
 *    - Check front-left/right IR for diagonal obstacles
 *    - Apply keepstraight() correction
 *    - Stop when target distance reached
 * 
 * 2. TURNING (trnng == true):
 *    - Monitor encoder counts
 *    - Stop when target angle reached
 * 
 * 3. OBSTACLE AVOIDANCE:
 *    - If diagonal obstacle detected, perform avoidance maneuver
 *    - Turn away from obstacle, move forward, turn back
 *    - Different speeds for exploration vs return journey
 */
void callibration() {
  static int wall_cnt = 0;        // Counter for wall detection (noise filtering)
  
  // Calculate average encoder count
  long int avg_enc = (labs(enca) + labs(encb)) / 2;
  
  // Read front ultrasonic
  float f_val = read_us();
  
  //===========================================================================
  // FORWARD MOTION CONTROL
  //===========================================================================
  if(mvng_frwrd) {
    
    // --- Front Wall Detection (with noise filtering) ---
    // Requires 5 consecutive readings below threshold to stop
    if(f_val < front_closed && f_val > 7) {
      wall_cnt++;
      if(wall_cnt >= 5) {
        Serial.print("Wall stop at: "); 
        Serial.println(f_val);
        stop();
        mvng_frwrd = false;
        wall_cnt = 0;
        return;
      }
    } else {
      wall_cnt = 0;               // Reset counter if clear reading
    }
    
    // --- Diagonal Obstacle Detection ---
    float fl_val = read_fl_ir();  // Front-left sensor
    float fr_val = read_fr_ir();  // Front-right sensor
    
    // Front-left obstacle - perform right avoidance maneuver
    if(fl_val < 8) {
      need_to_avoid = true;
      Serial.print("FL stop at: "); 
      Serial.println(fl_val);
      stop();
      
      // Different avoidance behavior for exploration vs return
      if(!maze_finished) {
        // EXPLORATION: Slow and careful, track north progress
        turn(45, b_speed);
        wait_until_done();
        go_forward_short();
        wait_until_done();
        turn(-45, b_speed);
        wait_until_done();
        stop();
        add_north_component();
        
        // Adjust north tracking for east/west travel
        if(bearing == 1) {
          north_avoid -= 7;
        } else if(bearing == 3) {
          north_avoid += 7;
        }
      } else {
        // RETURN: Fast, no north tracking needed
        turn(45, return_speed);
        wait_until_done();
        return_forward_short();
        wait_until_done();
        turn(-45, return_speed);
        wait_until_done();
        stop();
      }
      
      need_to_avoid = false;
      mvng_frwrd = false;
      return;
    }
    
    // Front-right obstacle - perform left avoidance maneuver
    else if(fr_val < 8) {
      need_to_avoid = true;
      Serial.print("FR stop at: "); 
      Serial.println(fr_val);
      stop();
      
      if(!maze_finished) {
        // EXPLORATION: Slow and careful
        turn(-45, b_speed);
        wait_until_done();
        go_forward_short();
        wait_until_done();
        turn(45, b_speed);
        wait_until_done();
        stop();
        add_north_component();
        
        if(bearing == 1) {
          north_avoid += 7;
        } else if(bearing == 3) {
          north_avoid -= 7;
        }
      } else {
        // RETURN: Fast
        turn(-45, return_speed);
        wait_until_done();
        return_forward_short();
        wait_until_done();
        turn(45, return_speed);
        wait_until_done();
        stop();
      }
      
      need_to_avoid = false;
      mvng_frwrd = false;
      return;
    }
    
    // --- Normal Forward Motion ---
    keepstraight();               // Apply straight-line correction
    
    if(avg_enc < tt) {
      wheels(lcs, rcs);           // Continue moving
    } else {
      stop();                     // Target reached
      mvng_frwrd = false;
    }
  }
  
  //===========================================================================
  // TURNING CONTROL
  //===========================================================================
  else if(trnng) {
    if(avg_enc < tt) {
      wheels(lcs * l_dir, rcs * r_dir);  // Continue turning
    } else {
      stop();                     // Target angle reached
      trnng = false;
    }
  }
}

/**
 * Block until current motion completes
 * Polls callibration() at 10kHz
 */
void wait_until_done() {
  while(mvng_frwrd || trnng) {
    callibration();
    wait_us(100);
  }
}

/**
 * Simplified wait for avoidance maneuvers
 * No obstacle checking, just distance tracking
 */
void wait_to_avoid() {
  long int avg_enc = (labs(enca) + labs(encb)) / 2;
  
  if(mvng_frwrd) {
    keepstraight();
    if(avg_enc < tt) {
      wheels(lcs, rcs);
    } else {
      stop();
      mvng_frwrd = false;
    }
  } else if(trnng) {
    if(avg_enc < tt) {
      wheels(lcs * l_dir, rcs * r_dir);
    } else {
      stop();
      trnng = false;
    }
  }
}

//=============================================================================
// EXPLORATION MOTION COMMANDS
// These use b_speed (base speed = 0.3) for careful maze exploration
//=============================================================================

/**
 * Move forward one grid cell (20cm)
 * Includes 100ms pause after movement for sensor stabilization
 */
void go_forward() {
  move_forward(move_dis, b_speed);
  wait_until_done();
}

/**
 * Move forward short distance (10cm)
 * Used to clear corners after turning
 */
void go_forward_short() {
  move_forward(10, b_speed);
  wait_until_done();
}

/**
 * Turn right 90 degrees
 * Updates bearing to track robot orientation
 */
void turn_right() {
  Serial.println("Action: Turning Right");
  turn(90, b_speed);
  wait_until_done();
  bearing = (bearing + 1) % 4;    // Update bearing: N→E→S→W→N
}

/**
 * Turn left 90 degrees
 * Updates bearing to track robot orientation
 */
void turn_left() {
  Serial.println("Action: Turning Left");
  turn(-90, b_speed);
  wait_until_done();
  bearing = (bearing + 3) % 4;    // Update bearing: N→W→S→E→N
}

/**
 * Turn around 180 degrees (for dead ends)
 */
void turn_away() {
  Serial.println("Action: Turning Back (180)");
  turn(180, b_speed);
  wait_until_done();
  bearing = (bearing + 2) % 4;    // Reverse bearing
}

//=============================================================================
// RETURN JOURNEY MOTION COMMANDS
// These use return_speed (0.7) for faster return trip
//=============================================================================

/**
 * Move forward one cell at return speed
 */
void return_forward() {
  move_forward(move_dis, return_speed);
  wait_until_done();
}

/**
 * Move forward short distance at return speed
 */
void return_forward_short() {
  move_forward(10, return_speed);
  wait_until_done();
}

/**
 * Turn right at return speed
 */
void return_right() {
  Serial.println("Action: Turning Right");
  turn(90, return_speed);
  wait_until_done();
  bearing = (bearing + 1) % 4;
}

/**
 * Turn left at return speed
 */
void return_left() {
  Serial.println("Action: Turning Left");
  turn(-90, return_speed);
  wait_until_done();
  bearing = (bearing + 3) % 4;
}

//=============================================================================
// STATUS AND SENSOR HELPER FUNCTIONS
//=============================================================================

/**
 * Convert bearing number to human-readable string
 */
const char* bearing_name() {
  switch(bearing) {
    case 0: return "NORTH";
    case 1: return "EAST";
    case 2: return "SOUTH";
    case 3: return "WEST";
    default: return "???";
  }
}

/**
 * Print comprehensive status to Serial monitor
 * Useful for debugging navigation issues
 */
void print_status() {
  Serial.println("---------------------------");
  Serial.print("Bearing: ");
  Serial.println(bearing_name());
  Serial.print("Mode: ");
  Serial.print("Front: ");
  Serial.print(read_us());
  Serial.println(" cm");
  Serial.print("Left: ");
  Serial.print(read_l_ir());
  Serial.println(" cm");
  Serial.print("Right: ");
  Serial.print(read_r_ir());
  Serial.println(" cm");
  Serial.println("---------------------------");
}

/**
 * Check if forward path is clear
 * Checks front ultrasonic AND both diagonal IR sensors
 */
bool can_go_forward() {
  float f_val = read_us();
  float fl_val = read_fl_ir();
  float fr_val = read_fr_ir();
  
  return (f_val > front_open && fl_val > 8 && fr_val > 8);
}

/**
 * Check if front is blocked by wall
 */
bool front_blocked() {
  float f_val = read_us();
  return (f_val < front_closed);
}

/**
 * Check if left side is open for turning
 */
bool left_open() {
  float l_val = read_l_ir();
  return (l_val > side_open);
}

/**
 * Check if right side is open for turning
 */
bool right_open() {
  float r_val = read_r_ir();
  return (r_val > side_open);
}
