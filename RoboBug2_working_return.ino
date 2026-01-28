/*
 * RoboBug2_working_return.ino - Main Robot Controller
 * 
 * Maze-Solving Robot for ELE00098H Assessment
 * University of York - School of Physics, Engineering and Technology
 * 
 * =============================================================================
 * PROJECT OVERVIEW
 * =============================================================================
 * 
 * This robot navigates a maze from START to FINISH and back using:
 *   - Bug navigation algorithm (modified for sensor-based decisions)
 *   - Path recording for efficient return journey
 *   - Multiple sensors for obstacle detection and avoidance
 * 
 * =============================================================================
 * HARDWARE CONFIGURATION
 * =============================================================================
 * 
 * Microcontroller: Arduino Nano 33 BLE
 * Motor Driver:    TB6612FNG dual H-bridge
 * Motors:          2x micro metal gear motors with encoders
 * 
 * Sensors:
 *   - 1x HC-SR04 ultrasonic (front, long range)
 *   - 4x Sharp IR distance sensors via I2C multiplexer:
 *     - Left (90°)
 *     - Right (90°)
 *     - Front-Left (45°)
 *     - Front-Right (45°)
 * 
 * =============================================================================
 * ALGORITHM SUMMARY
 * =============================================================================
 * 
 * Phase 3 - Maze Solving (bug2_try):
 * 1. Primary goal: Travel north toward finish line
 * 2. When blocked: Choose direction based on sensor readings
 * 3. Track total north progress to detect finish
 * 4. Record every move for return journey
 * 
 * Phase 4 - Return Journey (return_to_start):
 * 1. Turn 180° to face start
 * 2. Replay recorded moves in reverse order
 * 3. Invert turns (LEFT↔RIGHT)
 * 4. Travel at higher speed (0.6 vs 0.3)
 * 
 * =============================================================================
 * CODE ORGANIZATION
 * =============================================================================
 * 
 * RoboHW.h/cpp   - Hardware abstraction (motors, sensors, encoders)
 * RoboNav.h/cpp  - Navigation functions (motion commands, obstacle avoidance)
 * RoboMap.h/cpp  - Maze solving algorithm and path recording
 * 
 * Author: fwm509
 * Module: ELE00098H Robotics Design and Construction
 * Date: January 2026
 */

#include <mbed.h>
#include <cmath>

#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"

using namespace mbed;

//=============================================================================
// SETUP - Run once at startup
//=============================================================================

/**
 * Initialize robot hardware and prepare for maze solving
 */
void setup() {
  // Initialize serial communication for debugging (9600 baud)
  Serial.begin(9600);
  
  // Initialize hardware (motors, encoders, sensors)
  hw_init();
  
  // Wait 2 seconds for robot to be placed and user to step back
  // Also allows sensors to stabilize
  wait_us(2000000);
  
  // Initialize state flags
  maze_finished = false;
  path_len = 0;
  
  Serial.println("=================================");
  Serial.println("  MAZE SOLVING ROBOT");
  Serial.println("  Ready to start!");
  Serial.println("=================================");
}

//=============================================================================
// MAIN LOOP - Runs continuously
//=============================================================================

/**
 * Main control loop
 * 
 * State machine:
 * 1. maze_finished == false: Run bug2_try() to solve maze
 * 2. maze_finished == true:  Execute return journey, then stop
 */
void loop() {
  if(!maze_finished) {
    //=========================================================================
    // PHASE 3: SOLVE THE MAZE
    //=========================================================================
    // Run Bug2 algorithm to navigate from START to FINISH
    // This function records all moves and sets maze_finished = true when done
    bug2_try();
    
  } else {
    //=========================================================================
    // PHASE 4: RETURN TO START
    //=========================================================================
    
    // Print the recorded path for demonstration
    print_path();
    
    // Wait 3 seconds at finish line (for assessment demonstration)
    Serial.println("Waiting 3 seconds before return...");
    wait_us(3000000);
    
    // Execute return journey using recorded path
    return_to_start();
    
    // Mission complete - stop and report
    while(true) {
      stop();
      Serial.println("=================================");
      Serial.println("  MISSION COMPLETE!");
      Serial.println("  Robot has returned to start.");
      Serial.println("=================================");
      wait_us(5000000);           // Print status every 5 seconds
    }
  }
}
