/*
 * RoboNav.h - Navigation and Motion Control
 * 
 * This module provides high-level navigation functions for the maze robot.
 * It builds on the hardware layer to provide:
 *   - Blocking motion commands (go_forward, turn_left, etc.)
 *   - Obstacle avoidance during motion
 *   - Sensor threshold definitions
 *   - Status reporting functions
 * 
 * Author: fwm509
 * Module: ELE00098H Robotics Design and Construction
 * Date: January 2026
 */

#pragma once
#include <mbed.h>

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================
extern unsigned long last_print;    // Timestamp for debug output throttling
extern unsigned long last_pid;      // Timestamp for PID update
extern float steer;                 // Steering correction value
extern unsigned long lastsns;       // Last sensor read timestamp
extern bool need_to_avoid;          // Flag: obstacle avoidance in progress
extern float north_avoid;           // Accumulated north distance from avoidance maneuvers
extern float return_speed;          // Speed for return journey (faster than exploration)

//=============================================================================
// NAVIGATION CONSTANTS
// These thresholds control when the robot considers paths open or blocked
//=============================================================================
const float move_dis = 20;          // Standard move distance (cm) - one grid cell
const float front_open = 20;        // Front is "open" if distance > this (cm)
const float front_closed = 10;      // Front is "blocked" if distance < this (cm)
const float side_open = 15;         // Side is "open" if distance > this (cm)
const float side_closed = 10;       // Side is "blocked" if distance < this (cm)

//=============================================================================
// ORIENTATION ENUM
// Cardinal directions for robot heading tracking
//=============================================================================
enum orientation {
  up = 0,   // North - toward finish
  rt = 1,   // East - right of start
  dn = 2,   // South - toward start
  lt = 3,   // West - left of start
};

//=============================================================================
// CALIBRATION AND CONTROL FUNCTIONS
//=============================================================================

/**
 * Add north component when avoiding obstacles
 * Updates north_avoid based on current bearing
 */
void add_north_component();

/**
 * Main motion control loop - called continuously during motion
 * 
 * Handles:
 *   - Encoder-based distance tracking
 *   - Straight-line correction
 *   - Obstacle detection and avoidance
 *   - Motion completion detection
 */
void callibration();

/**
 * Block until current motion completes
 * Continuously calls callibration() while moving
 */
void wait_until_done();

/**
 * Wait during avoidance maneuver (simplified control)
 */
void wait_to_avoid();

//=============================================================================
// EXPLORATION MOTION COMMANDS (Slower, careful speed)
//=============================================================================

/**
 * Move forward one grid cell (20cm) at exploration speed
 * Blocking - returns when motion complete
 */
void go_forward();

/**
 * Move forward a short distance (10cm) at exploration speed
 * Used after turns to clear corners
 */
void go_forward_short();

/**
 * Turn right 90° at exploration speed
 * Updates bearing after turn
 */
void turn_right();

/**
 * Turn left 90° at exploration speed
 * Updates bearing after turn
 */
void turn_left();

/**
 * Turn around 180° at exploration speed
 * Used when reaching dead ends
 */
void turn_away();

//=============================================================================
// RETURN JOURNEY MOTION COMMANDS (Faster speed)
//=============================================================================

/**
 * Move forward one grid cell at return speed (faster)
 */
void return_forward();

/**
 * Move forward short distance at return speed
 */
void return_forward_short();

/**
 * Turn right 90° at return speed
 */
void return_right();

/**
 * Turn left 90° at return speed
 */
void return_left();

//=============================================================================
// STATUS AND SENSOR HELPER FUNCTIONS
//=============================================================================

/**
 * Get human-readable name for current bearing
 * @return "NORTH", "EAST", "SOUTH", "WEST", or "???"
 */
const char* bearing_name();

/**
 * Print current robot status to Serial
 * Shows bearing, mode, and all sensor readings
 */
void print_status();

/**
 * Check if robot can move forward safely
 * @return true if front, front-left, and front-right are all clear
 */
bool can_go_forward();

/**
 * Check if front is blocked
 * @return true if front distance < front_closed threshold
 */
bool front_blocked();

/**
 * Check if left side is open (for turning)
 * @return true if left distance > side_open threshold
 */
bool left_open();

/**
 * Check if right side is open (for turning)
 * @return true if right distance > side_open threshold
 */
bool right_open();
