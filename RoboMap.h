/*
 * RoboMap.h - Maze Solving and Path Recording
 * 
 * This module implements the maze-solving algorithm and path recording.
 * 
 * Algorithm: Modified Bug2
 *   - Primary goal: Travel north toward the finish line
 *   - When blocked: Use sensor data to choose best direction
 *   - When facing east/west: Check if north becomes available
 *   - Records all moves for return journey
 * 
 * Path Recording:
 *   - Every move is stored in a path[] array
 *   - After reaching finish, path is reversed for return
 *   - Turns are inverted (LEFT↔RIGHT) during return
 * 
 * Author: fwm509
 * Module: ELE00098H Robotics Design and Construction
 * Date: January 2026
 */

#pragma once
#include <mbed.h>

//=============================================================================
// GLOBAL STATE VARIABLES
//=============================================================================
extern int bearing;               // Current heading: 0=N, 1=E, 2=S, 3=W
extern bool maze_finished;        // True when finish line reached
extern int path_len;              // Number of moves recorded

//=============================================================================
// PATH RECORDING
//=============================================================================
const int MAX_PATH = 100;         // Maximum moves that can be recorded

/**
 * Move types for path recording
 * Each move the robot makes is classified as one of these types
 */
enum MoveType {
  M_FORWARD = 0,        // Forward one cell (20cm)
  M_FORWARD_SHORT = 1,  // Forward short distance (10cm)
  M_RIGHT = 2,          // Turn right 90°
  M_LEFT = 3,           // Turn left 90°
  M_AWAY = 4,           // Turn around 180°
};

extern MoveType path[MAX_PATH];   // Array storing the recorded path

//=============================================================================
// PATH RECORDING FUNCTIONS
//=============================================================================

/**
 * Record a move to the path array
 * @param m The move type to record
 */
void record(MoveType m);

/**
 * Print the entire recorded path to Serial
 * Useful for debugging and demonstration
 */
void print_path();

/**
 * Execute return journey by reversing the recorded path
 * 
 * Process:
 * 1. Turn 180° to face back toward start
 * 2. Replay moves in reverse order
 * 3. Invert turns: LEFT becomes RIGHT, RIGHT becomes LEFT
 * 4. Forward moves stay the same (already facing correct direction)
 */
void return_to_start();

//=============================================================================
// MAZE SOLVING ALGORITHMS
//=============================================================================

/**
 * Bug2 maze solver with path recording (MAIN ALGORITHM)
 * 
 * Strategy:
 * 1. Try to move north (toward finish)
 * 2. When blocked, use sensor readings to choose direction:
 *    - If both sides open: choose the more open one
 *    - If one side open: go that way
 *    - If both blocked: turn around
 * 3. When facing east/west, check if north becomes available
 * 4. Track total north progress to detect finish line
 * 5. Record every move for return journey
 * 
 * Exits when total_north >= 195cm (finish line)
 */
void bug2_try();
