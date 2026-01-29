/*
 * RoboHW.h - Hardware Abstraction Layer
 * 
 * This module provides low-level hardware control for the maze-solving robot.
 * It handles motor control, encoder reading, and sensor interfaces.
 * 
 * Hardware:
 *   - Arduino Nano 33 BLE microcontroller
 *   - TB6612FNG dual motor driver
 *   - 2x DC motors with quadrature encoders
 *   - HC-SR04 ultrasonic sensor (front)
 *   - 4x Sharp IR distance sensors (left, right, front-left, front-right)
 * 
 * Author: fwm509
 * Module: ELE00098H Robotics Design and Construction
 * Date: January 2026
 */

#pragma once
#include <mbed.h>

//=============================================================================
// ENCODER VARIABLES
// Used for odometry and distance tracking
//=============================================================================
extern volatile long int enca;    // Left motor encoder count (volatile for ISR)
extern volatile long int encb;    // Right motor encoder count (volatile for ISR)

//=============================================================================
// MOTION STATE FLAGS
// Track current robot motion state
//=============================================================================
extern bool mvng_frwrd;           // True when robot is moving forward
extern bool trnng;                // True when robot is turning

//=============================================================================
// MOTION CONTROL VARIABLES
//=============================================================================
extern long int tt;               // Target tick count for current motion
extern float lcs;                 // Left motor corrected speed (after PID)
extern float rcs;                 // Right motor corrected speed (after PID)
extern int l_dir;                 // Left motor direction (+1=forward, -1=back)
extern int r_dir;                 // Right motor direction (+1=forward, -1=back)
extern float disa;                // Distance traveled by left motor (cm)
extern float disb;                // Distance traveled by right motor (cm)
extern float diff;                // Encoder difference for straight-line correction

//=============================================================================
// CONSTANTS
//=============================================================================
const float b_speed = 0.3f;       // Base speed for maze exploration (0.0-1.0)

//=============================================================================
// MOTOR CONTROL FUNCTIONS
//=============================================================================

/**
 * Set individual wheel speeds
 * @param l_speed Left motor speed (-1.0 to 1.0, negative = reverse)
 * @param r_speed Right motor speed (-1.0 to 1.0, negative = reverse)
 */
void wheels(float l_speed, float r_speed);

/**
 * Stop both motors immediately
 * Also resets motion state flags (mvng_frwrd, trnng)
 */
void stop();

//=============================================================================
// MOTION PRIMITIVE FUNCTIONS
//=============================================================================

/**
 * Start moving forward a specified distance
 * Non-blocking: sets up motion, use wait_until_done() to complete
 * @param dis Distance to travel in centimeters
 * @param spd Speed (0.0 to 1.0)
 */
void move_forward(float dis, float spd);

/**
 * Start moving backward a specified distance
 * Non-blocking: sets up motion, use wait_until_done() to complete
 * @param dis Distance to travel in centimeters
 * @param spd Speed (0.0 to 1.0)
 */
void move_back(float dis, float spd);

/**
 * Proportional correction to keep robot traveling straight
 * Uses encoder difference to adjust left/right motor speeds
 * Called continuously during forward motion
 */
void keepstraight();

/**
 * Start turning a specified angle
 * Non-blocking: sets up motion, use wait_until_done() to complete
 * @param deg Degrees to turn (positive = right, negative = left)
 * @param spd Turn speed (0.0 to 1.0)
 */
void turn(float deg, float spd);

/**
 * Calculate distance traveled by each motor
 * Updates disa and disb variables
 */
void distance();

//=============================================================================
// SENSOR FUNCTIONS
//=============================================================================

/**
 * Read front ultrasonic sensor (HC-SR04)
 * @return Distance in centimeters (0-500, 1000 = timeout/no echo)
 */
float read_us();

/**
 * Read left IR distance sensor
 * @return Distance in centimeters
 */
float read_l_ir();

/**
 * Read right IR distance sensor
 * @return Distance in centimeters
 */
float read_r_ir();

/**
 * Read front-left IR distance sensor (angled 45°)
 * @return Distance in centimeters
 */
float read_fl_ir();

/**
 * Read front-right IR distance sensor (angled 45°)
 * @return Distance in centimeters
 */
float read_fr_ir();

//=============================================================================
// INITIALIZATION
//=============================================================================

/**
 * Initialize all hardware
 * Sets up PWM, encoders, I2C, and sensor interfaces
 * Must be called once in setup()
 */
void hw_init();
