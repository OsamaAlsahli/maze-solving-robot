/*
 * RoboHW.cpp - Hardware Abstraction Layer Implementation
 * 
 * This module implements low-level hardware control for the maze-solving robot.
 * It provides motor control with encoder feedback, and sensor reading functions.
 * 
 * Key Features:
 *   - Differential drive control with PWM
 *   - Quadrature encoder counting via interrupts
 *   - Proportional straight-line correction
 *   - Ultrasonic and IR sensor interfaces via I2C multiplexer
 * 
 * Author: fwm509
 * Module: ELE00098H Robotics Design and Construction
 * Date: January 2026
 */

#include "RoboHW.h"
#include "RoboNav.h"
#include "RoboMap.h"
#include <cmath>

using namespace mbed;

//=============================================================================
// HARDWARE PIN DEFINITIONS
//=============================================================================

// Motor A (Left) - PWM speed control and direction
PwmOut pwma(P0_27);               // PWM output for left motor speed
DigitalOut dira(P0_4);            // Direction control for left motor

// Motor B (Right) - PWM speed control and direction
PwmOut pwmb(P1_2);                // PWM output for right motor speed
DigitalOut dirb(P0_5);            // Direction control for right motor

// Encoder inputs - Using interrupts for accurate counting
InterruptIn tica(P1_11);          // Left motor encoder channel A
InterruptIn ticb(P1_12);          // Right motor encoder channel A

// Ultrasonic sensor - Single pin for trigger and echo
DigitalInOut us_front(P0_23);     // HC-SR04 ultrasonic sensor

// Timer for ultrasonic pulse measurement
Timer us_t;

// I2C bus for IR sensors (via multiplexer)
I2C i2c(P0_31, P0_2);             // SDA, SCL pins

//=============================================================================
// GLOBAL VARIABLES
//=============================================================================

// Encoder counts - volatile because modified in ISR
volatile long int enca = 0;       // Left encoder tick count
volatile long int encb = 0;       // Right encoder tick count

// Motion state flags
bool mvng_frwrd = false;          // Currently moving forward
bool trnng = false;               // Currently turning

// Motion control variables
long int tt = 0;                  // Target tick count for motion
float lcs = 0.0;                  // Left corrected speed
float rcs = 0.0;                  // Right corrected speed
float l_speed = 0;                // Left motor speed setting
float r_speed = 0;                // Right motor speed setting
int l_dir = +1;                   // Left motor direction
int r_dir = +1;                   // Right motor direction

// Odometry constants and variables
float dpt = 0.048;                // Distance per encoder tick (cm)
                                  // Calculated from wheel circumference / encoder CPR
float disa = 0;                   // Distance traveled by left wheel
float disb = 0;                   // Distance traveled by right wheel
float speed;                      // Current speed setting
float k = 0.025;                  // Proportional gain for straight-line correction
float tpd = 3.15;                 // Encoder ticks per degree of rotation
                                  // Calibrated for robot's wheel base

float diff = 0;                   // Encoder difference (for correction)
float us_r = 0;                   // Last ultrasonic reading

//=============================================================================
// ENCODER INTERRUPT SERVICE ROUTINES
//=============================================================================

/**
 * Left encoder ISR
 * Called on rising edge of encoder signal
 * Increments or decrements count based on direction
 */
void l_cnt() {
  enca = enca + l_dir;
}

/**
 * Right encoder ISR
 * Called on rising edge of encoder signal
 * Increments or decrements count based on direction
 */
void r_cnt() {
  encb = encb + r_dir;
}

//=============================================================================
// ODOMETRY FUNCTIONS
//=============================================================================

/**
 * Calculate distance traveled by each wheel
 * Converts encoder ticks to centimeters
 */
void distance() {
  disa = abs(enca) * dpt;
  disb = abs(encb) * dpt;
}

//=============================================================================
// MOTOR CONTROL FUNCTIONS
//=============================================================================

/**
 * Set individual wheel speeds with direction control
 * 
 * @param l_speed Left motor speed (-1.0 to 1.0)
 * @param r_speed Right motor speed (-1.0 to 1.0)
 * 
 * Positive values = forward, negative = reverse
 * Sets direction pins and PWM duty cycle
 */
void wheels(float l_speed, float r_speed) {
  // Left motor direction control
  if (l_speed > 0) {
    dira = 1;                     // Forward direction
    l_dir = +1;                   // Encoder counts up
  } else if (l_speed < 0) {
    dira = 0;                     // Reverse direction
    l_dir = -1;                   // Encoder counts down
  } else {
    l_dir = 0;                    // Stopped
  }
  
  // Right motor direction control
  if (r_speed > 0) {
    dirb = 1;                     // Forward direction
    r_dir = +1;                   // Encoder counts up
  } else if (r_speed < 0) {
    dirb = 0;                     // Reverse direction
    r_dir = -1;                   // Encoder counts down
  } else {
    r_dir = 0;                    // Stopped
  }
  
  // Set PWM duty cycle (absolute value for speed magnitude)
  pwma.write(fabs(l_speed));
  pwmb.write(fabs(r_speed));
}

/**
 * Emergency stop - immediately halt both motors
 * Also clears motion state flags
 */
void stop() {
  wheels(0, 0);
  mvng_frwrd = false;
  trnng = false;
}

//=============================================================================
// MOTION PRIMITIVE FUNCTIONS
//=============================================================================

/**
 * Initiate forward motion for a specified distance
 * 
 * @param dis Distance to travel in centimeters
 * @param spd Speed (0.0 to 1.0)
 * 
 * This is non-blocking - it sets up the motion parameters
 * Use wait_until_done() to block until motion completes
 */
void move_forward(float dis, float spd) {
  enca = 0;                       // Reset encoder counts
  encb = 0;
  tt = dis / dpt;                 // Calculate target ticks
  speed = spd;                    // Store speed for correction
  mvng_frwrd = true;              // Set motion flag
}

/**
 * Initiate backward motion for a specified distance
 * 
 * @param dis Distance to travel in centimeters
 * @param spd Speed (0.0 to 1.0)
 */
void move_back(float dis, float spd) {
  enca = 0;
  encb = 0;
  tt = dis / dpt;
  speed = -spd;                   // Negative speed for reverse
  mvng_frwrd = true;
}

/**
 * Proportional controller to maintain straight-line travel
 * 
 * Compares left and right encoder counts and adjusts motor
 * speeds to compensate for drift. Uses proportional gain k.
 * 
 * If left wheel is ahead: slow left, speed up right
 * If right wheel is ahead: slow right, speed up left
 */
void keepstraight() {
  diff = abs(enca) - abs(encb);   // Calculate encoder difference
  lcs = speed - k * diff;         // Adjust left speed
  rcs = speed + k * diff;         // Adjust right speed
  
  // Clamp speeds to valid range [-1, 1]
  if (lcs < -1) lcs = -1;
  if (lcs > 1) lcs = 1;
  if (rcs < -1) rcs = -1;
  if (rcs > 1) rcs = 1;
}

/**
 * Initiate a turn for a specified angle
 * 
 * @param deg Degrees to turn (positive = right, negative = left)
 * @param spd Turn speed (0.0 to 1.0)
 * 
 * Uses differential drive: one wheel forward, one reverse
 * for turning in place
 */
void turn(float deg, float spd) {
  enca = 0;
  encb = 0;
  tt = tpd * fabs(deg);           // Calculate target ticks for angle
  speed = spd;
  
  if(deg > 0) {
    // Turn right: left wheel forward, right wheel reverse
    lcs = spd;
    rcs = spd;
    l_dir = 1;
    r_dir = -1;
  } else {
    // Turn left: left wheel reverse, right wheel forward
    lcs = spd;
    rcs = spd;
    l_dir = -1;
    r_dir = 1;
  }
  trnng = true;                   // Set turning flag
}

//=============================================================================
// SENSOR FUNCTIONS
//=============================================================================

/**
 * Read the front ultrasonic sensor (HC-SR04)
 * 
 * @return Distance in centimeters (1000 = no echo/timeout)
 * 
 * Sends 10µs trigger pulse, measures echo pulse width
 * Distance = pulse_width / 58 (speed of sound calculation)
 */
float read_us() {
  // Send trigger pulse
  us_front.output();              // Set pin as output
  us_t.reset();
  us_front = 0;
  wait_us(5);
  us_front = 1;                   // 10µs HIGH pulse
  wait_us(10);
  us_front = 0;
  
  // Wait for echo
  us_front.input();               // Set pin as input
  us_t.reset();
  us_t.start();
  
  // Wait for echo to go HIGH (start of pulse)
  while (us_front.read() == 0 && us_t.read_us() < 30000) {}
  
  // Timeout check
  if (us_front.read() == 0) {
    return 1000;                  // No echo received
  }
  
  // Measure echo pulse width
  us_t.reset();
  us_t.start();
  while (us_front.read() == 1 && us_t.read_us() < 30000) {}
  us_t.stop();
  
  // Convert pulse width to distance
  float pw_us = us_t.read_us();
  us_r = pw_us / 58;              // Speed of sound: 343m/s → 58µs/cm
  return us_r;
}

/**
 * Read left IR distance sensor via I2C multiplexer
 * 
 * @return Distance in centimeters
 * 
 * Uses TCA9548A multiplexer (address 0xEE) to select channel
 * Then reads from Sharp IR sensor (address 0x80)
 */
float read_l_ir() {
  const char mux_cmd = 0x02;      // Select multiplexer channel 1
  const char mux_addr = 0xEE;     // Multiplexer I2C address
  i2c.write(mux_addr, &mux_cmd, 1);
  
  // Read from IR sensor
  char cmd[2];
  cmd[0] = 0x5E;                  // Distance register address
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);        // Send register address
  wait_us(1000);                  // Wait for measurement
  i2c.read(0x80, cmd, 2);         // Read 2 bytes
  
  // Convert raw value to distance
  unsigned int raw_l = (cmd[0] << 4) | (cmd[1] >> 4);
  float l_wall_dis = raw_l / 64.0f;
  return l_wall_dis;
}

/**
 * Read right IR distance sensor via I2C multiplexer
 * @return Distance in centimeters
 */
float read_r_ir() {
  const char mux_cmd = 0x04;      // Select multiplexer channel 2
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  
  unsigned int raw_r = (cmd[0] << 4) | (cmd[1] >> 4);
  float r_wall_dis = raw_r / 64.0f;
  return r_wall_dis;
}

/**
 * Read front-left IR distance sensor (angled 30°)
 * Used for detecting diagonal obstacles
 * @return Distance in centimeters
 */
float read_fl_ir() {
  const char mux_cmd = 0x01;      // Select multiplexer channel 0
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  
  unsigned int raw_fl = (cmd[0] << 4) | (cmd[1] >> 4);
  float fl_wall_dis = raw_fl / 64.0f;
  return fl_wall_dis;
}

/**
 * Read front-right IR distance sensor (angled 30°)
 * Used for detecting diagonal obstacles
 * @return Distance in centimeters
 */
float read_fr_ir() {
  const char mux_cmd = 0x08;      // Select multiplexer channel 3
  const char mux_addr = 0xEE;
  i2c.write(mux_addr, &mux_cmd, 1);
  
  char cmd[2];
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1);
  wait_us(1000);
  i2c.read(0x80, cmd, 2);
  
  unsigned int raw_fr = (cmd[0] << 4) | (cmd[1] >> 4);
  float fr_wall_dis = raw_fr / 64.0f;
  return fr_wall_dis;
}

//=============================================================================
// INITIALIZATION
//=============================================================================

/**
 * Initialize all hardware peripherals
 * 
 * Sets up:
 *   - PWM frequency for motor control (500Hz)
 *   - Encoder interrupt handlers
 *   - I2C bus frequency for sensors
 * 
 * Must be called once in setup() before using any hardware
 */
void hw_init() {
  // Set PWM period (2000µs = 500Hz)
  pwma.period_us(2000);
  pwmb.period_us(2000);
  
  // Attach encoder interrupt handlers
  tica.rise(&l_cnt);              // Left encoder rising edge
  ticb.rise(&r_cnt);              // Right encoder rising edge
  
  // Set I2C frequency for sensor communication
  i2c.frequency(100000);          // 100kHz standard mode
}
