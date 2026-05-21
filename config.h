/*
 * config.h — Sumo Robot Pin Assignments & Tunable Constants
 *
 * Fill in your actual pin numbers below.
 * All tunable thresholds are listed with their default values.
 */

#pragma once
#include <Arduino.h>

// =====================================================
//  PIN ASSIGNMENTS
//  Replace these with your actual wiring.
// =====================================================

// --- Motor Driver (A4950 ×2) ---
// Left motor (A):  AIN1 for fwd, AIN2 for rev
// Right motor (B): BIN1 for fwd, BIN2 for rev
// Use PWM-capable pins for speed control.
// ------ Pin 5  (~980Hz PWM)
#define MOTOR_A_IN1    5
// ------ Pin 6  (~980Hz PWM)
#define MOTOR_A_IN2    6
// ------ Pin 9  (~490Hz PWM)
#define MOTOR_B_IN1    10
// ------ Pin 10 (~490Hz PWM)
#define MOTOR_B_IN2    11

// --- Distance Sensors (Sharp GP2Y0A41SK0F  4-30 cm, analog out) ---
//     Higher raw ADC = object closer   |   ADC values are 0-1023
// ------ Points 90° left
#define DIST_LSIDE    A5
// ------ Points 10° left  from center
#define DIST_LFRONT   A4
// ------ Points straight ahead
#define DIST_CFRONT   A2
// ------ Points 10° right from center (DISABLED — not read)
#define DIST_RFRONT   A7
// ------ Points 90° right
#define DIST_RSIDE    A1

// --- Edge / Reflective Sensors (TCRT5000, analog out) ---
//     Higher ADC = darker surface  (black border → ~700-900, white arena → ~60-200)
//     Calibrate by reading both surfaces and picking a midpoint threshold.
#define EDGE_LEFT     A3
#define EDGE_RIGHT    A6

// --- Control Signals ---
//     Start pin from match-start module  (HIGH = match starts)
#define START_PIN     A0
//     Kill / emergency-stop signal       (HIGH = immediate stop)
#define KILL_PIN      3
//     Multipurpose button                (LOW→HIGH edge = start, for testing)
#define BUTTON_PIN    4


// =====================================================
//  MOTOR SPEEDS  (PWM duty-cycle  0-255)
// =====================================================

#define SPEED_MAX        255   // Full throttle (ramming)
#define SPEED_HIGH       220   // Fast tracking
#define SPEED_MED        180   // Normal tracking
#define SPEED_LOW        130   // Slow approach / gentle moves
#define SPEED_SEARCH     160   // Rotational speed during search


// =====================================================
//  DISTANCE THRESHOLDS  (raw ADC  0-1023)
//
//  Approx. distance reference for GP2Y0A41SK0F @ 5 V:
//    4 cm → ~530   |   6 cm → ~357   |  10 cm → ~219
//   15 cm → ~149   |  20 cm → ~115   |  30 cm → ~80
// =====================================================

// Sensor reads above this → opponent is close enough to start tracking (~22 cm)
#define DIST_DETECT_THRESH    100

// Sensor reads above this → opponent is in ram / attack range (~7 cm)
#define DIST_ATTACK_THRESH    300

// Sensor reads below this → opponent considered lost (~30 cm)
#define DIST_LOST_THRESH       85

// Side sensor reads above this → opponent is pressing from the side (~9 cm)
#define DIST_SIDE_THREAT      250


// =====================================================
//  EDGE THRESHOLDS  (raw ADC  0-1023)
//  ADC > EDGE_THRESH  →  black-border detected
// =====================================================

#define EDGE_THRESH          200


// =====================================================
//  SENSOR FILTERING
// =====================================================

// Number of analogRead() calls averaged together per sensor
#define SENSOR_SAMPLES         5


// =====================================================
//  TIMING  (milliseconds)
// =====================================================

// How long to rotate one way before switching during SEARCH
#define SEARCH_ROTATE_TIME    700
int lastSearchChange = 0 ;
int searchDirection = 0;
// Maximum ram duration before re-evaluating
#define ATTACK_TIMEOUT       1200

// Spin duration to shake opponent off our side
#define DEFEND_SPIN_TIME      400

// Reverse duration when avoiding the edge
#define EDGE_BACKUP_TIME      300

// Turn duration after backing away from the edge
#define EDGE_TURN_TIME        500


// =====================================================
//  DEBUG / TEST MODE
//  Set to 1 to enter a Serial-monitor test menu at boot.
//  The menu lets you read sensor values, test motors at
//  various speeds, and manually drive the robot.
//  Set to 0 for normal match behaviour.
// =====================================================

#define DEBUG_MODE         0     // 0 = match, 1 = test menu
#define SERIAL_BAUD      9600   // baud rate for Serial monitor


// =====================================================
//  TRACKING / STEERING GAINS
// =====================================================

// Proportional gain for steering:  turnBias = direction_angle × TRACK_KP
#define TRACK_KP            -3.5f

// Steering gain during ATTACK (lower = keeps more forward momentum)
#define ATTACK_KP           1.5f

// Maximum speed difference between left & right motors
#define MAX_TURN_BIAS         200
