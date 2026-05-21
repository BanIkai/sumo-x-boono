/*
 * motors.h — Motor control via A4950 H-bridge driver
 *
 * A4950 truth table (per motor):
 *   IN1=HIGH  IN2=LOW   →  Forward
 *   IN1=LOW   IN2=HIGH  →  Reverse
 *   IN1=LOW   IN2=LOW   →  Coast   (free-wheel)
 *   IN1=HIGH  IN2=HIGH  →  Brake   (terminals shorted)
 *
 * PWM on the active IN pin controls speed.
 */

#pragma once
#include <Arduino.h>
#include "config.h"


// =====================================================
//  Initialisation
// =====================================================

static inline void initMotors() {
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);

    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
}


// =====================================================
//  Single-motor primitives  (speed range   -255 … 255)
// =====================================================

static inline void driveMotorA(int speed) {   // Left motor
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        analogWrite(MOTOR_A_IN1,  speed);
        digitalWrite(MOTOR_A_IN2, LOW);
    } else if (speed < 0) {
        digitalWrite(MOTOR_A_IN1, LOW);
        analogWrite(MOTOR_A_IN2, -speed);
    } else {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
    }
}

static inline void driveMotorB(int speed) {   // Right motor
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        analogWrite(MOTOR_B_IN2,  speed);
        digitalWrite(MOTOR_B_IN1, LOW);
    } else if (speed < 0) {
        digitalWrite(MOTOR_B_IN2, LOW);
        analogWrite(MOTOR_B_IN1, -speed);
    } else {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
    }
}


// =====================================================
//  High-level differential drive
//
//    left  = baseSpeed - turnBias
//    right = baseSpeed + turnBias
//
//  positive turnBias  →  turn right
//  negative turnBias  →  turn left
//  baseSpeed = 0 with large turnBias  →  spin in place
// =====================================================

static inline void drive(int baseSpeed, int turnBias) {
    int left  = baseSpeed - turnBias;
    int right = baseSpeed + turnBias;

    left  = constrain(left,  -255, 255);
    right = constrain(right, -255, 255);

    driveMotorA(left);    // left  motor
    driveMotorB(right);   // right motor
}


// =====================================================
//  Stop helpers
// =====================================================

static inline void stopMotors() {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
}

static inline void brakeMotors() {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, HIGH);
}
