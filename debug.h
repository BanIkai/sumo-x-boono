/*
 * debug.h — Interactive test / calibration tools via Serial monitor
 *
 * Menu:
 *   1 — Print sensor values once
 *   2 — Continuous sensor readout (press any key to stop)
 *   3 — Motor test sequence (cycles through speeds & directions)
 *   4 — Manual motor control  (W/S fwd/rev, A/D steer, Q/E spin, 1-4 speed)
 *   5 — Run normal match strategy
 *   0 — Stop motors
 */

#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

// =====================================================
//  Show a single sensor snapshot
// =====================================================

static void printSensorsOnce() {
    SensorData s;
    readSensors(s);

    Serial.println(F("-------------------------------------------------"));
    Serial.println(F("  SENSOR      RAW       ~cm      STATUS"));
    Serial.println(F("-------------------------------------------------"));

    Serial.print(F("  LSide   "));  Serial.print(s.ls); Serial.print(F("\t    "));
    Serial.print(rawToCm(s.ls), 1); Serial.print(F("\t  "));
    if (s.ls > DIST_SIDE_THREAT) Serial.println(F("THREAT"));
    else if (s.ls > DIST_DETECT_THRESH) Serial.println(F("detected"));
    else Serial.println(F("-"));

    Serial.print(F("  LFront  "));  Serial.print(s.lf); Serial.print(F("\t    "));
    Serial.print(rawToCm(s.lf), 1); Serial.print(F("\t  "));
    if (s.lf > DIST_ATTACK_THRESH) Serial.println(F("RAM"));
    else if (s.lf > DIST_DETECT_THRESH) Serial.println(F("track"));
    else Serial.println(F("-"));

    Serial.print(F("  CFront  "));  Serial.print(s.cf); Serial.print(F("\t    "));
    Serial.print(rawToCm(s.cf), 1); Serial.print(F("\t  "));
    if (s.cf > DIST_ATTACK_THRESH) Serial.println(F("RAM"));
    else if (s.cf > DIST_DETECT_THRESH) Serial.println(F("track"));
    else Serial.println(F("-"));

    Serial.print(F("  RFront  "));
    Serial.print(F("DISABLED"));

    Serial.print(F("  RSide   "));  Serial.print(s.rs); Serial.print(F("\t    "));
    Serial.print(rawToCm(s.rs), 1); Serial.print(F("\t  "));
    if (s.rs > DIST_SIDE_THREAT) Serial.println(F("THREAT"));
    else if (s.rs > DIST_DETECT_THRESH) Serial.println(F("detected"));
    else Serial.println(F("-"));

    Serial.println(F("-------------------------------------------------"));
    Serial.print(F("  Edge L: ")); Serial.print(s.el);
    Serial.print(F("   Edge R: ")); Serial.print(s.er);
    Serial.print(F("   "));
    if (isOnEdge(s)) Serial.println(F("[EDGE!]"));
    else              Serial.println(F("[arena]"));

    Serial.print(F("  Opponent: "));
    if (s.detected) {
        Serial.print(F("YES  dir="));
        Serial.print(s.direction, 1);
        Serial.print(F(" deg"));
        Serial.print(F("  ram="));
        Serial.print(s.canRam ? F("YES") : F("no"));
        Serial.print(F("  side="));
        Serial.print(s.atSide ? (s.sideDirection == -1 ? F("LEFT") : F("RIGHT")) : F("no"));
        Serial.print(F("  maxRaw="));
        Serial.println(s.maxReading);
    } else {
        Serial.println(F("NO"));
    }
    Serial.println(F("-------------------------------------------------"));
}


// =====================================================
//  Continuous sensor readout (non-blocking, key to stop)
// =====================================================

static void sensorStream() {
    Serial.println(F("\n[Continuous readout — press any key to stop]\n"));

    while (!Serial.available()) {
        printSensorsOnce();
        Serial.println();

        // small delay so the output doesn't flood
        unsigned long start = millis();
        while (millis() - start < 250) {
            if (Serial.available()) goto stopStream;
        }
    }

stopStream:
    while (Serial.available()) Serial.read();   // flush input
    Serial.println(F("\n[Stopped.]\n"));
}


// =====================================================
//  Motor test sequence — automated speed / direction ramp
// =====================================================

static void motorTestSequence() {
    Serial.println(F("\n=== Motor Test Sequence ==="));
    Serial.println(F("Stop motors to abort.\n"));

    // Helper: run for N ms, checking Serial for abort (space bar)
    auto runStep = [](const char *label, int left, int right, unsigned long duration) {
        Serial.print(F("  "));
        Serial.print(label);
        Serial.print(F("  (L="));
        Serial.print(left);
        Serial.print(F(" R="));
        Serial.print(right);
        Serial.println(F(") ..."));

        unsigned long t0 = millis();
        while (millis() - t0 < duration) {
            driveMotorA(left);
            driveMotorB(right);
            if (Serial.available() && Serial.peek() == ' ') {
                Serial.read();                    // consume the space
                stopMotors();
                Serial.println(F("  [ABORTED]"));
                while (Serial.available()) Serial.read();
                return false;
            }
        }
        return true;
    };

    const int speeds[] = { 64, 128, 192, 255 };
    const char *labels[] = { "25%", "50%", "75%", "100%" };

    // ----- Forward ramp -----
    Serial.println(F("-- Forward --"));
    for (int i = 0; i < 4; i++) {
        if (!runStep(labels[i], speeds[i], speeds[i], 800)) return;
    }

    // ----- Reverse ramp -----
    Serial.println(F("-- Reverse --"));
    for (int i = 0; i < 3; i++) {
        if (!runStep(labels[i], -speeds[i], -speeds[i], 800)) return;
    }

    // ----- Spin in place -----
    Serial.println(F("-- Spin Left (CCW) --"));
    if (!runStep("50%",  -128,  128, 1000)) return;
    if (!runStep("100%", -255,  255, 1000)) return;

    Serial.println(F("-- Spin Right (CW) --"));
    if (!runStep("50%",   128, -128, 1000)) return;
    if (!runStep("100%",  255, -255, 1000)) return;

    // ----- Turn while moving forward -----
    Serial.println(F("-- Turn Left @50% fwd --"));
    if (!runStep("50%",    0,  192, 1000)) return;
    Serial.println(F("-- Turn Right @50% fwd --"));
    if (!runStep("50%",  192,    0, 1000)) return;

    stopMotors();
    Serial.println(F("\n[Done — all tests passed.]\n"));
}


// =====================================================
//  Manual motor control via keyboard
//    W/S : forward / reverse
//    A/D : steer left / right
//    Q/E : spin  left / right
//    1-4 : speed level  (1=25%, 2=50%, 3=75%, 4=100%)
//    Space : stop
//    X    : exit to menu
//    H    : print help
// =====================================================

static void manualControl() {
    Serial.println(F("\n=== Manual Motor Control ==="));
    Serial.println(F("  W/S = fwd/rev    A/D = steer L/R    Q/E = spin L/R"));
    Serial.println(F("  1-4 = speed lvl  SPACE = stop      X = exit   H = help\n"));

    int speedLevel = 4;                  // start at 100 %
    const int speedMap[4] = { 64, 128, 192, 255 };

    // Returns PWM duty for the current speed level
    auto spd = [&]() -> int { return speedMap[speedLevel - 1]; };

    while (true) {
        if (!Serial.available()) {
            delay(20);                    // keep loop light when idle
            continue;
        }

        char c = Serial.read();

        switch (c) {
            // ---- speed preset ----
            case '1': speedLevel = 1;  break;
            case '2': speedLevel = 2;  break;
            case '3': speedLevel = 3;  break;
            case '4': speedLevel = 4;  break;

            // ---- forward / reverse ----
            case 'w': case 'W': drive( spd(), 0);         break;
            case 's': case 'S': drive(-spd(), 0);         break;

            // ---- steer ----
            case 'a': case 'A': drive( spd(), -spd());    break;   // turn left  while moving fwd
            case 'd': case 'D': drive( spd(),  spd());    break;   // turn right while moving fwd

            // ---- spin in place ----
            case 'q': case 'Q': drive(0, -spd());         break;   // spin CCW
            case 'e': case 'E': drive(0,  spd());         break;   // spin CW

            // ---- stop / exit ----
            case ' ': stopMotors();                       break;
            case 'x': case 'X':
                stopMotors();
                while (Serial.available()) Serial.read();
                Serial.println(F("[Exited manual control]\n"));
                return;

            // ---- help ----
            case 'h': case 'H':
                Serial.println(F("  Speed: "));
                Serial.print(F("    1=25%  2=50%  3=75%  4=100%   [current: "));
                Serial.print(speedLevel);
                Serial.println(F("]"));
                Serial.println(F("  W/S = fwd/rev     A/D = steer L/R"));
                Serial.println(F("  Q/E = spin L/R    SPACE = stop   X = exit"));
                Serial.println();
                break;

            default: break;
        }

        // Show speed change feedback
        if (c >= '1' && c <= '4') {
            Serial.print(F("  Speed set to "));
            Serial.print(speedLevel);
            Serial.println(F("/4"));
        }
    }
}


// =====================================================
//  Main debug menu — called from setup() when DEBUG_MODE=1
// =====================================================

static void debugMenu() {
    Serial.println(F("\n\n========================================"));
    Serial.println(F("    SUMO ROBOT — TEST / CALIBRATION"));
    Serial.println(F("========================================"));
    Serial.println(F("  1 — Sensor snapshot (once)"));
    Serial.println(F("  2 — Continuous sensor stream"));
    Serial.println(F("  3 — Motor test sequence"));
    Serial.println(F("  4 — Manual motor control"));
    Serial.println(F("  5 — Run normal strategy"));
    Serial.println(F("  0 — Stop motors"));
    Serial.println(F("----------------------------------------"));

    while (true) {
        if (Serial.available()) {
            char c = Serial.read();

            switch (c) {
                case '1': printSensorsOnce();   break;
                case '2': sensorStream();        break;
                case '3': motorTestSequence();   break;
                case '4': manualControl();       break;
                case '5': return;                // exit menu → run strategy
                case '0': stopMotors(); Serial.println(F("Motors stopped.")); break;

                case '\n': case '\r': break;     // ignore newlines
                default:    break;
            }

            // Re-print prompt after action
            if (c != '\n' && c != '\r') {
                Serial.println(F("----------------------------------------"));
                Serial.println(F("  Choose: 1-sensor  2-stream  3-motors  4-manual  5-run  0-stop"));
                Serial.println();
            }
        }

        // Also keep the kill pin active during debug so motors can be killed
        if (digitalRead(KILL_PIN) == HIGH) {
            stopMotors();
            Serial.println(F("\n[KILL signal — motors stopped]\n"));
        }
    }
}
