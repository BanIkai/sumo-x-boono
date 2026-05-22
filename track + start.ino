/*
 * track_only.ino — Standalone tracking-only test
 *
 * ไม่มีวัตถุ → หมุนหาช้าๆ
 * เจอวัตถุ   → track เข้าหา
 * ค่า edge sensor < 800 → ถอย → เลี้ยว → พุ่ง → เลี้ยว → พุ่ง → ทำงานต่อปกติ
 *
 * START PIN (IR Remote):
 *   pulse HIGH ครั้งแรก = เริ่มทำงาน
 *   pulse HIGH ครั้งที่สอง = หยุด
 */
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

static bool          gTracking         = false;
static bool          gSearchCW         = true;
static unsigned long gSearchFlip       = 0;

// --- START PIN toggle (IR pulse HIGH) ---
static bool          start             = false;
static bool          lastStartPin      = LOW;
static unsigned long lastStartDebounce = 0;
#define DEBOUNCE_MS 50

// =====================================================
//  ฟังก์ชันหันกลับเข้าสนาม
//  ถอย → เลี้ยว → พุ่ง → เลี้ยว → พุ่ง
// =====================================================
void avoidEdge(bool leftEdge, bool rightEdge) {
    int turnDir = (rightEdge && !leftEdge) ? -1 : 1;

    Serial.println(F("[EDGE] ถอยหลัง..."));
    drive(-SPEED_HIGH, 0);
    delay(300);
    stopMotors();
    delay(20);

    Serial.println(F("[EDGE] เลี้ยว 1..."));
    drive(0, turnDir * SPEED_SEARCH);
    delay(280);
    stopMotors();
    delay(20);

    Serial.println(F("[EDGE] พุ่ง 1..."));
    drive(SPEED_HIGH, 0);
    delay(200);
    stopMotors();
    delay(20);

    Serial.println(F("[EDGE] เลี้ยว 2..."));
    drive(0, turnDir * SPEED_SEARCH);
    delay(180);
    stopMotors();
    delay(20);

    Serial.println(F("[EDGE] พุ่ง 2..."));
    drive(SPEED_HIGH, 0);
    delay(200);
    stopMotors();

    Serial.println(F("[EDGE] กลับเข้าสนามแล้ว — ทำงานต่อ"));
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(START_PIN, INPUT);
    initMotors();
    delay(200);
    Serial.println(F("=== TRACKING ONLY ==="));
    Serial.println(F("IR pulse ครั้งแรก = เริ่ม  |  IR pulse อีกครั้ง = หยุด"));
    Serial.println(F("ไม่เจอ → หมุนหา  |  เจอ → track  |  edge → ถอย/เลี้ยว/พุ่ง"));
}

void loop() {
    if (digitalRead(KILL_PIN) == HIGH) {
        stopMotors();
        Serial.println(F("[KILL] หยุดฉุกเฉิน — รีเซ็ตเพื่อเริ่มใหม่"));
        while (true);
    }

    // =====================================================
    //  ตรวจ edge sensor ก่อนทุกอย่าง
    // =====================================================
    int edgeL = analogRead(EDGE_LEFT);
    int edgeR = analogRead(EDGE_RIGHT);

    Serial.print(F("R=")); Serial.print(edgeR);
    Serial.print(F("  L=")); Serial.println(edgeL);

    if (edgeL < EDGE_THRESH || edgeR < EDGE_THRESH) {
        stopMotors();
        avoidEdge(edgeL < EDGE_THRESH, edgeR < EDGE_THRESH);
        return;
    }

    // =====================================================
    //  ตรวจสัญญาณ START — IR pulse HIGH → toggle
    // =====================================================
    bool currentStartPin = (digitalRead(START_PIN) == HIGH);
    if (currentStartPin && !lastStartPin) {           // rising edge (จับ pulse)
        if (millis() - lastStartDebounce > DEBOUNCE_MS) {
            start = !start;
            lastStartDebounce = millis();
            if (start) {
                Serial.println(F("[START] เริ่มทำงาน!"));
                drive(SPEED_LOW, 0);
                delay(400);
            } else {
                Serial.println(F("[START] หยุดโปรแกรม — ส่ง IR อีกครั้งเพื่อเริ่มใหม่"));
                stopMotors();
            }
        }
    }
    lastStartPin = currentStartPin;

    if (!start) {
        stopMotors();
        return;
    }

    // =====================================================
    //  อ่านเซนเซอร์ระยะ
    // =====================================================
    SensorData s;
    readSensors(s);

    // ยืนยันขอบอีกรอบหลัง avoidEdge
    {
        int reCheckL = analogRead(EDGE_LEFT);
        int reCheckR = analogRead(EDGE_RIGHT);
        if (reCheckL < EDGE_THRESH || reCheckR < EDGE_THRESH) {
            stopMotors();
            avoidEdge(reCheckL < EDGE_THRESH, reCheckR < EDGE_THRESH);
            return;
        }
    }

    // =====================================================
    //  ไม่เห็นวัตถุ → หมุนหา
    // =====================================================
    if (!s.detected || s.maxReading < DIST_LOST_THRESH) {
        if (gTracking) {
            Serial.println(F("LOST — หมุนหา..."));
            gTracking = false;
        }

        if (millis() - gSearchFlip > SEARCH_ROTATE_TIME) {
            gSearchCW   = !gSearchCW;
            gSearchFlip = millis();
        }

        int turnBias = gSearchCW ? -(SPEED_SEARCH * 3 / 4) : (SPEED_SEARCH * 3 / 4);
        drive(0, turnBias);
        return;
    }

    // =====================================================
    //  เจอวัตถุ → track เข้าหา
    // =====================================================
    if (!gTracking) {
        Serial.println(F("FOUND — tracking!"));
        gTracking = true;
    }

    int turnBias = (int)(s.direction * TRACK_KP);
    turnBias = constrain(turnBias, -MAX_TURN_BIAS, MAX_TURN_BIAS);

    float absDir = abs(s.direction);
    int baseSpeed;
    if      (absDir < 15.0f) baseSpeed = SPEED_MAX;
    else if (absDir < 35.0f) baseSpeed = SPEED_HIGH;
    else if (absDir < 60.0f) baseSpeed = SPEED_MED;
    else if (absDir < 75.0f) baseSpeed = SPEED_LOW;
    else                     baseSpeed = 0;

    drive(baseSpeed, turnBias);
}
