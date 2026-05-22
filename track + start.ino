/*
 * track_only.ino — Standalone tracking-only test
 *
 * ไม่มีวัตถุ → หมุนหาช้าๆ
 * เจอวัตถุ   → track เข้าหา
 *
 * ─── การแก้ไข ───────────────────────────────────────────
 *  • เพิ่ม start gate — รอ START_PIN หรือ BUTTON_PIN ก่อน
 *    หุ่นไม่ทำงานทันทีตอนเปิดเครื่อง
 *  • เซนเซอร์กันตกอ่านค่าทันทีทุก loop โดยใช้ s.el / s.er
 *    ที่ readSensors() เก็บไว้แล้ว ไม่ต้อง analogRead() ซ้ำ
 *  • ลบ delay(100) ออก ใช้ non-blocking timer แทน
 *    → loop วิ่งเร็วเต็มที่, edge ตอบสนองทันที
 *  • เซนเซอร์ระยะ 5 ตัวไม่ถูกแตะเพิ่มเลย (readSensors เดิม)
 * ────────────────────────────────────────────────────────
 *
 * Upload this instead of sumo_robot.ino.  Open Serial Monitor at 9600 baud.
 */
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

static bool          gStarted        = false;   // [แก้ไข] start gate
static bool          gTracking        = false;
static bool          gSearchCW        = true;
static unsigned long gSearchFlip      = 0;

// --- non-blocking timer แทน delay(100) ---
static unsigned long gLastPrintTime   = 0;
#define PRINT_INTERVAL_MS  100          // พิมพ์ Serial ทุก 100 ms โดยไม่หยุด loop

void setup() {
    initMotors();
    delay(200);
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("=== TRACKING ONLY ==="));
    Serial.println(F("รอสัญญาณ START..."));
    Serial.println(F("Dir   Spd  Turn  [  LS   LF   CF   RS ]  EdgeL  EdgeR"));
    Serial.println(F("----  ---  ----  ---------------------   -----  -----"));
}

void loop() {

    // ─── 0. [แก้ไข] รอสัญญาณก่อน ────────────────────────────
    if (digitalRead(START_PIN) == HIGH || digitalRead(BUTTON_PIN) == HIGH) {
        if (!gStarted) {
            gStarted = true;
            Serial.println(F("=== START! ==="));
        }
    }
    if (!gStarted) {
        stopMotors();
        return;
    }

    // ─── 1. อ่านเซนเซอร์ทั้งหมดครั้งเดียว ───────────────────
    SensorData s;
    readSensors(s);

    // ─── 2. ตรวจ edge ทันที ─────────────────────────────────
    bool onEdge = isOnEdge(s);

    if (onEdge) {
        stopMotors();
        if (millis() - gLastPrintTime >= PRINT_INTERVAL_MS) {
            gLastPrintTime = millis();
            Serial.print(F("*** EDGE ***  L="));
            Serial.print(s.el);
            Serial.print(F("  R="));
            Serial.println(s.er);
        }
        return;
    }

    // ─── 3. ไม่เจอวัตถุ → หมุนหาช้าๆ ───────────────────────
    if (!s.detected || s.maxReading < DIST_LOST_THRESH) {
        if (gTracking) {
            Serial.println(F("LOST — หมุนหา..."));
            gTracking = false;
        }

        if (millis() - gSearchFlip > SEARCH_ROTATE_TIME) {
            gSearchCW   = !gSearchCW;
            gSearchFlip = millis();
        }

        int turnBias = gSearchCW ? -(SPEED_SEARCH / 2) : (SPEED_SEARCH / 2);
        drive(0, turnBias);

        if (millis() - gLastPrintTime >= PRINT_INTERVAL_MS) {
            gLastPrintTime = millis();
            Serial.print(F("SEARCH  EdgeL="));
            Serial.print(s.el);
            Serial.print(F("  EdgeR="));
            Serial.println(s.er);
        }
        return;
    }

    // ─── 4. เจอวัตถุ → track เข้าหา ─────────────────────────
    if (!gTracking) {
        Serial.println(F("FOUND — tracking!"));
        gTracking = true;
    }

    int turnBias = (int)(s.direction * TRACK_KP);
    turnBias = constrain(turnBias, -MAX_TURN_BIAS, MAX_TURN_BIAS);

    float absDir = abs(s.direction);
    int baseSpeed;
    if      (absDir < 15.0f) baseSpeed = SPEED_HIGH;
    else if (absDir < 45.0f) baseSpeed = SPEED_MED;
    else if (absDir < 70.0f) baseSpeed = SPEED_LOW;
    else                     baseSpeed = 0;

    drive(baseSpeed, turnBias);

    static int lastSpeed = -1;
    static int lastTurn  = -999;
    if (baseSpeed != lastSpeed || turnBias != lastTurn ||
        millis() - gLastPrintTime >= PRINT_INTERVAL_MS) {

        gLastPrintTime = millis();
        lastSpeed = baseSpeed;
        lastTurn  = turnBias;

        Serial.print(F("TRACK  dir="));
        Serial.print((int)s.direction);
        Serial.print(F("  spd="));
        Serial.print(baseSpeed);
        Serial.print(F("  turn="));
        Serial.print(turnBias);
        Serial.print(F("  [LS="));
        Serial.print(s.ls);
        Serial.print(F(" LF="));
        Serial.print(s.lf);
        Serial.print(F(" CF="));
        Serial.print(s.cf);
        Serial.print(F(" RS="));
        Serial.print(s.rs);
        Serial.print(F("]  EdgeL="));
        Serial.print(s.el);
        Serial.print(F("  EdgeR="));
        Serial.println(s.er);
    }
}
