/*
 * track_only.ino — Standalone tracking-only test
 *
 * ไม่มีวัตถุ → หมุนหาช้าๆ
 * เจอวัตถุ   → track เข้าหา
 *
 * Upload this instead of sumo_robot.ino.  Open Serial Monitor at 9600 baud.
 */
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

static bool          gTracking   = false;
static bool          gSearchCW   = true;
static unsigned long gSearchFlip = 0;

void setup() {
    Serial.begin(9600);
    /*
    initMotors();
    delay(200);
    Serial.begin(SERIAL_BAUD);
    Serial.println(F("=== TRACKING ONLY ==="));
    Serial.println(F("ไม่เจอ → หมุนหา  |  เจอ → track"));
    Serial.println(F("Dir   Spd  Turn  [  LS   LF   CF   RS ]"));
    Serial.println(F("----  ---  ----  ---------------------"));
    */
}

void loop() {
  
    SensorData s;
    readSensors(s);

    // =====================================================
    //  ไม่เห็นวัตถุ → หมุนหาช้าๆ
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

        int turnBias = gSearchCW ? -(SPEED_SEARCH / 2) : (SPEED_SEARCH / 2);
        drive(0, turnBias);

        
        Serial.print("R=");
        Serial.print(analogRead(EDGE_RIGHT));
        Serial.print(" ");
        Serial.print("L=");
        Serial.print(analogRead(EDGE_LEFT));
        Serial.println("");
        
        if((analogRead(EDGE_LEFT)<500)||((analogRead(EDGE_RIGHT))<500))
        {
            Serial.print("STOP");
        }
        delay(100);
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
    if (absDir < 15.0f)      baseSpeed = SPEED_HIGH;
    else if (absDir < 45.0f) baseSpeed = SPEED_MED;
    else if (absDir < 70.0f) baseSpeed = SPEED_LOW;
    else                     baseSpeed = 0;

    drive(baseSpeed, turnBias);

    static int lastSpeed = -1;
    static int lastTurn  = -999;
    if (baseSpeed != lastSpeed || turnBias != lastTurn) {
        lastSpeed = baseSpeed;
        lastTurn  = turnBias;


        
    }
    
}