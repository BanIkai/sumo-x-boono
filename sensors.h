/*
 * sensors.h — Read, filter & interpret all robot sensors
 *
 * Produces a SensorData struct containing raw readings and
 * analysed opponent-position info needed by the strategy.
 */

#pragma once
#include <Arduino.h>
#include "config.h"

// =====================================================
//  Sensor Data — one struct to rule them all
// =====================================================

struct SensorData {
    // ---- raw distance readings (0-1023, higher = closer) ----
    int ls;          // left side    ( 90° left)
    int lf;          // left front   ( 10° left from centre)
    int cf;          // centre front (straight ahead)
    int rf;          // right front  ( 10° right from centre)
    int rs;          // right side   ( 90° right)

    // ---- raw edge readings (0-1023, higher = black) ----
    int el;          // left  edge sensor
    int er;          // right edge sensor

    // ---- analysed opponent info ----
    bool  detected;       // opponent anywhere in view?
    float direction;      // -90° (hard left) … 0° (front) … +90° (hard right)
    int   maxReading;     // highest value across all five distance sensors
    bool  atSide;         // opponent threatening from one side only?
    int   sideDirection;  // -1 = left side,  +1 = right side,  0 = not at side
    bool  canRam;         // opponent front and close enough to ram
};


// =====================================================
//  Helper: read pin N times and return the average
// =====================================================

static inline int analogReadAvg(uint8_t pin, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
    }
    return (int)(sum / samples);
}


// =====================================================
//  Helper: convert raw ADC → approximate cm
//  (Sharp GP2Y0A41SK0F — useful for debugging only)
// =====================================================

static inline float rawToCm(int raw) {
    if (raw <= 15) return 30.0f;         // too far → clamp
    if (raw >= 600) return  3.5f;        // too near → clamp
    return 2076.0f / (raw - 11.0f);
}


// =====================================================
//  Main: read all sensors + analyse opponent position
// =====================================================

static inline void readSensors(SensorData &s) {
    // ----- distance sensors (5 × average) -----
    s.ls = analogReadAvg(DIST_LSIDE,  SENSOR_SAMPLES);
    s.lf = analogReadAvg(DIST_LFRONT, SENSOR_SAMPLES);
    s.cf = analogReadAvg(DIST_CFRONT, SENSOR_SAMPLES);
    s.rf = 0;   // A7 sensor disabled
    s.rs = analogReadAvg(DIST_RSIDE,  SENSOR_SAMPLES);

    // ----- edge sensors (quick 2-sample average) -----
    s.el = analogReadAvg(EDGE_LEFT,  2);
    s.er = analogReadAvg(EDGE_RIGHT, 2);

    // ---------------------------------------------------
    //  Weighted-average direction estimation
    //  Each sensor votes for its angle, weighted by how
    //  much its reading exceeds the detection threshold.
    // ---------------------------------------------------
    struct AngleReading {
        int   value;
        float angle;        // degrees from centre
    };
    AngleReading readings[4] = {
        { s.ls, -90.0f },
        { s.lf, -10.0f },
        { s.cf,   0.0f },
        { s.rs,  90.0f }
    };

    float weightedDir  = 0.0f;
    float totalWeight  = 0.0f;
    s.maxReading       = 0;

    for (int i = 0; i < 4; i++) {
        int val = readings[i].value;
        if (val > s.maxReading) { s.maxReading = val; }

        if (val > DIST_DETECT_THRESH) {
            // weight = how much ABOVE threshold (closer = stronger vote)
            float w = (float)(val - DIST_DETECT_THRESH);
            weightedDir += readings[i].angle * w;
            totalWeight += w;
        }
    }

    s.detected  = (totalWeight > 0.0f);
    s.direction = s.detected ? (weightedDir / totalWeight) : 0.0f;

    // ---------------------------------------------------
    //  Side-threat detection
    //  "atSide" only when a side sensor fires strongly
    //  AND the centre sensor sees nothing.
    // ---------------------------------------------------
    bool leftThreat  = (s.ls > DIST_SIDE_THREAT) && (s.cf < DIST_DETECT_THRESH);
    bool rightThreat = (s.rs > DIST_SIDE_THREAT) && (s.cf < DIST_DETECT_THRESH);

    if (leftThreat && !rightThreat) {
        s.atSide       = true;
        s.sideDirection = -1;                   // opponent on left
    } else if (rightThreat && !leftThreat) {
        s.atSide       = true;
        s.sideDirection =  1;                   // opponent on right
    } else {
        s.atSide       = false;
        s.sideDirection =  0;
    }

    // ---------------------------------------------------
    //  Attack-range check — is any front sensor HOT?
    // ---------------------------------------------------
    s.canRam = (s.lf > DIST_ATTACK_THRESH) ||
               (s.cf > DIST_ATTACK_THRESH);
}


// =====================================================
//  Edge detection — are we on the black border?
// =====================================================

static inline bool isOnEdge(const SensorData &s) {
    return (s.el < EDGE_THRESH) || (s.er < EDGE_THRESH);
}
