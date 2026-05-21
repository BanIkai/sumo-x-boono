/*
 * strategy.h — State machine & robot behaviour
 *
 *  ┌────────┐  start   ┌──────────┐   found    ┌─────────┐
 *  │  IDLE  │────────→│  SEARCH  │───────────→│  TRACK  │
 *  └────────┘          └──────────┘            └────┬────┘
 *                           ↑          lost / far   │  close
 *                           │         ┌──────────────┘
 *                           │         ▼
 *                           │    ┌─────────┐  opponent  ┌──────────┐
 *                           │    │ ATTACK  │←───────────│  DEFEND  │
 *                           │    └────┬────┘            └──────────┘
 *                           │         │ side                  │
 *                           └─────────┘  after spin/ lost    │
 *
 *   Edge detection overrides everything and always runs first.
 */

#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensors.h"
#include "motors.h"

// =====================================================
//  State enum
// =====================================================

enum State {
    STATE_IDLE,
    STATE_SEARCH,
    STATE_TRACK,
    STATE_ATTACK,
    STATE_DEFEND,
    STATE_EDGE_AVOID
};

// =====================================================
//  Module-level variables
// =====================================================

static State         gState       = STATE_IDLE;
static State         gPrevState   = STATE_IDLE;
static unsigned long gStateTimer  = 0;
static SensorData    gSens;

static bool          gSearchCW    = true;
static unsigned long gSearchFlip  = 0;
static bool          gEdgeHandled = false;

// =====================================================
//  Internal helpers
// =====================================================

static inline void setState(State newState) {
    if (newState != gState) {
        gState      = newState;
        gStateTimer = millis();
    }
}

static void handleIdle();
static void handleSearch();
static void handleTrack();
static void handleAttack();
static void handleDefend();
static void handleEdgeAvoid();

// =====================================================
//  runStrategy()
// =====================================================

static inline void runStrategy() {
    // ----- immediate kill -----
    if (digitalRead(KILL_PIN) == HIGH) {
        setState(STATE_IDLE);
        stopMotors();
        return;
    }

    // ----- read sensors -----
    readSensors(gSens);

    // ----- edge always has priority — ขัดจังหวะทุก state ทันที -----
    if (isOnEdge(gSens)) {
        if (!gEdgeHandled) {
            gPrevState   = gState;
            gEdgeHandled = true;
            setState(STATE_EDGE_AVOID);
        }
        handleEdgeAvoid();   // รันทันทีเลย ไม่รอ dispatch
        return;
    }
    gEdgeHandled = false;

    // ----- dispatch -----
    switch (gState) {
        case STATE_IDLE:       handleIdle();       break;
        case STATE_SEARCH:     handleSearch();     break;
        case STATE_TRACK:      handleTrack();      break;
        case STATE_ATTACK:     handleAttack();     break;
        case STATE_DEFEND:     handleDefend();     break;
        case STATE_EDGE_AVOID: handleEdgeAvoid();  break;
    }
}

// =====================================================
//  IDLE
// =====================================================

static void handleIdle() {
    stopMotors();
    if (digitalRead(START_PIN) == HIGH || digitalRead(BUTTON_PIN) == HIGH) {
        delay(80);
        setState(STATE_SEARCH);
    }
}

// =====================================================
//  SEARCH
// =====================================================

static void handleSearch() {
    if (gSens.detected) {
        setState(STATE_TRACK);
        return;
    }

    if (millis() - gSearchFlip > SEARCH_ROTATE_TIME) {
        gSearchCW   = !gSearchCW;
        gSearchFlip = millis();
    }

    int slowSearch = SPEED_SEARCH / 2;
    int turnBias   = gSearchCW ? -slowSearch : slowSearch;
    drive(0, turnBias);
}

// =====================================================
//  TRACK
// =====================================================

static void handleTrack() {
    if (!gSens.detected || gSens.maxReading < DIST_LOST_THRESH) {
        setState(STATE_SEARCH);
        return;
    }

    if (gSens.canRam) {
        setState(STATE_ATTACK);
        return;
    }

    if (gSens.atSide) {
        setState(STATE_DEFEND);
        return;
    }

    int turnBias = (int)(gSens.direction * TRACK_KP);
    turnBias = constrain(turnBias, -MAX_TURN_BIAS, MAX_TURN_BIAS);

    float absDir = abs(gSens.direction);
    int baseSpeed;
    if (absDir < 15.0f)      baseSpeed = SPEED_HIGH;
    else if (absDir < 45.0f) baseSpeed = SPEED_MED;
    else if (absDir < 70.0f) baseSpeed = SPEED_LOW;
    else                     baseSpeed = 0;

    drive(baseSpeed, turnBias);
}

// =====================================================
//  ATTACK
// =====================================================

static void handleAttack() {
    if (!gSens.detected || gSens.maxReading < DIST_DETECT_THRESH) {
        setState(STATE_TRACK);
        return;
    }

    if (millis() - gStateTimer > ATTACK_TIMEOUT) {
        if (gSens.canRam) {
            gStateTimer = millis();
        } else {
            setState(STATE_TRACK);
            return;
        }
    }

    int turnBias = (int)(gSens.direction * ATTACK_KP);
    turnBias = constrain(turnBias, -60, 60);

    drive(SPEED_MAX, turnBias);
}

// =====================================================
//  DEFEND
// =====================================================

static void handleDefend() {
    if (gSens.detected && abs(gSens.direction) < 45.0f &&
        gSens.cf > DIST_DETECT_THRESH) {
        setState(STATE_TRACK);
        return;
    }

    if (!gSens.detected) {
        setState(STATE_SEARCH);
        return;
    }

    if (millis() - gStateTimer > DEFEND_SPIN_TIME) {
        setState(STATE_SEARCH);
        return;
    }

    int turnBias = (gSens.sideDirection == -1) ? SPEED_MAX : -SPEED_MAX;
    bool shake = ((millis() / 100) % 2 == 0);
    drive(0, shake ? turnBias : -turnBias / 2);
}

// =====================================================
//  EDGE_AVOID — ถอยหลัง แล้วหมุนกลับเข้าสนาม
//  el < EDGE_THRESH  →  ขอบซ้าย  →  หมุนขวา  (turnBias บวก)
//  er < EDGE_THRESH  →  ขอบขวา   →  หมุนซ้าย (turnBias ลบ)
// =====================================================

static void handleEdgeAvoid() {
    unsigned long elapsed = millis() - gStateTimer;

    bool leftEdge  = (gSens.el < EDGE_THRESH);
    bool rightEdge = (gSens.er < EDGE_THRESH);

    if (elapsed < EDGE_BACKUP_TIME) {
        // Phase 1: ถอยหลัง
        drive(-SPEED_MED, 0);

    } else if (elapsed < EDGE_BACKUP_TIME + EDGE_TURN_TIME) {
        // Phase 2: หมุนกลับเข้าสนาม
        if (leftEdge && rightEdge) {
            drive(0,  SPEED_SEARCH);   // ขอบทั้งสอง → หมุนขวา
        } else if (leftEdge) {
            drive(0,  SPEED_SEARCH);   // ขอบซ้าย → หมุนขวา
        } else {
            drive(0, -SPEED_SEARCH);   // ขอบขวา → หมุนซ้าย
        }

    } else {
        // Phase 3: จบแล้วกลับ state เดิม
        setState(gPrevState);
    }
}
