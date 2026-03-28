#pragma once
// planner_core.h — Portable motion planner core
//
// Pure-math trajectory planning with no Arduino/FreeRTOS dependencies.
// Can be compiled and tested on any C++11+ platform (PC native, ESP32, etc.).
//
// Contains:
//   - MotionCommand / PlannedBlock structs
//   - TrajectoryPlanner — single-block S-curve/trapezoidal profile generator
//   - planChain()       — Marlin-style forward+reverse pass velocity optimizer

#include <cmath>

namespace planner {

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

struct MotionCommand {
    long  distance;      // Relative distance or absolute target in microsteps
    float acceleration;  // Max acceleration  (microsteps/sec²)
    float maxSpeed;      // Max velocity       (microsteps/sec)
    bool  absolute;      // true → distance is an absolute position
    bool  chain;         // true → do not stop at move end; transition to next command
};

// Pre-planned motion block produced by planChain().
// entry/exit velocities are globally optimal (Marlin-style forward+reverse pass).
struct PlannedBlock {
    float dist;      // unsigned distance (steps)
    float entryVel;  // speed at block start (steps/s, >= 0)
    float cruiseVel; // maximum speed within block (steps/s)
    float exitVel;   // speed at block end (steps/s, >= 0)
    float accel;     // acceleration magnitude (steps/s²)
    bool  forward;   // true = positive direction
};

// S-curve jerk configuration passed to TrajectoryPlanner::resetForBlock().
// Decoupled from global state so the planner core is stateless/testable.
//
// Priority: jerkAbsolute > jerkRampSeconds > auto (maxA * 100, ~10ms ramp).
// Absolute jerk (µsteps/s³) is preferred because at higher accelerations the
// ramp time lengthens automatically, producing smoother mechanical transitions
// without the user having to retune per-move.
struct JerkConfig {
    float jerkAbsolute;     // absolute jerk (µsteps/s³); 0 = use rampSeconds or auto
    float jerkRampSeconds;  // ramp time (s); 0 = auto
};

static constexpr int MAX_CHAIN_LEN = 32;

// ---------------------------------------------------------------------------
// Trajectory Planner
//
// Executes a single pre-planned block (produced by planChain()).  Entry and
// exit velocities are guaranteed achievable by the planner — no reactive
// junction clamping needed here.  The planner runs a simple trapezoidal
// (S-curve smoothed) profile:
//
//   Phase 1  – accelerate from entryVel toward cruiseVel
//   Phase 2  – coast at cruiseVel (may be absent for short segments)
//   Phase 3  – decelerate from cruiseVel to exitVel
//
// isComplete() fires on a simple position crossing — no velocity window.
// Any small velocity residual at the crossing is carried into the next block
// and corrected by the PD control loop.
// ---------------------------------------------------------------------------
class TrajectoryPlanner {
public:
    float currentPos = 0;
    float currentVel = 0;
    float currentAcc = 0;
    float targetPos  = 0;

private:
    float cruiseVel  = 0;
    float exitVel    = 0;
    float maxA       = 0;
    float jerk       = 0;
    bool  moveForward = true;

public:
    // Reset for a new pre-planned block.
    // currentPos/currentVel/currentAcc carry over from the previous block
    // (velocity continuity). The caller must zero currentVel before the
    // very first block.
    void resetForBlock(float startPos, float endPos, const PlannedBlock& blk,
                       const JerkConfig& jcfg) {
        currentPos  = startPos;
        targetPos   = endPos;
        cruiseVel   = blk.cruiseVel;
        exitVel     = blk.exitVel;
        maxA        = (blk.accel > 1.0f) ? blk.accel : 1.0f;
        if (jcfg.jerkAbsolute > 0.0f) {
            jerk = jcfg.jerkAbsolute;              // absolute µsteps/s³ (preferred)
        } else if (jcfg.jerkRampSeconds > 0.0f) {
            jerk = maxA / jcfg.jerkRampSeconds;    // ramp-time mode (legacy)
        } else {
            jerk = maxA * 100.0f;                  // auto (~10 ms ramp, essentially trapezoidal)
        }
        moveForward = blk.forward;
        currentAcc  = 0;
        // currentVel intentionally preserved for velocity continuity
    }

    // Update the exit velocity of the currently executing block.
    // Called when incrementalPlan() revises the ring after a new command arrives.
    // The trajectory planner adjusts its braking profile on the next update().
    void updateExitVel(float newExitVel) {
        exitVel = newExitVel;
    }

    void update(float dt) {
        float distToTarget = std::fabs(targetPos - currentPos);
        float spd          = std::fabs(currentVel);

        // Braking distance needed to decelerate from current speed to exitVel.
        float brakeDist = 0;
        if (spd > exitVel) {
            brakeDist = (spd * spd - exitVel * exitVel) / (2.0f * maxA);
        }

        float targetAcc = 0.0f;

        if (distToTarget < 5.0f && std::fabs(spd - exitVel) < 20.0f) {
            // Damping zone: servo velocity to exitVel; if stopped short, nudge toward target.
            float signedExit = moveForward ? exitVel : -exitVel;
            targetAcc = (signedExit - currentVel) * 10.0f;
            // If motor has stopped short of target add a gentle position-proportional nudge
            // so isComplete() can fire rather than hanging at ~0 velocity.
            if (spd < 10.0f && distToTarget > 0.1f) {
                float nudge = moveForward ? (distToTarget * 300.0f) : -(distToTarget * 300.0f);
                targetAcc += nudge;
            }
            if (std::fabs(targetAcc) > maxA) targetAcc = (targetAcc > 0) ? maxA : -maxA;
        } else if (brakeDist >= distToTarget - _brakeLookahead(spd)) {
            // Start braking with S-curve-aware lookahead (see _brakeLookahead).
            targetAcc = (currentVel > 0) ? -maxA : maxA;
        } else if (spd < cruiseVel) {
            // Accelerate to cruise speed
            targetAcc = moveForward ? maxA : -maxA;
        }
        // else: coast at cruiseVel

        // S-curve jerk limit
        if (currentAcc < targetAcc) {
            currentAcc += jerk * dt;
            if (currentAcc > targetAcc) currentAcc = targetAcc;
        } else if (currentAcc > targetAcc) {
            currentAcc -= jerk * dt;
            if (currentAcc < targetAcc) currentAcc = targetAcc;
        }

        currentVel += currentAcc * dt;
        // Clamp to ±cruiseVel
        if (currentVel >  cruiseVel) { currentVel =  cruiseVel; currentAcc = 0; }
        if (currentVel < -cruiseVel) { currentVel = -cruiseVel; currentAcc = 0; }

        currentPos += currentVel * dt;
    }

    // Position crossing with a small tolerance for zero-exit-vel blocks.
    // Allows isComplete() to fire when the motor stops fractionally short of the
    // target (<=3 steps) due to S-curve undershoot; the hold PD then corrects it.
    // For chained blocks with non-zero exitVel the tolerance is 0 (exact crossing).
    bool isComplete() const {
        float tol = (exitVel < 5.0f) ? 3.0f : 0.0f;
        return moveForward ? (currentPos >= targetPos - tol)
                           : (currentPos <= targetPos + tol);
    }

private:
    // Dynamic braking lookahead that accounts for the S-curve jerk ramp.
    float _brakeLookahead(float spd) const {
        float brakeTarget = (currentVel >= 0.0f) ? -maxA : maxA;
        float T = std::fabs(currentAcc - brakeTarget) / jerk;
        return jerk * T * T * (spd / (2.0f * maxA) + T / 3.0f) + 2.0f;
    }
};

// ---------------------------------------------------------------------------
// Marlin-style chain planner
//
// Computes globally-optimal entry/exit velocities for a sequence of moves
// using a forward pass (kinematic achievability) followed by a reverse pass
// (safe-stop propagation).
//
//   Forward pass:  entry[i+1] = min(desired_junction, sqrt(entry[i]^2 + 2*a[i]*d[i]))
//   Reverse pass:  entry[i+1] = min(entry[i+1], sqrt(exit[i+1]^2 + 2*a[i+1]*d[i+1]))
//                  exit[i]    = entry[i+1]
//   Feasibility:   if exit[i] is below the minimum achievable (motor can't decelerate
//                  fast enough), raise it to the kinematic minimum.
//
// cmds:     command array
// n:        command count (1..MAX_CHAIN_LEN)
// startPos: absolute encoder position at chain start
// out:      output array (at least n elements)
// ---------------------------------------------------------------------------
inline void planChain(const MotionCommand* cmds, int n,
                      float startPos, PlannedBlock* out) {
    // ---- Populate blocks ----
    float pos = startPos;
    for (int i = 0; i < n; i++) {
        float raw = cmds[i].absolute
                    ? ((float)cmds[i].distance - pos)
                    : (float)cmds[i].distance;
        out[i].dist     = std::fabs(raw);
        out[i].forward  = (raw >= 0.0f);
        out[i].cruiseVel = std::fabs(cmds[i].maxSpeed);
        out[i].accel     = std::fabs(cmds[i].acceleration);
        if (out[i].accel < 1.0f) out[i].accel = 1.0f;
        out[i].entryVel  = 0.0f;
        out[i].exitVel   = 0.0f;
        pos += raw;
    }

    // ---- Forward pass ----
    // Propagate maximum achievable entry velocity at each junction.
    out[0].entryVel = 0.0f; // chain always starts from rest
    for (int i = 1; i < n; i++) {
        bool sameDir = (out[i-1].forward == out[i].forward);
        float desired = sameDir
            ? std::fmin(out[i-1].cruiseVel, out[i].cruiseVel)
            : 0.0f; // direction reversal: must stop at boundary
        float maxReach = std::sqrt(out[i-1].entryVel * out[i-1].entryVel
                                   + 2.0f * out[i-1].accel * out[i-1].dist);
        out[i].entryVel = std::fmin(desired, maxReach);
        if (out[i].entryVel > out[i].cruiseVel) out[i].entryVel = out[i].cruiseVel;
    }

    // Provisional exit speeds = next block's entry (chain always ends at rest)
    for (int i = 0; i < n - 1; i++) out[i].exitVel = out[i+1].entryVel;
    out[n-1].exitVel = 0.0f;

    // ---- Reverse pass ----
    // Constrain entry speeds so the motor can always stop by chain end.
    for (int i = n - 2; i >= 0; i--) {
        float maxEntry = std::sqrt(out[i+1].exitVel  * out[i+1].exitVel
                                   + 2.0f * out[i+1].accel * out[i+1].dist);
        if (out[i+1].entryVel > maxEntry) out[i+1].entryVel = maxEntry;
        out[i].exitVel = out[i+1].entryVel; // propagate back
    }

    // ---- Feasibility clamp ----
    for (int i = 0; i < n; i++) {
        if (out[i].entryVel > out[i].exitVel && out[i].dist > 0) {
            float sq = out[i].entryVel * out[i].entryVel
                       - 2.0f * out[i].accel * out[i].dist;
            float minExit = (sq > 0.0f) ? std::sqrt(sq) : 0.0f;
            if (out[i].exitVel < minExit) {
                out[i].exitVel = minExit;
                if (i + 1 < n) out[i+1].entryVel = minExit;
            }
        }
        // exitVel cannot exceed cruiseVel
        if (out[i].exitVel > out[i].cruiseVel) out[i].exitVel = out[i].cruiseVel;
    }
}

// ---------------------------------------------------------------------------
// Block Ring Buffer
//
// Fixed-size circular buffer of planned motion blocks for continuous
// look-ahead planning. Owned exclusively by PlannerTask (no mutex needed).
//
//   execHead (peekHead) = block currently being trajectory-generated
//   planTail             = next empty slot for incoming commands
//
// The executing block's entry velocity is committed (locked); its exit
// velocity and all subsequent blocks are plannable.
// ---------------------------------------------------------------------------

static constexpr int BLOCK_RING_SIZE = 24;

struct BlockEntry {
    PlannedBlock plan;    // velocities, accel, direction, distance
    float        startPos;
    float        endPos;
};

class BlockRingBuffer {
public:
    bool isEmpty() const { return count_ == 0; }
    bool isFull()  const { return count_ == BLOCK_RING_SIZE; }
    int  count()   const { return count_; }

    bool append(const BlockEntry& entry) {
        if (isFull()) return false;
        buf_[tail_] = entry;
        tail_ = (tail_ + 1) % BLOCK_RING_SIZE;
        ++count_;
        return true;
    }

    void advanceHead() {
        if (isEmpty()) return;
        head_ = (head_ + 1) % BLOCK_RING_SIZE;
        --count_;
    }

    const BlockEntry& peekHead() const { return buf_[head_]; }

    // Index 0 = head, index count()-1 = newest entry
    BlockEntry& at(int index) {
        return buf_[(head_ + index) % BLOCK_RING_SIZE];
    }
    const BlockEntry& at(int index) const {
        return buf_[(head_ + index) % BLOCK_RING_SIZE];
    }

    void clear() {
        head_  = 0;
        tail_  = 0;
        count_ = 0;
    }

private:
    BlockEntry buf_[BLOCK_RING_SIZE] = {};
    int head_  = 0;
    int tail_  = 0;
    int count_ = 0;
};

// ---------------------------------------------------------------------------
// Incremental Planner
//
// Operates on a BlockRingBuffer instead of a flat array. Produces identical
// results to planChain() for equivalent input, but can be called after
// appending new blocks without replanning already-committed blocks.
//
// The algorithm:
//   1. Forward pass: propagate max achievable junction velocities
//   2. Set last block exitVel = 0 (must stop at end of planned chain)
//   3. Reverse pass: constrain entry velocities for safe stopping
//   4. Feasibility clamp: ensure kinematic consistency
// ---------------------------------------------------------------------------
inline void incrementalPlan(BlockRingBuffer& ring, float headEntryVel = 0.0f,
                            float tailExitVel = 0.0f) {
    int n = ring.count();
    if (n == 0) return;

    // --- Forward pass ---
    ring.at(0).plan.entryVel = headEntryVel;  // 0 for fresh chain, committed value during execution
    for (int i = 1; i < n; i++) {
        PlannedBlock& prev = ring.at(i - 1).plan;
        PlannedBlock& cur  = ring.at(i).plan;

        bool sameDir = (prev.forward == cur.forward);
        float desired = sameDir
            ? std::fmin(prev.cruiseVel, cur.cruiseVel)
            : 0.0f;
        float maxReach = std::sqrt(prev.entryVel * prev.entryVel
                                   + 2.0f * prev.accel * prev.dist);
        cur.entryVel = std::fmin(desired, maxReach);
        if (cur.entryVel > cur.cruiseVel) cur.entryVel = cur.cruiseVel;
    }

    // Provisional exit speeds
    for (int i = 0; i < n - 1; i++) {
        ring.at(i).plan.exitVel = ring.at(i + 1).plan.entryVel;
    }
    ring.at(n - 1).plan.exitVel = tailExitVel;  // 0 = stop at end; >0 = loop wrap

    // --- Reverse pass ---
    for (int i = n - 2; i >= 0; i--) {
        PlannedBlock& next = ring.at(i + 1).plan;
        float maxEntry = std::sqrt(next.exitVel * next.exitVel
                                   + 2.0f * next.accel * next.dist);
        if (next.entryVel > maxEntry) next.entryVel = maxEntry;
        ring.at(i).plan.exitVel = next.entryVel;
    }

    // --- Feasibility clamp ---
    for (int i = 0; i < n; i++) {
        PlannedBlock& blk = ring.at(i).plan;
        if (blk.entryVel > blk.exitVel && blk.dist > 0) {
            float sq = blk.entryVel * blk.entryVel
                       - 2.0f * blk.accel * blk.dist;
            float minExit = (sq > 0.0f) ? std::sqrt(sq) : 0.0f;
            if (blk.exitVel < minExit) {
                blk.exitVel = minExit;
                if (i + 1 < n) ring.at(i + 1).plan.entryVel = minExit;
            }
        }
        if (blk.exitVel > blk.cruiseVel) blk.exitVel = blk.cruiseVel;
    }
}

// ---------------------------------------------------------------------------
// Loop Planner
//
// Like incrementalPlan() but treats the block sequence as a circular loop.
// The last block's exit velocity connects to the first block's entry velocity
// (wrap-around junction). Same-direction wrap = min(cruise_last, cruise_first);
// direction reversal at wrap = 0.
//
// Uses iterative convergence: forward+reverse passes are repeated until the
// wrap junction stabilises (typically 2-3 iterations).
// ---------------------------------------------------------------------------
inline void planLoop(BlockRingBuffer& ring) {
    int n = ring.count();
    if (n == 0) return;

    // Determine wrap junction: direction continuity at last→first boundary
    bool wrapSameDir = (ring.at(n - 1).plan.forward == ring.at(0).plan.forward);
    float wrapDesired = wrapSameDir
        ? std::fmin(ring.at(n - 1).plan.cruiseVel, ring.at(0).plan.cruiseVel)
        : 0.0f;

    // Seed: start with desired wrap velocity as entry to block 0
    float wrapVel = wrapDesired;

    // Iterative convergence (typically converges in 2-3 passes)
    for (int iter = 0; iter < 5; iter++) {
        float prevWrap = wrapVel;

        // --- Forward pass ---
        ring.at(0).plan.entryVel = wrapVel;
        for (int i = 1; i < n; i++) {
            PlannedBlock& prev = ring.at(i - 1).plan;
            PlannedBlock& cur  = ring.at(i).plan;

            bool sameDir = (prev.forward == cur.forward);
            float desired = sameDir
                ? std::fmin(prev.cruiseVel, cur.cruiseVel)
                : 0.0f;
            float maxReach = std::sqrt(prev.entryVel * prev.entryVel
                                       + 2.0f * prev.accel * prev.dist);
            cur.entryVel = std::fmin(desired, maxReach);
            if (cur.entryVel > cur.cruiseVel) cur.entryVel = cur.cruiseVel;
        }

        // Provisional exit speeds (circular: last→first)
        for (int i = 0; i < n - 1; i++) {
            ring.at(i).plan.exitVel = ring.at(i + 1).plan.entryVel;
        }
        // Wrap junction: last block exit = achievable wrap velocity
        float lastMaxReach = std::sqrt(ring.at(n - 1).plan.entryVel * ring.at(n - 1).plan.entryVel
                                       + 2.0f * ring.at(n - 1).plan.accel * ring.at(n - 1).plan.dist);
        ring.at(n - 1).plan.exitVel = std::fmin(wrapDesired, lastMaxReach);
        if (ring.at(n - 1).plan.exitVel > ring.at(n - 1).plan.cruiseVel)
            ring.at(n - 1).plan.exitVel = ring.at(n - 1).plan.cruiseVel;

        // --- Reverse pass (circular) ---
        // Start from block n-1 backward, wrapping around to first block
        // The "next" of block 0 is block n-1 in the reverse direction,
        // but we process: n-1 constrains n-2, ..., 1 constrains 0, then 0 constrains n-1
        for (int i = n - 2; i >= 0; i--) {
            PlannedBlock& next = ring.at(i + 1).plan;
            float maxEntry = std::sqrt(next.exitVel * next.exitVel
                                       + 2.0f * next.accel * next.dist);
            if (next.entryVel > maxEntry) next.entryVel = maxEntry;
            ring.at(i).plan.exitVel = next.entryVel;
        }
        // Wrap: block 0 constrains block n-1's exit
        {
            PlannedBlock& first = ring.at(0).plan;
            float maxWrap = std::sqrt(first.exitVel * first.exitVel
                                      + 2.0f * first.accel * first.dist);
            // Also constrain by what block 0 can decelerate FROM
            float maxFromFirst = std::sqrt(first.entryVel * first.entryVel
                                           + 2.0f * first.accel * first.dist);
            // Wrap velocity must be achievable by both last block (reaching it)
            // and first block (starting from it)
            wrapVel = std::fmin(ring.at(n - 1).plan.exitVel, wrapDesired);
            // Constrain by what last block can actually reach
            float lastEntry = ring.at(n - 1).plan.entryVel;
            float lastReach = std::sqrt(lastEntry * lastEntry
                                        + 2.0f * ring.at(n - 1).plan.accel * ring.at(n - 1).plan.dist);
            wrapVel = std::fmin(wrapVel, lastReach);
            if (wrapVel > ring.at(n - 1).plan.cruiseVel)
                wrapVel = ring.at(n - 1).plan.cruiseVel;
        }

        ring.at(n - 1).plan.exitVel = wrapVel;
        ring.at(0).plan.entryVel = wrapVel;

        // --- Feasibility clamp ---
        for (int i = 0; i < n; i++) {
            PlannedBlock& blk = ring.at(i).plan;
            if (blk.entryVel > blk.exitVel && blk.dist > 0) {
                float sq = blk.entryVel * blk.entryVel
                           - 2.0f * blk.accel * blk.dist;
                float minExit = (sq > 0.0f) ? std::sqrt(sq) : 0.0f;
                if (blk.exitVel < minExit) {
                    blk.exitVel = minExit;
                    if (i + 1 < n) ring.at(i + 1).plan.entryVel = minExit;
                    else ring.at(0).plan.entryVel = minExit; // wrap
                }
            }
            if (blk.exitVel > blk.cruiseVel) blk.exitVel = blk.cruiseVel;
        }

        // Convergence check
        if (std::fabs(wrapVel - prevWrap) < 0.5f) break;
    }
}

// ---------------------------------------------------------------------------
// Controlled Stop Injection
//
// Clears the ring buffer and injects a single deceleration block that brings
// the motor from its current velocity to zero.
//
// Parameters:
//   ring     — block ring buffer (will be cleared and receive the decel block)
//   stopPos  — current motor position
//   velocity — current speed (unsigned magnitude)
//   accel    — deceleration rate
//   forward  — current direction of travel
//
// Returns: number of blocks in ring after injection (0 or 1)
// ---------------------------------------------------------------------------
inline int injectStopBlock(BlockRingBuffer& ring, float stopPos, float velocity,
                           float accel, bool forward) {
    ring.clear();

    if (velocity < 0.5f) {
        // Already stopped — nothing to inject
        return 0;
    }

    float dist = velocity * velocity / (2.0f * accel);

    BlockEntry decel = {};
    decel.plan.dist      = dist;
    decel.plan.forward   = forward;
    decel.plan.cruiseVel = velocity;
    decel.plan.accel     = accel;
    decel.plan.entryVel  = velocity;
    decel.plan.exitVel   = 0.0f;
    decel.startPos = stopPos;
    decel.endPos   = forward ? (stopPos + dist) : (stopPos - dist);

    ring.append(decel);
    return 1;
}

} // namespace planner
