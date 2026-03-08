#pragma once
// trajectory_buffer.h — SPSC (single-producer, single-consumer) ring buffer
// for trajectory reference points produced by the Planner Task and consumed
// by the Control Task.
//
// IMPORTANT: Include this header from motion_control.cpp only. The static
// storage relies on a single translation unit.
//
// Design: lock-free SPSC with power-of-2 size. head is written by the
// producer (Planner Task, Core 0), tail is written by the consumer (Control
// Task, Core 1). No mutex is required because:
//   • Only one task writes head, only one task writes tail.
//   • uint8_t reads/writes are atomic on ESP32.
//
// Buffer capacity: BUF_SIZE - 1 slots (one slot is sacrificed for full/empty
// detection without an extra counter).
//
// If push() is called on a full buffer the oldest entry is silently dropped
// so the consumer always sees the freshest trajectory data.

#include <Arduino.h>
#include <stdint.h>

struct TrajectoryPoint {
    float pos;  // planner reference position  (microsteps, absolute)
    float vel;  // planner reference velocity  (microsteps/sec)
    float acc;  // planner reference acceleration (microsteps/sec²)
};

namespace trajbuf {

static constexpr uint8_t BUF_SIZE = 16;  // must be a power of 2

static TrajectoryPoint s_buf[BUF_SIZE];
static volatile uint8_t s_head = 0;  // producer advances
static volatile uint8_t s_tail = 0;  // consumer advances

inline uint8_t count() {
    return (uint8_t)((s_head - s_tail) & (BUF_SIZE - 1));
}

inline bool isEmpty() {
    return s_head == s_tail;
}

inline bool isFull() {
    return count() >= (BUF_SIZE - 1);
}

// Push a point. Drops the oldest entry if full (keeps latest data fresh).
inline void push(const TrajectoryPoint &pt) {
    if (isFull()) {
        // Advance tail to drop oldest
        s_tail = (s_tail + 1) & (BUF_SIZE - 1);
    }
    s_buf[s_head] = pt;
    s_head = (s_head + 1) & (BUF_SIZE - 1);
}

// Pop the oldest point. Returns false if empty.
inline bool pop(TrajectoryPoint &pt) {
    if (isEmpty()) return false;
    pt = s_buf[s_tail];
    s_tail = (s_tail + 1) & (BUF_SIZE - 1);
    return true;
}

// Clear without touching buffer contents (just resets indices).
inline void clear() {
    s_tail = s_head;
}

} // namespace trajbuf
