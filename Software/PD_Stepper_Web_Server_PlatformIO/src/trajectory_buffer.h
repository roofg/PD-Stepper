#pragma once
// trajectory_buffer.h — SPSC (single-producer, single-consumer) ring buffer
// for trajectory reference points produced by the Planner Task and consumed
// by the Control Task.
//
// IMPORTANT: Include this header from motion_control.cpp only. The static
// storage relies on a single translation unit.
//
// Design: lock-free SPSC with power-of-2 size.
//
//   Synchronization invariant (MUST be maintained):
//     • s_head is written ONLY by the producer (Planner Task, Core 0).
//     • s_tail is written ONLY by the consumer (Control Task, Core 1).
//     Crossing this rule creates an unfixable data race on dual-core hardware.
//
//   Memory ordering (ESP32-S3 Xtensa LX7):
//     • portMEMORY_BARRIER() is a no-op in Arduino-ESP32 FreeRTOS — do NOT use it.
//     • std::atomic<uint8_t> with acquire/release semantics provides real
//       inter-core ordering without locking.
//     • Producer: write payload, then store head with memory_order_release.
//     • Consumer: load head with memory_order_acquire before reading payload,
//       then store tail with memory_order_release.
//
// Buffer capacity: BUF_SIZE - 1 slots (one slot sacrificed for full/empty
// detection without an extra counter).
//
// If push() is called on a full buffer the incoming entry is silently dropped
// (producer never touches s_tail). Since the Control Task (1 kHz) consumes
// faster than the Planner Task (500 Hz) produces, a full buffer indicates a
// transient scheduling delay — dropping the excess is correct behaviour.

#include <Arduino.h>
#include <atomic>
#include <stdint.h>

struct TrajectoryPoint {
    float pos;  // planner reference position  (microsteps, absolute)
    float vel;  // planner reference velocity  (microsteps/sec)
    float acc;  // planner reference acceleration (microsteps/sec²)
};

namespace trajbuf {

static constexpr uint8_t BUF_SIZE = 16;  // must be a power of 2

static TrajectoryPoint s_buf[BUF_SIZE];
// Indices are std::atomic for real acquire/release cross-core ordering.
// Producer (Planner Task) owns s_head. Consumer (Control Task) owns s_tail.
static std::atomic<uint8_t> s_head{0};
static std::atomic<uint8_t> s_tail{0};

inline uint8_t count() {
    // Relaxed: called only from producer (single-reader of its own head).
    uint8_t h = s_head.load(std::memory_order_relaxed);
    uint8_t t = s_tail.load(std::memory_order_acquire);
    return (uint8_t)((h - t) & (BUF_SIZE - 1));
}

inline bool isEmpty() {
    // Consumer calls this — acquire tail (own), acquire head (cross-core).
    uint8_t h = s_head.load(std::memory_order_acquire);
    uint8_t t = s_tail.load(std::memory_order_relaxed);
    return h == t;
}

inline bool isFull() {
    return count() >= (BUF_SIZE - 1);
}

// Push a point. Drops the incoming entry if full — NEVER touches s_tail.
// Release store on s_head ensures payload bytes are visible to the consumer
// before the consumer sees the updated index.
inline void push(const TrajectoryPoint &pt) {
    uint8_t h = s_head.load(std::memory_order_relaxed);
    uint8_t next_h = (h + 1) & (BUF_SIZE - 1);
    if (next_h == s_tail.load(std::memory_order_acquire)) {
        return; // Full — drop incoming, never write s_tail from producer
    }
    s_buf[h] = pt;
    s_head.store(next_h, std::memory_order_release);
}

// Pop the oldest point. Returns false if empty.
// Acquire load on s_head ensures payload is visible after consumer sees
// the updated head index. Release store on s_tail signals producer.
inline bool pop(TrajectoryPoint &pt) {
    uint8_t t = s_tail.load(std::memory_order_relaxed);
    if (t == s_head.load(std::memory_order_acquire)) {
        return false; // Empty
    }
    pt = s_buf[t];
    s_tail.store((t + 1) & (BUF_SIZE - 1), std::memory_order_release);
    return true;
}

// Clear without touching buffer contents (just resets indices).
// Only safe to call when no motion is active (before s_running is set true).
inline void clear() {
    s_tail.store(s_head.load(std::memory_order_relaxed), std::memory_order_relaxed);
}

} // namespace trajbuf
