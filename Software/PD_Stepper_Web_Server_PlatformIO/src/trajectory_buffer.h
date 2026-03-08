#pragma once
// trajectory_buffer.h — SPSC (single-producer, single-consumer) ring buffer
// for trajectory reference points produced by the Planner Task and consumed
// by the Control Task.
//
// Design: lock-free SPSC with power-of-2 size (16 slots).
//
//   Synchronization invariant (MUST be maintained):
//     • s_head is written ONLY by the producer (Planner Task, Core 0).
//     • s_tail is written ONLY by the consumer (Control Task, Core 1).
//
//   Memory ordering (ESP32-S3 Xtensa LX7):
//     • std::atomic<uint8_t> with acquire/release semantics provides real
//       inter-core ordering without locking.
//     • Producer: write payload, then store head with memory_order_release.
//     • Consumer: load head with memory_order_acquire before reading payload,
//       then store tail with memory_order_release.
//
// If push() is called on a full buffer the incoming entry is silently dropped
// (producer never touches s_tail). Control Task (1 kHz) consumes faster than
// Planner Task (500 Hz) produces so a full buffer is transient scheduling jitter.

#include <stdint.h>

struct TrajectoryPoint {
    float pos;  // planner reference position  (microsteps, absolute)
    float vel;  // planner reference velocity  (microsteps/sec)
    float acc;  // planner reference acceleration (microsteps/sec²)
};

namespace trajbuf {

// Push a point from the producer (Planner Task). Drops if full.
void push(const TrajectoryPoint &pt);

// Pop the oldest point into pt. Returns false if empty.
bool pop(TrajectoryPoint &pt);

// Returns approximate number of queued entries (approximate — for diagnostics only).
uint8_t count();

// Reset indices. Only safe when no motion is active.
void clear();

} // namespace trajbuf
