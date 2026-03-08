// trajectory_buffer.cpp — SPSC ring buffer implementation.
// All static storage lives here (single translation unit) to avoid ODR violations
// that would occur if the previous header-only approach were included from
// multiple .cpp files.

#include "trajectory_buffer.h"
#include <atomic>

namespace trajbuf {

static constexpr uint8_t BUF_SIZE = 16; // must be power of 2

static TrajectoryPoint s_buf[BUF_SIZE];

// Producer (Planner Task, Core 0) owns s_head — only push() writes it.
// Consumer (Control Task, Core 1) owns s_tail — only pop() writes it.
static std::atomic<uint8_t> s_head{0};
static std::atomic<uint8_t> s_tail{0};

void push(const TrajectoryPoint &pt) {
    uint8_t h      = s_head.load(std::memory_order_relaxed);
    uint8_t next_h = (h + 1) & (BUF_SIZE - 1);
    // Full check: acquire tail so we see the consumer's latest release store.
    if (next_h == s_tail.load(std::memory_order_acquire)) {
        return; // Drop — never write s_tail from producer
    }
    s_buf[h] = pt;
    // Release: ensures s_buf[h] write is visible before consumer sees new head.
    s_head.store(next_h, std::memory_order_release);
}

bool pop(TrajectoryPoint &pt) {
    uint8_t t = s_tail.load(std::memory_order_relaxed);
    // Acquire head so payload bytes written by producer are visible.
    if (t == s_head.load(std::memory_order_acquire)) {
        return false; // Empty
    }
    pt = s_buf[t];
    // Release: signals producer that this slot is now free.
    s_tail.store((t + 1) & (BUF_SIZE - 1), std::memory_order_release);
    return true;
}

uint8_t count() {
    uint8_t h = s_head.load(std::memory_order_acquire);
    uint8_t t = s_tail.load(std::memory_order_acquire);
    return (uint8_t)((h - t) & (BUF_SIZE - 1));
}

void clear() {
    // Safe only before s_running is set true — no contention.
    s_tail.store(s_head.load(std::memory_order_relaxed), std::memory_order_relaxed);
}

} // namespace trajbuf
