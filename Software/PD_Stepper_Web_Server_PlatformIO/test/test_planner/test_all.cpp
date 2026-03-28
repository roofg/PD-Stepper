// test_all.cpp — Unified test runner for all planner and PD controller tests
//
// PlatformIO Unity links all .cpp in a test directory into one binary,
// so there must be exactly one main()/setUp()/tearDown(). This file
// includes all test functions and runs them from a single entry point.

#include <unity.h>
#include "../../src/planner_core.h"
#include "../../src/pd_controller.h"
#include <cmath>

using namespace planner;

// ===================================================================
// Shared helpers
// ===================================================================

static MotionCommand relMove(long dist, float accel, float speed, bool chain = true) {
    return { dist, accel, speed, false, chain };
}

static MotionCommand absMove(long target, float accel, float speed, bool chain = false) {
    return { target, accel, speed, true, chain };
}

static void fillRingFromCommands(BlockRingBuffer& ring, const MotionCommand* cmds,
                                 int n, float startPos) {
    float pos = startPos;
    for (int i = 0; i < n; i++) {
        BlockEntry entry = {};
        float raw = cmds[i].absolute
                    ? ((float)cmds[i].distance - pos)
                    : (float)cmds[i].distance;
        entry.plan.dist     = std::fabs(raw);
        entry.plan.forward  = (raw >= 0.0f);
        entry.plan.cruiseVel = std::fabs(cmds[i].maxSpeed);
        entry.plan.accel     = std::fabs(cmds[i].acceleration);
        if (entry.plan.accel < 1.0f) entry.plan.accel = 1.0f;
        entry.plan.entryVel  = 0.0f;
        entry.plan.exitVel   = 0.0f;
        entry.startPos = pos;
        pos += raw;
        entry.endPos = pos;
        ring.append(entry);
    }
}

// ===================================================================
// planChain() tests
// ===================================================================

void test_single_move_entry_exit_zero() {
    MotionCommand cmds[] = { relMove(10000, 5000, 20000, false) };
    PlannedBlock blocks[1];
    planChain(cmds, 1, 0.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[0].entryVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[0].exitVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10000.0f, blocks[0].dist);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20000.0f, blocks[0].cruiseVel);
    TEST_ASSERT_TRUE(blocks[0].forward);
}

void test_two_same_dir_junction_velocity() {
    MotionCommand cmds[] = {
        relMove(10000, 5000, 10000, true),
        relMove(10000, 5000, 10000, false),
    };
    PlannedBlock blocks[2];
    planChain(cmds, 2, 0.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[0].entryVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[1].exitVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10000.0f, blocks[0].exitVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 10000.0f, blocks[1].entryVel);
}

void test_direction_reversal_junction_zero() {
    MotionCommand cmds[] = {
        relMove(5000, 5000, 10000, true),
        relMove(-5000, 5000, 10000, false),
    };
    PlannedBlock blocks[2];
    planChain(cmds, 2, 0.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[0].exitVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[1].entryVel);
    TEST_ASSERT_TRUE(blocks[0].forward);
    TEST_ASSERT_FALSE(blocks[1].forward);
}

void test_short_segment_feasibility_clamp() {
    MotionCommand cmds[] = {
        relMove(20000, 5000, 20000, true),
        relMove(100, 5000, 20000, true),
        relMove(20000, 5000, 20000, false),
    };
    PlannedBlock blocks[3];
    planChain(cmds, 3, 0.0f, blocks);

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_TRUE(blocks[i].exitVel >= 0.0f);
        TEST_ASSERT_TRUE(blocks[i].entryVel >= 0.0f);
        TEST_ASSERT_TRUE(blocks[i].exitVel <= blocks[i].cruiseVel);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[2].exitVel);
}

void test_reverse_pass_safe_stop() {
    MotionCommand cmds[] = {
        relMove(10000, 10000, 20000, true),
        relMove(10000, 10000, 20000, true),
        relMove(500, 10000, 20000, false),
    };
    PlannedBlock blocks[3];
    planChain(cmds, 3, 0.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[2].exitVel);
    float maxEntryLast = std::sqrt(2.0f * 10000.0f * 500.0f);
    TEST_ASSERT_TRUE(blocks[2].entryVel <= maxEntryLast + 1.0f);
}

void test_absolute_positioning() {
    MotionCommand cmds[] = { absMove(5000, 5000, 10000) };
    PlannedBlock blocks[1];
    planChain(cmds, 1, 1000.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 4000.0f, blocks[0].dist);
    TEST_ASSERT_TRUE(blocks[0].forward);
}

void test_absolute_positioning_backward() {
    MotionCommand cmds[] = { absMove(1000, 5000, 10000) };
    PlannedBlock blocks[1];
    planChain(cmds, 1, 5000.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 4000.0f, blocks[0].dist);
    TEST_ASSERT_FALSE(blocks[0].forward);
}

void test_accel_clamped_to_minimum() {
    MotionCommand cmds[] = { relMove(1000, 0.5f, 100, false) };
    PlannedBlock blocks[1];
    planChain(cmds, 1, 0.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, blocks[0].accel);
}

void test_symmetric_chain() {
    MotionCommand cmds[] = {
        relMove(5000, 10000, 15000, true),
        relMove(10000, 10000, 15000, true),
        relMove(5000, 10000, 15000, false),
    };
    PlannedBlock blocks[3];
    planChain(cmds, 3, 0.0f, blocks);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[0].entryVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, blocks[2].exitVel);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, blocks[0].exitVel, blocks[2].entryVel);
}

// ===================================================================
// TrajectoryPlanner tests
// ===================================================================

void test_trajectory_forward_completes() {
    PlannedBlock blk = { 1000.0f, 0.0f, 500.0f, 0.0f, 5000.0f, true };
    JerkConfig jcfg = { 0.0f, 0.0f };

    TrajectoryPlanner tp;
    tp.currentVel = 0.0f;
    tp.currentAcc = 0.0f;
    tp.resetForBlock(0.0f, 1000.0f, blk, jcfg);

    bool completed = false;
    for (int i = 0; i < 2500; i++) {
        tp.update(0.002f);
        if (tp.isComplete()) { completed = true; break; }
    }
    TEST_ASSERT_TRUE(completed);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1000.0f, tp.currentPos);
}

void test_trajectory_backward_completes() {
    PlannedBlock blk = { 1000.0f, 0.0f, 500.0f, 0.0f, 5000.0f, false };
    JerkConfig jcfg = { 0.0f, 0.0f };

    TrajectoryPlanner tp;
    tp.currentVel = 0.0f;
    tp.currentAcc = 0.0f;
    tp.resetForBlock(0.0f, -1000.0f, blk, jcfg);

    bool completed = false;
    for (int i = 0; i < 2500; i++) {
        tp.update(0.002f);
        if (tp.isComplete()) { completed = true; break; }
    }
    TEST_ASSERT_TRUE(completed);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, -1000.0f, tp.currentPos);
}

void test_trajectory_respects_cruise() {
    PlannedBlock blk = { 10000.0f, 0.0f, 1000.0f, 0.0f, 50000.0f, true };
    JerkConfig jcfg = { 0.0f, 0.0f };

    TrajectoryPlanner tp;
    tp.currentVel = 0.0f;
    tp.currentAcc = 0.0f;
    tp.resetForBlock(0.0f, 10000.0f, blk, jcfg);

    float maxVel = 0.0f;
    for (int i = 0; i < 5000; i++) {
        tp.update(0.002f);
        float absVel = std::fabs(tp.currentVel);
        if (absVel > maxVel) maxVel = absVel;
        if (tp.isComplete()) break;
    }
    TEST_ASSERT_FLOAT_WITHIN(50.0f, 1000.0f, maxVel);
}

void test_trajectory_velocity_continuity() {
    MotionCommand cmds[] = {
        relMove(5000, 10000, 8000, true),
        relMove(5000, 10000, 8000, false),
    };
    PlannedBlock blocks[2];
    planChain(cmds, 2, 0.0f, blocks);

    JerkConfig jcfg = { 0.0f, 0.0f };
    TrajectoryPlanner tp;
    tp.currentVel = 0.0f;
    tp.currentAcc = 0.0f;

    tp.resetForBlock(0.0f, 5000.0f, blocks[0], jcfg);
    for (int i = 0; i < 2500; i++) {
        tp.update(0.002f);
        if (tp.isComplete()) break;
    }
    float velAtJunction = tp.currentVel;

    tp.resetForBlock(tp.currentPos, tp.currentPos + 5000.0f, blocks[1], jcfg);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, velAtJunction, tp.currentVel);

    for (int i = 0; i < 2500; i++) {
        tp.update(0.002f);
        if (tp.isComplete()) break;
    }
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 10000.0f, tp.currentPos);
}

void test_trajectory_scurve_jerk() {
    PlannedBlock blk = { 10000.0f, 0.0f, 5000.0f, 0.0f, 20000.0f, true };
    JerkConfig jcfg = { 0.15f, 0.0f };

    TrajectoryPlanner tp;
    tp.currentVel = 0.0f;
    tp.currentAcc = 0.0f;
    tp.resetForBlock(0.0f, 10000.0f, blk, jcfg);

    tp.update(0.002f);
    float accAfterFirstTick = std::fabs(tp.currentAcc);
    TEST_ASSERT_TRUE(accAfterFirstTick < 20000.0f * 0.1f);
}

// ===================================================================
// BlockRingBuffer tests
// ===================================================================

void test_ring_starts_empty() {
    BlockRingBuffer ring;
    TEST_ASSERT_TRUE(ring.isEmpty());
    TEST_ASSERT_FALSE(ring.isFull());
    TEST_ASSERT_EQUAL_INT(0, ring.count());
}

void test_ring_append_one() {
    BlockRingBuffer ring;
    BlockEntry entry = {};
    entry.plan.dist = 1000.0f;
    entry.plan.forward = true;

    bool ok = ring.append(entry);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_INT(1, ring.count());
    TEST_ASSERT_FALSE(ring.isEmpty());
}

void test_ring_append_and_read() {
    BlockRingBuffer ring;
    BlockEntry entry = {};
    entry.plan.dist = 42.0f;
    entry.startPos = 100.0f;
    entry.endPos = 142.0f;

    ring.append(entry);
    const BlockEntry& head = ring.peekHead();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 42.0f, head.plan.dist);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, head.startPos);
}

void test_ring_advance_head() {
    BlockRingBuffer ring;
    BlockEntry e1 = {}, e2 = {};
    e1.plan.dist = 100.0f;
    e2.plan.dist = 200.0f;

    ring.append(e1);
    ring.append(e2);
    TEST_ASSERT_EQUAL_INT(2, ring.count());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, ring.peekHead().plan.dist);

    ring.advanceHead();
    TEST_ASSERT_EQUAL_INT(1, ring.count());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, ring.peekHead().plan.dist);
}

void test_ring_full() {
    BlockRingBuffer ring;
    for (int i = 0; i < BLOCK_RING_SIZE; i++) {
        BlockEntry e = {};
        e.plan.dist = (float)i;
        bool ok = ring.append(e);
        TEST_ASSERT_TRUE(ok);
    }
    TEST_ASSERT_TRUE(ring.isFull());
    TEST_ASSERT_EQUAL_INT(BLOCK_RING_SIZE, ring.count());

    BlockEntry overflow = {};
    TEST_ASSERT_FALSE(ring.append(overflow));
}

void test_ring_wrap_around() {
    BlockRingBuffer ring;
    for (int i = 0; i < BLOCK_RING_SIZE / 2; i++) {
        BlockEntry e = {};
        e.plan.dist = (float)i;
        ring.append(e);
    }
    for (int i = 0; i < BLOCK_RING_SIZE / 2; i++) {
        ring.advanceHead();
    }
    TEST_ASSERT_TRUE(ring.isEmpty());

    for (int i = 0; i < BLOCK_RING_SIZE; i++) {
        BlockEntry e = {};
        e.plan.dist = (float)(100 + i);
        bool ok = ring.append(e);
        TEST_ASSERT_TRUE(ok);
    }
    TEST_ASSERT_TRUE(ring.isFull());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, ring.peekHead().plan.dist);
}

void test_ring_clear() {
    BlockRingBuffer ring;
    BlockEntry e = {};
    ring.append(e);
    ring.append(e);
    ring.clear();
    TEST_ASSERT_TRUE(ring.isEmpty());
    TEST_ASSERT_EQUAL_INT(0, ring.count());
}

void test_ring_at_index() {
    BlockRingBuffer ring;
    for (int i = 0; i < 5; i++) {
        BlockEntry e = {};
        e.plan.dist = (float)(i * 100);
        ring.append(e);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ring.at(0).plan.dist);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 200.0f, ring.at(2).plan.dist);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 400.0f, ring.at(4).plan.dist);
}

void test_ring_mutable_at() {
    BlockRingBuffer ring;
    BlockEntry e = {};
    e.plan.dist = 100.0f;
    ring.append(e);

    ring.at(0).plan.exitVel = 999.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 999.0f, ring.at(0).plan.exitVel);
}

// ===================================================================
// incrementalPlan() tests
// ===================================================================

void test_incremental_matches_planchain_single() {
    MotionCommand cmds[] = { relMove(10000, 5000, 20000, false) };

    PlannedBlock ref[1];
    planChain(cmds, 1, 0.0f, ref);

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 1, 0.0f);
    incrementalPlan(ring);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, ref[0].entryVel, ring.at(0).plan.entryVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, ref[0].exitVel, ring.at(0).plan.exitVel);
}

void test_incremental_matches_planchain_two_same_dir() {
    MotionCommand cmds[] = {
        relMove(10000, 5000, 10000, true),
        relMove(10000, 5000, 10000, false),
    };

    PlannedBlock ref[2];
    planChain(cmds, 2, 0.0f, ref);

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    incrementalPlan(ring);

    for (int i = 0; i < 2; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1.0f, ref[i].entryVel, ring.at(i).plan.entryVel);
        TEST_ASSERT_FLOAT_WITHIN(1.0f, ref[i].exitVel, ring.at(i).plan.exitVel);
    }
}

void test_incremental_matches_planchain_reversal() {
    MotionCommand cmds[] = {
        relMove(5000, 5000, 10000, true),
        relMove(-5000, 5000, 10000, false),
    };

    PlannedBlock ref[2];
    planChain(cmds, 2, 0.0f, ref);

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    incrementalPlan(ring);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(0).plan.exitVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(1).plan.entryVel);
}

void test_incremental_matches_planchain_three_moves() {
    MotionCommand cmds[] = {
        relMove(10000, 10000, 20000, true),
        relMove(10000, 10000, 20000, true),
        relMove(500, 10000, 20000, false),
    };

    PlannedBlock ref[3];
    planChain(cmds, 3, 0.0f, ref);

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 3, 0.0f);
    incrementalPlan(ring);

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1.0f, ref[i].entryVel, ring.at(i).plan.entryVel);
        TEST_ASSERT_FLOAT_WITHIN(1.0f, ref[i].exitVel, ring.at(i).plan.exitVel);
    }
}

void test_incremental_append_after_commit() {
    BlockRingBuffer ring;
    MotionCommand cmds[] = {
        relMove(10000, 5000, 10000, true),
        relMove(10000, 5000, 10000, true),
    };
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    incrementalPlan(ring);

    BlockEntry e3 = {};
    e3.plan.dist = 10000.0f;
    e3.plan.forward = true;
    e3.plan.cruiseVel = 10000.0f;
    e3.plan.accel = 5000.0f;
    e3.startPos = 20000.0f;
    e3.endPos = 30000.0f;
    ring.append(e3);

    incrementalPlan(ring);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(0).plan.entryVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(2).plan.exitVel);
    TEST_ASSERT_TRUE(ring.at(1).plan.exitVel > 0.0f);
}

void test_incremental_replan_lowers_exit_velocity() {
    BlockRingBuffer ring;
    MotionCommand cmds[] = {
        relMove(10000, 5000, 10000, true),
        relMove(500, 5000, 10000, false),
    };
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    incrementalPlan(ring);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(1).plan.exitVel);

    float maxJunction = std::sqrt(2.0f * 5000.0f * 500.0f);
    TEST_ASSERT_TRUE(ring.at(0).plan.exitVel <= maxJunction + 1.0f);
}

void test_split_chain_replan_raises_junction() {
    BlockRingBuffer ring;
    MotionCommand cmd1 = relMove(-30000, 5000, 5000, true);
    fillRingFromCommands(ring, &cmd1, 1, 0.0f);
    incrementalPlan(ring);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(0).plan.exitVel);

    MotionCommand cmd2 = relMove(-30000, 1000, 1000, false);
    float tailPos = ring.at(0).endPos;
    BlockEntry entry = {};
    float raw = (float)cmd2.distance;
    entry.plan.dist      = std::fabs(raw);
    entry.plan.forward   = (raw >= 0.0f);
    entry.plan.cruiseVel = std::fabs(cmd2.maxSpeed);
    entry.plan.accel     = std::fabs(cmd2.acceleration);
    entry.plan.entryVel  = 0.0f;
    entry.plan.exitVel   = 0.0f;
    entry.startPos = tailPos;
    entry.endPos   = tailPos + raw;
    ring.append(entry);

    incrementalPlan(ring, ring.at(0).plan.entryVel);

    TEST_ASSERT_TRUE(ring.at(0).plan.exitVel > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, ring.at(0).plan.exitVel, ring.at(1).plan.entryVel);
}

void test_split_chain_trajectory_sees_updated_exit_vel() {
    BlockRingBuffer ring;
    MotionCommand cmd1 = relMove(-30000, 5000, 5000, true);
    fillRingFromCommands(ring, &cmd1, 1, 0.0f);
    incrementalPlan(ring);

    TrajectoryPlanner traj;
    traj.currentVel = 0.0f;
    traj.currentAcc = 0.0f;
    JerkConfig jcfg = {0.0f, 0.0f};
    traj.resetForBlock(ring.at(0).startPos, ring.at(0).endPos,
                       ring.at(0).plan, jcfg);

    for (int i = 0; i < 50; i++) traj.update(0.002f);
    TEST_ASSERT_TRUE(std::fabs(traj.currentVel) > 0.0f);

    MotionCommand cmd2 = relMove(-30000, 1000, 1000, false);
    float tailPos = ring.at(0).endPos;
    BlockEntry entry = {};
    float raw = (float)cmd2.distance;
    entry.plan.dist      = std::fabs(raw);
    entry.plan.forward   = (raw >= 0.0f);
    entry.plan.cruiseVel = std::fabs(cmd2.maxSpeed);
    entry.plan.accel     = std::fabs(cmd2.acceleration);
    entry.plan.entryVel  = 0.0f;
    entry.plan.exitVel   = 0.0f;
    entry.startPos = tailPos;
    entry.endPos   = tailPos + raw;
    ring.append(entry);
    incrementalPlan(ring, ring.at(0).plan.entryVel);

    float newExitVel = ring.at(0).plan.exitVel;
    TEST_ASSERT_TRUE(newExitVel > 0.0f);

    traj.updateExitVel(newExitVel);

    for (int i = 0; i < 50000; i++) {
        traj.update(0.002f);
        if (traj.isComplete()) break;
    }
    TEST_ASSERT_TRUE(traj.isComplete());

    float finalVel = std::fabs(traj.currentVel);
    TEST_ASSERT_TRUE(finalVel > newExitVel * 0.5f);
}

// ===================================================================
// Loop planner tests
// ===================================================================

void test_loop_same_dir_continuous_velocity() {
    MotionCommand cmds[] = {
        relMove(10000, 5000, 10000, true),
        relMove(10000, 5000, 10000, true),
    };

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    planLoop(ring);

    float wrapJunction = ring.at(1).plan.exitVel;
    TEST_ASSERT_TRUE(wrapJunction > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, wrapJunction, ring.at(0).plan.entryVel);
    TEST_ASSERT_TRUE(wrapJunction <= 10000.0f);
}

void test_loop_reversal_at_wrap_zero_velocity() {
    MotionCommand cmds[] = {
        relMove(10000, 5000, 10000, true),
        relMove(-10000, 5000, 10000, true),
    };

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    planLoop(ring);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(1).plan.exitVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.at(0).plan.entryVel);
}

void test_loop_three_moves_wrap_velocity() {
    MotionCommand cmds[] = {
        relMove(10000, 10000, 20000, true),
        relMove(10000, 10000, 20000, true),
        relMove(10000, 10000, 20000, true),
    };

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 3, 0.0f);
    planLoop(ring);

    TEST_ASSERT_TRUE(ring.at(0).plan.exitVel > 0.0f);
    TEST_ASSERT_TRUE(ring.at(1).plan.exitVel > 0.0f);
    TEST_ASSERT_TRUE(ring.at(2).plan.exitVel > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, ring.at(2).plan.exitVel, ring.at(0).plan.entryVel);
}

void test_loop_single_move() {
    MotionCommand cmds[] = { relMove(10000, 5000, 10000, true) };

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 1, 0.0f);
    planLoop(ring);

    TEST_ASSERT_FLOAT_WITHIN(1.0f, ring.at(0).plan.entryVel, ring.at(0).plan.exitVel);
    TEST_ASSERT_TRUE(ring.at(0).plan.entryVel > 0.0f);
}

void test_loop_different_cruise_speeds() {
    MotionCommand cmds[] = {
        relMove(10000, 5000, 20000, true),
        relMove(10000, 5000, 5000, true),
    };

    BlockRingBuffer ring;
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    planLoop(ring);

    TEST_ASSERT_TRUE(ring.at(0).plan.exitVel <= 5000.0f + 1.0f);
    TEST_ASSERT_TRUE(ring.at(1).plan.exitVel <= 5000.0f + 1.0f);
}

// ===================================================================
// Controlled stop injection tests
// ===================================================================

void test_stop_injection_from_cruise() {
    float velocity = 10000.0f;
    float accel = 5000.0f;
    float expectedDist = velocity * velocity / (2.0f * accel);

    BlockRingBuffer ring;
    MotionCommand cmds[] = {
        relMove(20000, 5000, 10000, true),
        relMove(20000, 5000, 10000, true),
    };
    fillRingFromCommands(ring, cmds, 2, 0.0f);
    incrementalPlan(ring);

    float stopPos = 5000.0f;
    injectStopBlock(ring, stopPos, velocity, accel, true);

    TEST_ASSERT_EQUAL_INT(1, ring.count());

    const BlockEntry& decel = ring.peekHead();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expectedDist, decel.plan.dist);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, velocity, decel.plan.entryVel);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, decel.plan.exitVel);
    TEST_ASSERT_TRUE(decel.plan.forward);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, stopPos, decel.startPos);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, stopPos + expectedDist, decel.endPos);
}

void test_stop_injection_backward() {
    float velocity = 8000.0f;
    float accel = 4000.0f;
    float expectedDist = velocity * velocity / (2.0f * accel);

    BlockRingBuffer ring;
    MotionCommand cmds[] = { relMove(-20000, 4000, 8000, true) };
    fillRingFromCommands(ring, cmds, 1, 0.0f);
    incrementalPlan(ring);

    float stopPos = -5000.0f;
    injectStopBlock(ring, stopPos, velocity, accel, false);

    TEST_ASSERT_EQUAL_INT(1, ring.count());
    const BlockEntry& decel = ring.peekHead();
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expectedDist, decel.plan.dist);
    TEST_ASSERT_FALSE(decel.plan.forward);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, stopPos - expectedDist, decel.endPos);
}

void test_stop_injection_already_slow() {
    float velocity = 100.0f;
    float accel = 5000.0f;
    float expectedDist = velocity * velocity / (2.0f * accel);

    BlockRingBuffer ring;
    MotionCommand cmds[] = { relMove(20000, 5000, 10000, true) };
    fillRingFromCommands(ring, cmds, 1, 0.0f);
    incrementalPlan(ring);

    float stopPos = 100.0f;
    injectStopBlock(ring, stopPos, velocity, accel, true);

    TEST_ASSERT_EQUAL_INT(1, ring.count());
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expectedDist, ring.peekHead().plan.dist);
}

void test_stop_injection_zero_velocity() {
    BlockRingBuffer ring;
    MotionCommand cmds[] = { relMove(20000, 5000, 10000, true) };
    fillRingFromCommands(ring, cmds, 1, 0.0f);
    incrementalPlan(ring);

    float stopPos = 0.0f;
    injectStopBlock(ring, stopPos, 0.0f, 5000.0f, true);

    if (ring.count() > 0) {
        TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, ring.peekHead().plan.dist);
    }
}

// ===================================================================
// PDController tests
// ===================================================================

void test_deadband_suppresses_small_error() {
    PDController pd;
    pd.setGains(3.0f, 0.1f);
    pd.reset();

    float correction = pd.compute(100.0f, 0.0f, 99.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, correction);
}

void test_deadband_passes_larger_error() {
    PDController pd;
    pd.setGains(3.0f, 0.1f);
    pd.reset();

    float correction = pd.compute(100.0f, 0.0f, 95.0f, 0.001f);
    TEST_ASSERT_TRUE(correction > 0.0f);
}

void test_proportional_response() {
    PDController pd;
    pd.setGains(3.0f, 0.0f);
    pd.setDFilterAlpha(0.0f);
    pd.reset();

    pd.compute(100.0f, 0.0f, 90.0f, 0.001f);
    pd.reset();
    float correction = pd.compute(100.0f, 0.0f, 90.0f, 0.001f);

    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, correction);
}

void test_clamp_limits_correction() {
    PDController pd;
    pd.setGains(100.0f, 0.0f);
    pd.setDFilterAlpha(0.0f);
    pd.max_correction = 5000.0f;
    pd.reset();

    pd.compute(1000.0f, 0.0f, 0.0f, 0.001f);
    pd.reset();
    float correction = pd.compute(1000.0f, 0.0f, 0.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 5000.0f, correction);

    pd.reset();
    pd.compute(0.0f, 0.0f, 1000.0f, 0.001f);
    pd.reset();
    float negCorr = pd.compute(0.0f, 0.0f, 1000.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -5000.0f, negCorr);
}

void test_phase_lead() {
    PDController pd;
    pd.setGains(3.0f, 0.0f);
    pd.setDFilterAlpha(0.0f);
    pd.setPhaseLeadGain(0.001f);
    pd.reset();

    pd.compute(100.0f, 10000.0f, 100.0f, 0.001f);
    pd.reset();
    float correction = pd.compute(100.0f, 10000.0f, 100.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, correction);

    pd.setPhaseLeadGain(0.0f);
    pd.reset();
    pd.compute(100.0f, 10000.0f, 100.0f, 0.001f);
    pd.reset();
    float corrNoLead = pd.compute(100.0f, 10000.0f, 100.0f, 0.001f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, corrNoLead);
}

void test_ema_filter_suppresses_spikes() {
    PDController pd;
    pd.setGains(0.0f, 1.0f);
    pd.setDFilterAlpha(0.95f);
    pd.reset();

    float corr1 = pd.compute(100.0f, 0.0f, 0.0f, 0.001f);
    TEST_ASSERT_TRUE(std::fabs(corr1) <= 5001.0f);

    PDController pdNoFilter;
    pdNoFilter.setGains(0.0f, 1.0f);
    pdNoFilter.setDFilterAlpha(0.0f);
    pdNoFilter.reset();
    float corrNoFilter = pdNoFilter.compute(100.0f, 0.0f, 0.0f, 0.001f);
    TEST_ASSERT_TRUE(std::fabs(corrNoFilter) <= 5001.0f);
}

void test_d_term_responds_to_change() {
    PDController pd;
    pd.setGains(0.0f, 1.0f);
    pd.setDFilterAlpha(0.0f);
    pd.reset();

    pd.compute(110.0f, 0.0f, 100.0f, 0.001f);
    float correction = pd.compute(120.0f, 0.0f, 100.0f, 0.001f);

    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5000.0f, correction);
}

void test_reset_clears_state() {
    PDController pd;
    pd.setGains(3.0f, 0.1f);

    pd.compute(100.0f, 0.0f, 50.0f, 0.001f);
    pd.compute(100.0f, 0.0f, 60.0f, 0.001f);

    pd.reset();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pd.prev_error);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pd.filtered_rate);
}

// ===================================================================
// Entry point
// ===================================================================

void setUp(void) {}
void tearDown(void) {}

int main() {
    UNITY_BEGIN();

    // planChain
    RUN_TEST(test_single_move_entry_exit_zero);
    RUN_TEST(test_two_same_dir_junction_velocity);
    RUN_TEST(test_direction_reversal_junction_zero);
    RUN_TEST(test_short_segment_feasibility_clamp);
    RUN_TEST(test_reverse_pass_safe_stop);
    RUN_TEST(test_absolute_positioning);
    RUN_TEST(test_absolute_positioning_backward);
    RUN_TEST(test_accel_clamped_to_minimum);
    RUN_TEST(test_symmetric_chain);

    // TrajectoryPlanner
    RUN_TEST(test_trajectory_forward_completes);
    RUN_TEST(test_trajectory_backward_completes);
    RUN_TEST(test_trajectory_respects_cruise);
    RUN_TEST(test_trajectory_velocity_continuity);
    RUN_TEST(test_trajectory_scurve_jerk);

    // Ring buffer
    RUN_TEST(test_ring_starts_empty);
    RUN_TEST(test_ring_append_one);
    RUN_TEST(test_ring_append_and_read);
    RUN_TEST(test_ring_advance_head);
    RUN_TEST(test_ring_full);
    RUN_TEST(test_ring_wrap_around);
    RUN_TEST(test_ring_clear);
    RUN_TEST(test_ring_at_index);
    RUN_TEST(test_ring_mutable_at);

    // incrementalPlan
    RUN_TEST(test_incremental_matches_planchain_single);
    RUN_TEST(test_incremental_matches_planchain_two_same_dir);
    RUN_TEST(test_incremental_matches_planchain_reversal);
    RUN_TEST(test_incremental_matches_planchain_three_moves);
    RUN_TEST(test_incremental_append_after_commit);
    RUN_TEST(test_incremental_replan_lowers_exit_velocity);
    RUN_TEST(test_split_chain_replan_raises_junction);
    RUN_TEST(test_split_chain_trajectory_sees_updated_exit_vel);

    // Loop planner
    RUN_TEST(test_loop_same_dir_continuous_velocity);
    RUN_TEST(test_loop_reversal_at_wrap_zero_velocity);
    RUN_TEST(test_loop_three_moves_wrap_velocity);
    RUN_TEST(test_loop_single_move);
    RUN_TEST(test_loop_different_cruise_speeds);

    // Controlled stop
    RUN_TEST(test_stop_injection_from_cruise);
    RUN_TEST(test_stop_injection_backward);
    RUN_TEST(test_stop_injection_already_slow);
    RUN_TEST(test_stop_injection_zero_velocity);

    // PDController
    RUN_TEST(test_deadband_suppresses_small_error);
    RUN_TEST(test_deadband_passes_larger_error);
    RUN_TEST(test_proportional_response);
    RUN_TEST(test_clamp_limits_correction);
    RUN_TEST(test_phase_lead);
    RUN_TEST(test_ema_filter_suppresses_spikes);
    RUN_TEST(test_d_term_responds_to_change);
    RUN_TEST(test_reset_clears_state);

    return UNITY_END();
}
