// Minimal dual-core scaffold for ESP32 (Arduino)
// Provides example pinned tasks: control (core 0) and network (core 1).

#pragma once

#include <Arduino.h>

// Start the demo tasks. Call this from `setup()` when ready.
void start_dual_core_demo();

// Stop the demo tasks.
void stop_dual_core_demo();

// Demo command struct (shared)
struct Cmd { int cmd_id; int value; };

// Send a command to the network task. Returns true on success.
bool dispatch_network_cmd(int cmd_id, int value);
