#pragma once

#include <Arduino.h>

namespace webserver {

// Initialize the web server module on the specified core (0 or 1)
// Default is core 1 (Arduino framework default)
void initWebServer(int core = 1);

// Start the web server
void beginWebServer();

/**
 * @brief Broadcast a text message to all connected WebSocket clients.
 */
void broadcastWebSocket(const String &message);

} // namespace webserver
