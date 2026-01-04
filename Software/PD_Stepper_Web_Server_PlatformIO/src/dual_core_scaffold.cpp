#include "dual_core_scaffold.h"
#include <Arduino.h>

// FreeRTOS API is available in Arduino environment
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
}

static TaskHandle_t ctrlHandle = NULL;
static TaskHandle_t netHandle = NULL;
static QueueHandle_t cmdQueue = NULL;
static QueueHandle_t netCmdQueue = NULL;

static void IRAM_ATTR controller_task(void *pvParameters){
  const TickType_t period = pdMS_TO_TICKS(5); // 200 Hz control loop
  TickType_t lastWake = xTaskGetTickCount();
  Cmd cmd;
  for(;;){
    // wait until next period
    vTaskDelayUntil(&lastWake, period);
    // process incoming commands non-blocking
    if (cmdQueue && xQueueReceive(cmdQueue, &cmd, 0) == pdPASS){
      // handle command (demo: toggle LED)
      if (cmd.cmd_id == 1){ digitalWrite(LED_BUILTIN, cmd.value); }
    }
    // perform time-critical work here: read encoder, compute control, update pulse generator
    // Keep this short and deterministic.
  }
}

static void network_task(void *pvParameters){
  Cmd cmd;
  for(;;){
    // First, check for commands targeted at the network task
    if (netCmdQueue && xQueueReceive(netCmdQueue, &cmd, pdMS_TO_TICKS(100)) == pdPASS)
    {
        USBSerial.printf("NetworkTask running on core %d\n", xPortGetCoreID());
        USBSerial.println("Network task received command: ID=" + String(cmd.cmd_id) + " Value=" + String(cmd.value));   
        // Process the incoming network-targeted command
        if (cmd.cmd_id == 1)
        { 
            digitalWrite(LED_BUILTIN, cmd.value); 
        }
      // Add more handlers here
    }

    // Simulate or perform lower-priority networking work
    // Optionally send demo command to controller to show bidirectional messaging
    if (cmdQueue){
      cmd.cmd_id = 2; // demo id for controller
      cmd.value = !digitalRead(LED_BUILTIN);
      xQueueSend(cmdQueue, &cmd, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void start_dual_core_demo(){
  if (cmdQueue) return; // already started
  //pinMode(LED_BUILTIN, OUTPUT);
  cmdQueue = xQueueCreate(8, sizeof(Cmd));
  netCmdQueue = xQueueCreate(8, sizeof(Cmd));
  // Controller: high priority, pin to core 1
  USBSerial.printf("Pin controllerTask to core %d\n", 1);
  xTaskCreatePinnedToCore(controller_task, "ctrl", 4096, NULL, configMAX_PRIORITIES-1, &ctrlHandle, 1);
  // Network: lower priority, pin to core 0
  USBSerial.printf("Pin networkTask to core %d\n", 0);
  xTaskCreatePinnedToCore(network_task, "net", 4096, NULL, 1, &netHandle, 0);
}

void stop_dual_core_demo(){
  if (ctrlHandle) { vTaskDelete(ctrlHandle); ctrlHandle = NULL; }
  if (netHandle) { vTaskDelete(netHandle); netHandle = NULL; }
  if (cmdQueue) { vQueueDelete(cmdQueue); cmdQueue = NULL; }
  if (netCmdQueue) { vQueueDelete(netCmdQueue); netCmdQueue = NULL; }
}

bool dispatch_network_cmd(int cmd_id, int value)
{
  if (!netCmdQueue) 
  {
    return false;
  }
  Cmd cmd;
  cmd.cmd_id = cmd_id;
  cmd.value = value;
  BaseType_t ok = xQueueSend(netCmdQueue, &cmd, pdMS_TO_TICKS(0));
  return ok == pdTRUE;
}
