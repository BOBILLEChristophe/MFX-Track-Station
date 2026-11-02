/*

   RxTask.h


*/

#ifndef RX_TASK_H
#define RX_TASK_H

#include <Arduino.h>
#include "Message.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Defined in main.cpp: works with WiFi and Ethernet, and accepts reconnections.
extern uint8_t rxBuffer[];
extern const uint8_t BUFFER_S;
bool readRocrailFrame(uint8_t *buf, size_t len);
bool sendToTcp(const uint8_t *buf, size_t len); // to Rocrail only

void rxTask(void *pvParameters);

#endif // RX_TASK_H
