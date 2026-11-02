/*

  RxTask.cpp

  The frames of Rocrail (TCP) and of the CAN bus (an MS2...) are handled
  here, by one task, so that Message (static state) parses one frame at a
  time. As a CS2 between its bus and Rocrail, the command station passes
  each frame to the other side, then executes it as a Gleisbox would.

*/

#include "RxTask.h"
#include "CanBus.h"

// Prints a frame received from the bus (pings left out: every few seconds),
// or sent by Rocrail to the bus (prefix "Rocrail->CAN")
static void traceBusFrame(const uint8_t *f, const char *prefix = "CAN")
{
    const uint8_t cmd = (uint8_t)(f[1] >> 1);
    if (cmd == CAN_PING)
        return;
    Serial.printf("%s: cmd 0x%02X%s hash %02X%02X (%u bytes:", prefix, cmd, (f[1] & 1) ? " R" : "", f[2], f[3], f[4]);
    for (uint8_t i = 0; i < f[4] && i < 8; i++)
        Serial.printf(" %02X", f[5 + i]);
    Serial.println(")");
}

void rxTask(void *pvParameters)
{
    static uint8_t busFrame[13];
    while (true)
    {
        // Every complete frame of Rocrail: to the bus, then executed
        while (readRocrailFrame(rxBuffer, BUFFER_S))
        {
            CanBus::send(rxBuffer, BUFFER_S);
            // Configuration files sent by Rocrail (to an MS2): every frame
            const uint8_t cmd = (uint8_t)(rxBuffer[1] >> 1);
            if (Message::busTrace() && CanBus::ready() && (cmd == 0x20 || cmd == 0x21))
                traceBusFrame(rxBuffer, "Rocrail->CAN");
            Message::fromBus = false;
            Message::decodeMsg(rxBuffer);
            Message::parse();
        }
        // Every frame of the bus: to Rocrail, then executed
        while (CanBus::receive(busFrame))
        {
            if (Message::busTrace())
                traceBusFrame(busFrame);
            sendToTcp(busFrame, sizeof(busFrame));
            Message::fromBus = true;
            Message::decodeMsg(busFrame);
            Message::parse();
        }
        Message::fromBus = false;
        vTaskDelay(5 / portTICK_PERIOD_MS); // Avoid busy-waiting
    }
}
