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
    {
        // Ping answers: once per device and type (who is on the bus)
        static uint32_t seen[8];
        static uint8_t nSeen = 0;
        if (!(f[1] & 1) || f[4] < 8)
            return;
        const uint32_t uid = (uint32_t)f[5] << 24 | (uint32_t)f[6] << 16 | (uint32_t)f[7] << 8 | f[8];
        const uint16_t type = (uint16_t)f[11] << 8 | f[12];
        const uint32_t key = uid ^ type;
        for (uint8_t i = 0; i < nSeen; i++)
            if (seen[i] == key)
                return;
        if (nSeen < 8)
            seen[nSeen++] = key;
        Serial.printf("%s: ping answer of UID %08lX, version %u.%u, type %04X\n", prefix, (unsigned long)uid, f[9],
                      f[10], type);
        return;
    }
    Serial.printf("%s: cmd 0x%02X%s hash %02X%02X (%u bytes:", prefix, cmd, (f[1] & 1) ? " R" : "", f[2], f[3], f[4]);
    for (uint8_t i = 0; i < f[4] && i < 8; i++)
        Serial.printf(" %02X", f[5 + i]);
    Serial.println(")");
}

// Ping answer of a CS2 GUI (type FFFF), as the MaeCAN server sends it when
// it is the master: an MS2 then follows a Central Station
static const uint8_t GUI_PING[13] = {0x00, 0x31, 0x03, 0x00, 8, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF};

// Every 10 s (as the MaeCAN server): a ping on the bus, and the answer of
// the CS2 GUI
static void masterPing()
{
    static uint32_t last = 0;
    if (!Message::simulatesGui() || !CanBus::ready() || (last != 0 && millis() - last < 10000))
        return;
    last = millis() | 1;
    static const uint8_t ping[13] = {0x00, (uint8_t)(CAN_PING << 1), 0x03, 0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    CanBus::send(ping, sizeof(ping));
    CanBus::send(GUI_PING, sizeof(GUI_PING));
}

void rxTask(void *pvParameters)
{
    static uint8_t busFrame[13];
    bool dropStream = false; // answer of Rocrail to "ms2-conf": not sent to the bus
    while (true)
    {
        // Every complete frame of Rocrail: to the bus, then executed
        while (readRocrailFrame(rxBuffer, BUFFER_S))
        {
            const uint8_t cmd = (uint8_t)(rxBuffer[1] >> 1);
            // Test: the requests of the MS2 are answered by the command
            // station; the answers of Rocrail (requests echoed, streams) and
            // its ping answer (an MS2 of UID 0) stay off the bus
            bool keep = true;
            if (cmd == 0x20 && (rxBuffer[1] & 1) && memcmp(rxBuffer + 5, "ms2-conf", 8) == 0)
                dropStream = Message::answersMs2Conf();
            else if (cmd == 0x20 && (rxBuffer[1] & 1))
                dropStream = Message::simulatesGui();
            else if (cmd != 0x21)
                dropStream = false;
            // Rocrail also sends streams without echo (its loco names):
            // the command station is the only one to answer the MS2
            if (cmd == 0x21 && Message::simulatesGui())
                dropStream = true;
            if (cmd == CAN_PING && (rxBuffer[1] & 1) && Message::simulatesGui())
                keep = false;
            if (keep && !dropStream)
            {
                CanBus::send(rxBuffer, BUFFER_S);
                // Configuration files sent by Rocrail (to an MS2): every frame
                if (Message::busTrace() && CanBus::ready() && (cmd == 0x20 || cmd == 0x21 || cmd == CAN_PING))
                    traceBusFrame(rxBuffer, "Rocrail->CAN");
            }
            Message::fromBus = false;
            Message::decodeMsg(rxBuffer);
            Message::parse();
        }
        // Every frame of the bus: to Rocrail, then executed
        while (CanBus::receive(busFrame))
        {
            if (Message::busTrace())
                traceBusFrame(busFrame);
            // Test (can2udp, option -g): an MS2 answers a ping; a CS2 GUI
            // (type FFFF) answers too, so the MS2 follows a Central Station
            const uint16_t type = (uint16_t)busFrame[11] << 8 | busFrame[12];
            if (Message::simulatesGui() && busFrame[1] == ((CAN_PING << 1) | 1) && busFrame[4] == 8 &&
                (type & 0xFFF0) == 0x0030)
            {
                CanBus::send(GUI_PING, sizeof(GUI_PING));
                static bool told = false;
                if (!told)
                    Serial.println("CAN: ping answer of a CS2 GUI sent (the MS2 should follow a Central Station)");
                told = true;
            }
            sendToTcp(busFrame, sizeof(busFrame));
            Message::fromBus = true;
            Message::decodeMsg(busFrame);
            Message::parse();
        }
        Message::fromBus = false;
        masterPing();
        vTaskDelay(5 / portTICK_PERIOD_MS); // Avoid busy-waiting
    }
}
