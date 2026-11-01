/*

   CanBus.cpp

*/

#include "CanBus.h"
#include <ACAN_ESP32.h>

bool CanBus::sReady = false;

static const uint32_t BIT_RATE = 250 * 1000; // Marklin CAN bus

bool CanBus::setup(gpio_num_t rx, gpio_num_t tx)
{
    ACAN_ESP32_Settings settings(BIT_RATE);
    settings.mRxPin = rx;
    settings.mTxPin = tx;
    const uint32_t error = ACAN_ESP32::can.begin(settings);
    sReady = (error == 0);
    if (sReady)
        Serial.printf("CAN bus: 250 kbit/s, RX GPIO %d, TX GPIO %d\n", (int)rx, (int)tx);
    else
        Serial.printf("CAN bus: not started (error 0x%lX), the command station works without it\n",
                      (unsigned long)error);
    return sReady;
}

void CanBus::send(const uint8_t *frames, size_t len)
{
    if (!sReady)
        return;
    for (size_t k = 0; k + 13 <= len; k += 13)
    {
        const uint8_t *f = frames + k;
        CANMessage m;
        m.id = ((uint32_t)f[0] << 24 | (uint32_t)f[1] << 16 | (uint32_t)f[2] << 8 | f[3]) & 0x1FFFFFFF;
        m.ext = true;
        m.len = f[4] > 8 ? 8 : f[4];
        for (uint8_t i = 0; i < 8; i++)
            m.data[i] = f[5 + i];
        // The driver keeps a few frames: wait a little if it is full
        uint8_t tries = 0;
        while (!ACAN_ESP32::can.tryToSend(m) && ++tries < 20)
            vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool CanBus::receive(uint8_t *frame13)
{
    if (!sReady)
        return false;
    CANMessage m;
    while (ACAN_ESP32::can.receive(m))
    {
        if (!m.ext || m.rtr)
            continue; // not a frame of the Marklin protocol
        frame13[0] = (uint8_t)(m.id >> 24);
        frame13[1] = (uint8_t)(m.id >> 16);
        frame13[2] = (uint8_t)(m.id >> 8);
        frame13[3] = (uint8_t)m.id;
        frame13[4] = m.len > 8 ? 8 : m.len;
        for (uint8_t i = 0; i < 8; i++)
            frame13[5 + i] = i < m.len ? m.data[i] : 0;
        return true;
    }
    return false;
}
