/*

   Accessories.cpp

*/

#include "Accessories.h"

namespace
{
    struct Job
    {
        Dcc::Packet p;
        uint8_t left;    // packets still to send
        int64_t due;     // not before (µs, esp_timer)
    };

    const uint8_t JOBS = 16;
    Job sJobs[JOBS];
    uint8_t sCount = 0;

    // Last position of each DCC address (1 bit each)
    uint8_t sPos[(Dcc::MAX_ACCESSORY + 8) / 8];

    portMUX_TYPE sMux = portMUX_INITIALIZER_UNLOCKED;

    // Under sMux. Replaces a job for the same address and coil state.
    void add(const Dcc::Packet &p, int64_t due)
    {
        for (uint8_t i = 0; i < sCount; i++)
            if (sJobs[i].p.b[0] == p.b[0] && (sJobs[i].p.b[1] & 0xFE) == (p.b[1] & 0xFE))
            {
                sJobs[i].p = p;
                sJobs[i].left = Accessories::REPEATS;
                sJobs[i].due = due;
                return;
            }
        if (sCount == JOBS)
        {
            memmove(&sJobs[0], &sJobs[1], (JOBS - 1) * sizeof(Job)); // the oldest is dropped
            sCount--;
        }
        sJobs[sCount++] = {p, Accessories::REPEATS, due};
    }
}

namespace Accessories
{
    bool order(uint16_t id, uint8_t position, uint8_t current, uint16_t time10ms)
    {
        if (id < FIRST_DCC || id >= FIRST_DCC + Dcc::MAX_ACCESSORY)
        {
            Serial.printf("Accessory %04X: only the DCC ones (0x3800...) are handled for now\n", id);
            return false;
        }
        const uint16_t address = (uint16_t)(id - FIRST_DCC + 1);
        const bool pos = position != 0;
        const int64_t now = esp_timer_get_time();
        portENTER_CRITICAL(&sMux);
        if (pos)
            sPos[address / 8] |= (uint8_t)(1u << (address % 8));
        else
            sPos[address / 8] &= (uint8_t)~(1u << (address % 8));
        if (current)
        {
            add(Dcc::accessory(address, pos, true), now);
            if (time10ms)
                add(Dcc::accessory(address, pos, false), now + (int64_t)time10ms * 10000);
        }
        else
            add(Dcc::accessory(address, pos, false), now);
        portEXIT_CRITICAL(&sMux);
        Serial.printf("DCC accessory %u: %s, coil %s", address, pos ? "straight/green" : "round/red",
                      current ? "on" : "off");
        if (current && time10ms)
            Serial.printf(" for %u ms", time10ms * 10u);
        Serial.println();
        return true;
    }

    uint8_t position(uint16_t id)
    {
        if (id < FIRST_DCC || id >= FIRST_DCC + Dcc::MAX_ACCESSORY)
            return 0;
        const uint16_t address = (uint16_t)(id - FIRST_DCC + 1);
        return (sPos[address / 8] >> (address % 8)) & 1;
    }

    bool pending(int64_t now)
    {
        bool p = false;
        portENTER_CRITICAL(&sMux);
        for (uint8_t i = 0; i < sCount && !p; i++)
            p = sJobs[i].due <= now;
        portEXIT_CRITICAL(&sMux);
        return p;
    }

    uint8_t take(Dcc::Packet *out, uint8_t max, int64_t now)
    {
        uint8_t n = 0;
        portENTER_CRITICAL(&sMux);
        for (uint8_t i = 0; i < sCount && n < max;)
        {
            Job &j = sJobs[i];
            if (j.due > now)
            {
                i++;
                continue;
            }
            out[n++] = j.p;
            if (--j.left == 0)
            {
                memmove(&sJobs[i], &sJobs[i + 1], (sCount - i - 1) * sizeof(Job));
                sCount--;
            }
            else
                i++;
        }
        portEXIT_CRITICAL(&sMux);
        return n;
    }
}
