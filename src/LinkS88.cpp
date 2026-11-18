/*

   LinkS88.cpp

   See LinkS88.h.

*/

#include "LinkS88.h"
#include "CanBus.h"
#include "ConfigStore.h"

namespace LinkS88
{
    static const uint8_t CMD_SYSTEM = 0x00;
    static const uint8_t CMD_S88_EVENT = 0x11;
    static const uint8_t CMD_PING = 0x18;
    static const uint8_t CMD_BOOTLOADER = 0x1B;
    static const uint8_t SUB_CONFIG = 0x0B;   // system: configuration value
    static const uint8_t SUB_IDENT = 0x0C;    // system: device identifier
    static const uint16_t HASH = 0x0300;      // as wake-up-links88
    static const uint16_t L88_UID_HIGH = 0x5338; // "S8"
    static const uint32_t SEARCH_MS = 10000;
    static const uint32_t GAP_MS = 10;
    static const uint8_t MAX_L88 = 4;
    static const uint8_t MAX_MODULES = 31;
    static const char *FILE_NAME = "l88"; // /config/l88.json
    static const uint8_t FILE_VERSION = 1;

    static uint8_t sLength[3] = {0, 0, 0};
    static uint16_t sCycleMs = 100;
    static volatile bool sReconfigure = false;

    static uint32_t sKnown[MAX_L88];
    static uint8_t sCount = 0;
    static uint32_t sEvents = 0;
    static uint16_t sLastContact = 0;
    static uint8_t sLastState = 0;

    // Frames to send, 10 ms apart (written and read by RxTask only)
    static const uint8_t QUEUE = 24;
    static uint8_t sQueue[QUEUE][13];
    static uint8_t sHead = 0, sTail = 0;
    static uint32_t sLastSend = 0;

    static uint32_t be32(const uint8_t *p)
    {
        return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
    }

    static void push(uint8_t cmd, const uint8_t *data, uint8_t len, bool answerBit = false)
    {
        const uint8_t next = (uint8_t)((sHead + 1) % QUEUE);
        if (next == sTail)
            return; // full: the next search will do it again
        uint8_t *f = sQueue[sHead];
        memset(f, 0, 13);
        f[1] = (uint8_t)(cmd << 1 | (answerBit ? 1 : 0));
        f[2] = (uint8_t)(HASH >> 8);
        f[3] = (uint8_t)HASH;
        f[4] = len;
        if (len)
            memcpy(f + 5, data, len);
        sHead = next;
    }

    // Command with the UID of the L88 first, then n bytes
    static void pushUid(uint8_t cmd, uint32_t uid, const uint8_t *rest, uint8_t n, bool answerBit = false)
    {
        uint8_t d[8] = {(uint8_t)(uid >> 24), (uint8_t)(uid >> 16), (uint8_t)(uid >> 8), (uint8_t)uid};
        memcpy(d + 4, rest, n);
        push(cmd, d, (uint8_t)(4 + n), answerBit);
    }

    static bool isL88(uint32_t uid)
    {
        return (uid >> 16) == L88_UID_HIGH;
    }

    static int indexOf(uint32_t uid)
    {
        for (uint8_t i = 0; i < sCount; i++)
            if (sKnown[i] == uid)
                return i;
        return -1;
    }

    static void forget(uint32_t uid)
    {
        const int i = indexOf(uid);
        if (i < 0)
            return;
        sKnown[i] = sKnown[--sCount];
    }

    static void wakeUp(uint32_t uid)
    {
        static const uint8_t I[] = {0xE4};
        static const uint8_t II[] = {0x11};
        pushUid(CMD_BOOTLOADER, uid, I, 1);
        pushUid(CMD_BOOTLOADER, uid, II, 1);
        // Sent with the answer bit set (identifier 0x00010300), as a CS2
        // does and as wake-up-links88 copies it
        const uint8_t ident[] = {SUB_IDENT, 0x00, (uint8_t)uid};
        pushUid(CMD_SYSTEM, uid, ident, 3, true);
        push(CMD_PING, nullptr, 0); // its answer starts the configuration
    }

    static void sendConfig(uint32_t uid)
    {
        for (uint8_t b = 0; b < 3; b++)
        {
            const uint8_t v[] = {SUB_CONFIG, (uint8_t)(2 + b), 0x00, sLength[b]};
            pushUid(CMD_SYSTEM, uid, v, 4);
        }
        for (uint8_t b = 0; b < 3; b++)
        {
            const uint8_t v[] = {SUB_CONFIG, (uint8_t)(5 + b), (uint8_t)((sCycleMs >> 8) & 0x03), (uint8_t)sCycleMs};
            pushUid(CMD_SYSTEM, uid, v, 4);
        }
        Serial.printf("L88 %08lX: configured, modules %u / %u / %u, cycle %u ms\n", (unsigned long)uid, sLength[0],
                      sLength[1], sLength[2], sCycleMs);
    }

    void setup()
    {
        JsonDocument doc;
        if (!ConfigStore::load(FILE_NAME, doc))
            return; // defaults: no S88 module, 100 ms
        JsonArrayConst bus = doc["bus"];
        for (uint8_t b = 0; b < 3 && b < bus.size(); b++)
            sLength[b] = min((uint8_t)(bus[b] | 0), MAX_MODULES);
        sCycleMs = constrain((uint16_t)(doc["cycleMs"] | 100), (uint16_t)10, (uint16_t)1000);
        Serial.printf("L88: modules %u / %u / %u, cycle %u ms (/config/l88.json)\n", sLength[0], sLength[1],
                      sLength[2], sCycleMs);
    }

    void onBusFrame(const uint8_t *f)
    {
        const uint8_t cmd = (uint8_t)(f[1] >> 1);
        const bool answer = f[1] & 1;
        const uint8_t len = f[4];
        const uint8_t *d = f + 5;
        if (!answer || len < 4)
            return;
        const uint32_t uid = be32(d);

        if (cmd == CMD_BOOTLOADER && len == 8 && isL88(uid))
        {
            // Sleeping (just switched on, or switched off and on)
            Serial.printf("L88 %08lX found in its bootloader: wake-up\n", (unsigned long)uid);
            forget(uid);
            wakeUp(uid);
        }
        else if (cmd == CMD_PING && len == 8 && isL88(uid) && indexOf(uid) < 0)
        {
            // Awake, not configured yet
            if (sCount < MAX_L88)
                sKnown[sCount++] = uid;
            sendConfig(uid);
        }
        else if (cmd == CMD_SYSTEM && len == 7 && isL88(uid) && d[4] == SUB_CONFIG && d[6] == 0)
            Serial.printf("L88 %08lX: value %u refused (out of range?)\n", (unsigned long)uid, d[5]);
        else if (cmd == CMD_S88_EVENT && len == 8)
        {
            sEvents = sEvents + 1;
            const uint16_t contact = (uint16_t)d[2] << 8 | d[3];
            sLastContact = contact;
            sLastState = d[5] ? 1 : 0;
            Serial.printf("S88: contact %u %s\n", contact, d[5] ? "on" : "off");
        }
    }

    void loop()
    {
        if (!CanBus::ready())
            return;
        const uint32_t now = millis();
        if (sReconfigure)
        {
            sReconfigure = false;
            for (uint8_t i = 0; i < sCount; i++)
                sendConfig(sKnown[i]);
        }
        static uint32_t lastSearch = 0;
        if (lastSearch == 0 || now - lastSearch >= SEARCH_MS)
        {
            lastSearch = now | 1;
            push(CMD_BOOTLOADER, nullptr, 0);
            push(CMD_PING, nullptr, 0);
        }
        if (sTail != sHead && now - sLastSend >= GAP_MS)
        {
            CanBus::send(sQueue[sTail], 13);
            sTail = (uint8_t)((sTail + 1) % QUEUE);
            sLastSend = now;
        }
    }

    bool configure(uint8_t bus1, uint8_t bus2, uint8_t bus3, uint16_t cycleMs)
    {
        sLength[0] = min(bus1, MAX_MODULES);
        sLength[1] = min(bus2, MAX_MODULES);
        sLength[2] = min(bus3, MAX_MODULES);
        sCycleMs = constrain(cycleMs, (uint16_t)10, (uint16_t)1000);
        sReconfigure = true;
        JsonDocument doc;
        doc["version"] = FILE_VERSION;
        JsonArray bus = doc["bus"].to<JsonArray>();
        for (uint8_t b = 0; b < 3; b++)
            bus.add(sLength[b]);
        doc["cycleMs"] = sCycleMs;
        const bool ok = ConfigStore::save(FILE_NAME, doc);
        if (!ok)
            Serial.println("L88: /config/l88.json not written (applied until the next restart)");
        return ok;
    }

    void status(JsonObject out)
    {
        JsonArray bus = out["bus"].to<JsonArray>();
        for (uint8_t b = 0; b < 3; b++)
            bus.add(sLength[b]);
        out["cycleMs"] = sCycleMs;
        JsonArray found = out["found"].to<JsonArray>();
        char uid[9];
        for (uint8_t i = 0; i < sCount; i++)
        {
            snprintf(uid, sizeof(uid), "%08lX", (unsigned long)sKnown[i]);
            found.add(uid);
        }
        out["events"] = sEvents;
        out["lastContact"] = sLastContact;
        out["lastState"] = sLastState;
    }

    void printStatus()
    {
        Serial.printf("\nL88: modules bus 1 = %u, bus 2 = %u, bus 3 = %u, cycle %u ms; %u L88 found", sLength[0],
                      sLength[1], sLength[2], sCycleMs, sCount);
        for (uint8_t i = 0; i < sCount; i++)
            Serial.printf(" %08lX", (unsigned long)sKnown[i]);
        Serial.printf("; %lu S88 events\n", (unsigned long)sEvents);
    }
}
