/*

   MmLocos.cpp

*/

#include "MmLocos.h"
#include <Preferences.h>

namespace
{
    struct Entry
    {
        uint8_t addr;   // 0: free
        uint16_t v;     // CS2 speed, 0 to 1000
        bool forward;
        uint8_t fn;     // bit n = Fn, F0 to F4
        uint8_t next;   // next packet of the refresh (0 to 8)
    };

    Entry sLocos[MmLocos::MAX_LOCOS];
    uint8_t sRr = 0;

    const uint8_t URGENT = 12;
    const uint8_t SPEED_REPEATS = 3;
    const uint8_t FUNCTION_REPEATS = 6;
    Mm::Packet sUrgent[URGENT];
    uint8_t sUrgentHead = 0, sUrgentCount = 0;

    volatile bool sEnabled = true;
    portMUX_TYPE sMux = portMUX_INITIALIZER_UNLOCKED;

    const char *NVS_NAMESPACE = "mm";
    const char *NVS_KEY = "list";

    Entry *find(uint8_t addr)
    {
        for (uint8_t i = 0; i < MmLocos::MAX_LOCOS; i++)
            if (sLocos[i].addr == addr)
                return &sLocos[i];
        return nullptr;
    }

    Entry *findOrAdd(uint16_t addr, bool *added)
    {
        *added = false;
        if (addr == 0 || addr > Mm::MAX_ADDRESS)
            return nullptr;
        if (Entry *e = find((uint8_t)addr))
            return e;
        Entry *e = find(0);
        if (e == nullptr)
            return nullptr;
        memset(e, 0, sizeof(*e));
        e->addr = (uint8_t)addr;
        e->forward = true;
        *added = true;
        return e;
    }

    uint8_t code(const Entry &e)
    {
        return Mm::codeFromCs2(e.v);
    }

    Mm::Packet speedPacket(const Entry &e)
    {
        return Mm::speed(e.addr, e.fn & 1, e.forward, code(e));
    }

    Mm::Packet functionPacket(const Entry &e, uint8_t fn)
    {
        return Mm::function(e.addr, e.fn & 1, code(e), fn, (e.fn >> fn) & 1);
    }

    void pushUrgent(const Mm::Packet &p)
    {
        if (sUrgentCount == URGENT)
        {
            sUrgentHead = (uint8_t)((sUrgentHead + 1) % URGENT);
            sUrgentCount--;
        }
        sUrgent[(sUrgentHead + sUrgentCount) % URGENT] = p;
        sUrgentCount++;
    }

    // speed, F1, speed, F2, speed, F3, speed, F4, function decoder
    Mm::Packet refreshPacket(Entry &e)
    {
        const uint8_t k = e.next;
        e.next = (uint8_t)((e.next + 1) % 9);
        if (k == 8)
            return Mm::functionDecoder(e.addr, (uint8_t)(e.fn >> 1 & 0x0F));
        if (k % 2 == 0)
            return speedPacket(e);
        return functionPacket(e, (uint8_t)(k / 2 + 1));
    }

    void save()
    {
        uint8_t a[MmLocos::MAX_LOCOS];
        uint8_t n = 0;
        portENTER_CRITICAL(&sMux);
        for (uint8_t i = 0; i < MmLocos::MAX_LOCOS; i++)
            if (sLocos[i].addr != 0)
                a[n++] = sLocos[i].addr;
        portEXIT_CRITICAL(&sMux);
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false))
            return;
        if (n == 0)
            prefs.remove(NVS_KEY);
        else
            prefs.putBytes(NVS_KEY, a, n);
        prefs.end();
    }

    // repeats: the packet of the order is sent as this many pairs, as the
    // Gleisbox (recorded: a function switched, 6 pairs in a row, about
    // 25 ms apart; a new speed, 2 or 3). With one pair only, the decoder of
    // the MM 60 followed the speed and F0 but not F1 to F4 (bench, 11 Nov).
    template <typename F>
    bool order(uint16_t addr, F change, uint8_t repeats = SPEED_REPEATS)
    {
        bool added;
        portENTER_CRITICAL(&sMux);
        Entry *e = findOrAdd(addr, &added);
        if (e != nullptr)
        {
            const Mm::Packet p = change(*e);
            for (uint8_t i = 0; i < repeats; i++)
                pushUrgent(p);
        }
        portEXIT_CRITICAL(&sMux);
        if (e == nullptr)
        {
            Serial.printf("MM %u: list full (%u locomotives) or address not 1 to 80, order ignored\n", addr,
                          MmLocos::MAX_LOCOS);
            return false;
        }
        if (added)
        {
            Serial.printf("MM %u added to the refresh\n", addr);
            save();
        }
        return true;
    }
}

namespace MmLocos
{
    void setup()
    {
        memset(sLocos, 0, sizeof(sLocos));
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false))
            return;
        if (prefs.isKey(NVS_KEY))
        {
            uint8_t a[MAX_LOCOS];
            const size_t len = prefs.getBytesLength(NVS_KEY);
            if (len > 0 && len <= MAX_LOCOS)
            {
                prefs.getBytes(NVS_KEY, a, len);
                uint8_t n = 0;
                for (uint8_t i = 0; i < len; i++)
                    if (a[i] >= 1 && a[i] <= Mm::MAX_ADDRESS)
                    {
                        sLocos[n].addr = a[i];
                        sLocos[n].forward = true;
                        n++;
                    }
                Serial.printf("MM: %u locomotives in the refresh (memory)\n", n);
            }
        }
        prefs.end();
    }

    void enable(bool on)
    {
        if (on != sEnabled)
            Serial.printf("MM2 %s on the track (master)\n", on ? "enabled" : "disabled");
        sEnabled = on;
    }

    bool enabled() { return sEnabled; }

    bool speed(uint16_t addr, uint16_t v)
    {
        if (v > 1000)
            v = 1000;
        return order(addr, [v](Entry &e)
                     {
                         e.v = v;
                         return speedPacket(e); });
    }

    bool direction(uint16_t addr, uint8_t dir)
    {
        return order(addr, [dir](Entry &e)
                     {
                         const bool before = e.forward;
                         if (dir == 1)
                             e.forward = true;
                         else if (dir == 2)
                             e.forward = false;
                         else if (dir == 3)
                             e.forward = !e.forward;
                         // As a CS2: a change of direction stops the locomotive
                         if (e.forward != before)
                             e.v = 0;
                         return speedPacket(e); });
    }

    bool function(uint16_t addr, uint8_t fn, bool on)
    {
        if (fn > 4)
            return false; // MM2: F0 to F4 only
        return order(addr, [fn, on](Entry &e)
                     {
                         if (on)
                             e.fn |= (uint8_t)(1u << fn);
                         else
                             e.fn &= (uint8_t)~(1u << fn);
                         return fn == 0 ? speedPacket(e) : functionPacket(e, fn); },
                     fn == 0 ? SPEED_REPEATS : FUNCTION_REPEATS);
    }

    bool emergencyStop(uint16_t addr)
    {
        return order(addr, [](Entry &e)
                     {
                         e.v = 0;
                         return speedPacket(e); });
    }

    bool forget(uint16_t addr)
    {
        bool found = false;
        portENTER_CRITICAL(&sMux);
        Entry *e = (addr >= 1 && addr <= Mm::MAX_ADDRESS) ? find((uint8_t)addr) : nullptr;
        if (e)
        {
            e->addr = 0;
            found = true;
        }
        portEXIT_CRITICAL(&sMux);
        if (found)
        {
            Serial.printf("MM %u out of the refresh (master)\n", addr);
            save();
        }
        return found;
    }

    uint16_t cs2Speed(uint16_t addr)
    {
        portENTER_CRITICAL(&sMux);
        const Entry *e = (addr >= 1 && addr <= Mm::MAX_ADDRESS) ? find((uint8_t)addr) : nullptr;
        const uint16_t v = e ? e->v : 0;
        portEXIT_CRITICAL(&sMux);
        return v;
    }

    uint8_t cs2Direction(uint16_t addr)
    {
        portENTER_CRITICAL(&sMux);
        const Entry *e = (addr >= 1 && addr <= Mm::MAX_ADDRESS) ? find((uint8_t)addr) : nullptr;
        const bool fwd = e ? e->forward : true;
        portEXIT_CRITICAL(&sMux);
        return fwd ? 1 : 2;
    }

    bool functionOn(uint16_t addr, uint8_t fn)
    {
        portENTER_CRITICAL(&sMux);
        const Entry *e = (addr >= 1 && addr <= Mm::MAX_ADDRESS) ? find((uint8_t)addr) : nullptr;
        const bool on = e && fn <= 4 && (e->fn >> fn & 1);
        portEXIT_CRITICAL(&sMux);
        return on;
    }

    uint8_t count()
    {
        uint8_t n = 0;
        portENTER_CRITICAL(&sMux);
        for (uint8_t i = 0; i < MAX_LOCOS; i++)
            if (sLocos[i].addr != 0)
                n++;
        portEXIT_CRITICAL(&sMux);
        return n;
    }

    bool urgent()
    {
        return sUrgentCount != 0;
    }

    uint8_t round(Mm::Packet *out, uint8_t max)
    {
        uint8_t n = 0;
        portENTER_CRITICAL(&sMux);
        // The orders first (the whole round if needed: they are short bursts)
        while (sUrgentCount && n < max)
        {
            out[n++] = sUrgent[sUrgentHead];
            sUrgentHead = (uint8_t)((sUrgentHead + 1) % URGENT);
            sUrgentCount--;
        }
        for (uint8_t k = 0; k < MAX_LOCOS && n < max; k++)
        {
            Entry &e = sLocos[sRr];
            sRr = (uint8_t)((sRr + 1) % MAX_LOCOS);
            if (e.addr != 0)
                out[n++] = refreshPacket(e);
        }
        portEXIT_CRITICAL(&sMux);
        return n;
    }

    void print()
    {
        Entry copy[MAX_LOCOS];
        portENTER_CRITICAL(&sMux);
        memcpy(copy, sLocos, sizeof(copy));
        portEXIT_CRITICAL(&sMux);
        uint8_t n = 0;
        for (uint8_t i = 0; i < MAX_LOCOS; i++)
        {
            const Entry &e = copy[i];
            if (e.addr == 0)
                continue;
            n++;
            Serial.printf("  MM  %5u : speed %4u/1000 (code %2u) %s, F0-F4 %c%c%c%c%c\n", e.addr, e.v, Mm::codeFromCs2(e.v),
                          e.forward ? "forward " : "backward", e.fn & 1 ? '1' : '0', e.fn & 2 ? '1' : '0',
                          e.fn & 4 ? '1' : '0', e.fn & 8 ? '1' : '0', e.fn & 16 ? '1' : '0');
        }
        Serial.printf("MM: %u locomotives in the refresh, MM2 %s, pulses at level %s\n", n,
                      sEnabled ? "on" : "off (master)", Mm::pulseLevel ? "1" : "0");
    }
}
