/*

   DccLocos.cpp

*/

#include "DccLocos.h"
#include <Preferences.h>

namespace
{
    struct Entry
    {
        uint16_t addr;    // 0: free
        uint8_t prot;     // CS2 protocol (0 to 4, see DccLocos.h)
        uint16_t v;       // CS2 speed, 0 to 1000
        bool forward;
        bool emergency;   // emergency stop until the next speed order
        uint32_t fn;      // bit n = Fn
        bool f13, f21;    // a function of F13-F20 / F21-F28 was switched on
        uint8_t next;     // next packet of the refresh (0 = speed)
    };

    Entry sLocos[DccLocos::MAX_LOCOS];
    uint8_t sRr = 0; // locomotive the next round starts with

    const uint8_t URGENT = 8;
    Dcc::Packet sUrgent[URGENT];
    uint8_t sUrgentHead = 0, sUrgentCount = 0;

    volatile bool sEnabled = true;
    portMUX_TYPE sMux = portMUX_INITIALIZER_UNLOCKED;

    const char *NVS_NAMESPACE = "dcc";
    const char *NVS_KEY = "list";

    struct __attribute__((packed)) Record
    {
        uint16_t addr;
        uint8_t prot;
    };

    uint8_t steps(uint8_t prot)
    {
        switch (prot)
        {
        case 1:
            return Dcc::STEPS_14;
        case 2:
        case 4:
            return Dcc::STEPS_128;
        default:
            return Dcc::STEPS_28;
        }
    }

    bool forceLong(uint8_t prot)
    {
        return prot == 3 || prot == 4;
    }

    // Under sMux
    Entry *find(uint16_t addr)
    {
        for (uint8_t i = 0; i < DccLocos::MAX_LOCOS; i++)
            if (sLocos[i].addr == addr)
                return &sLocos[i];
        return nullptr;
    }

    // Under sMux. *added: a new entry was taken
    Entry *findOrAdd(uint16_t addr, bool *added)
    {
        *added = false;
        if (addr == 0 || addr > DccLocos::MAX_ADDRESS)
            return nullptr;
        if (Entry *e = find(addr))
            return e;
        Entry *e = find(0);
        if (e == nullptr)
            return nullptr;
        memset(e, 0, sizeof(*e));
        e->addr = addr;
        e->forward = true;
        *added = true;
        return e;
    }

    Dcc::Packet speedPacket(const Entry &e)
    {
        return Dcc::speed(e.addr, forceLong(e.prot), steps(e.prot), e.forward,
                          Dcc::stepFromCs2(e.v, steps(e.prot)), e.emergency, e.fn & 1);
    }

    Dcc::Packet groupPacket(const Entry &e, Dcc::Group g)
    {
        return Dcc::functions(e.addr, forceLong(e.prot), g, e.fn);
    }

    Dcc::Group groupOf(uint8_t fn)
    {
        if (fn <= 4)
            return Dcc::F0_F4;
        if (fn <= 8)
            return Dcc::F5_F8;
        if (fn <= 12)
            return Dcc::F9_F12;
        if (fn <= 20)
            return Dcc::F13_F20;
        return Dcc::F21_F28;
    }

    // Under sMux: the oldest one is dropped when full
    void pushUrgent(const Dcc::Packet &p)
    {
        if (sUrgentCount == URGENT)
        {
            sUrgentHead = (uint8_t)((sUrgentHead + 1) % URGENT);
            sUrgentCount--;
        }
        sUrgent[(sUrgentHead + sUrgentCount) % URGENT] = p;
        sUrgentCount++;
    }

    // Next packet of the refresh for this locomotive (under sMux)
    Dcc::Packet refreshPacket(Entry &e)
    {
        for (;;)
        {
            const uint8_t k = e.next;
            e.next = (uint8_t)((e.next + 1) % 6);
            switch (k)
            {
            case 0:
                return speedPacket(e);
            case 1:
                return groupPacket(e, Dcc::F0_F4);
            case 2:
                return groupPacket(e, Dcc::F5_F8);
            case 3:
                return groupPacket(e, Dcc::F9_F12);
            case 4:
                if (e.f13)
                    return groupPacket(e, Dcc::F13_F20);
                break;
            case 5:
                if (e.f21)
                    return groupPacket(e, Dcc::F21_F28);
                break;
            }
        }
    }

    // Writes the list (address, protocol). Not under sMux: it takes time.
    void save()
    {
        Record r[DccLocos::MAX_LOCOS];
        uint8_t n = 0;
        portENTER_CRITICAL(&sMux);
        for (uint8_t i = 0; i < DccLocos::MAX_LOCOS; i++)
            if (sLocos[i].addr != 0)
            {
                r[n].addr = sLocos[i].addr;
                r[n].prot = sLocos[i].prot;
                n++;
            }
        portEXIT_CRITICAL(&sMux);
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false))
            return;
        if (n == 0)
            prefs.remove(NVS_KEY);
        else
            prefs.putBytes(NVS_KEY, r, n * sizeof(Record));
        prefs.end();
    }

    // Common part of the orders: finds (or adds) the locomotive, applies
    // `change` and queues the packet it returns
    template <typename F>
    bool order(uint16_t addr, F change)
    {
        bool added;
        portENTER_CRITICAL(&sMux);
        Entry *e = findOrAdd(addr, &added);
        if (e != nullptr)
            pushUrgent(change(*e));
        portEXIT_CRITICAL(&sMux);
        if (e == nullptr)
        {
            Serial.printf("DCC %u: list full (%u locomotives) or wrong address, order ignored\n", addr,
                          DccLocos::MAX_LOCOS);
            return false;
        }
        if (added)
        {
            Serial.printf("DCC %u added to the refresh\n", addr);
            save();
        }
        return true;
    }
}

namespace DccLocos
{
    void setup()
    {
        memset(sLocos, 0, sizeof(sLocos));
        Preferences prefs;
        if (!prefs.begin(NVS_NAMESPACE, false))
            return;
        if (prefs.isKey(NVS_KEY))
        {
            Record r[MAX_LOCOS];
            const size_t len = prefs.getBytesLength(NVS_KEY);
            if (len > 0 && len % sizeof(Record) == 0 && len / sizeof(Record) <= MAX_LOCOS)
            {
                prefs.getBytes(NVS_KEY, r, len);
                const uint8_t n = (uint8_t)(len / sizeof(Record));
                for (uint8_t i = 0; i < n; i++)
                {
                    sLocos[i].addr = r[i].addr;
                    sLocos[i].prot = r[i].prot <= 4 ? r[i].prot : 0;
                    sLocos[i].forward = true;
                }
                Serial.printf("DCC: %u locomotives in the refresh (memory)\n", n);
            }
        }
        prefs.end();
    }

    void enable(bool on)
    {
        if (on != sEnabled)
            Serial.printf("DCC %s on the track (master)\n", on ? "enabled" : "disabled");
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
                         e.emergency = false;
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
        if (fn > 28)
            return false; // F29 and beyond: not sent in DCC
        return order(addr, [fn, on](Entry &e)
                     {
                         if (on)
                             e.fn |= 1UL << fn;
                         else
                             e.fn &= ~(1UL << fn);
                         if (on && fn >= 13 && fn <= 20)
                             e.f13 = true;
                         if (on && fn >= 21)
                             e.f21 = true;
                         // 14 steps: F0 is in the speed packet
                         if (fn == 0 && steps(e.prot) == Dcc::STEPS_14)
                             return speedPacket(e);
                         return groupPacket(e, groupOf(fn)); });
    }

    bool emergencyStop(uint16_t addr)
    {
        return order(addr, [](Entry &e)
                     {
                         e.v = 0;
                         e.emergency = true;
                         return speedPacket(e); });
    }

    bool protocol(uint16_t addr, uint8_t prot)
    {
        if (prot > 4)
            return false;
        bool changed = false;
        const bool ok = order(addr, [prot, &changed](Entry &e)
                              {
                                  changed = e.prot != prot;
                                  e.prot = prot;
                                  return speedPacket(e); });
        if (ok && changed)
        {
            Serial.printf("DCC %u: %u steps%s\n", addr, steps(prot), forceLong(prot) ? ", long address" : "");
            save();
        }
        return ok;
    }

    bool forget(uint16_t addr)
    {
        bool found = false;
        portENTER_CRITICAL(&sMux);
        if (Entry *e = find(addr))
        {
            e->addr = 0;
            found = true;
        }
        portEXIT_CRITICAL(&sMux);
        if (found)
        {
            Serial.printf("DCC %u out of the refresh (master)\n", addr);
            save();
        }
        return found;
    }

    bool known(uint16_t addr)
    {
        portENTER_CRITICAL(&sMux);
        const bool k = addr != 0 && find(addr) != nullptr;
        portEXIT_CRITICAL(&sMux);
        return k;
    }

    uint16_t cs2Speed(uint16_t addr)
    {
        portENTER_CRITICAL(&sMux);
        const Entry *e = addr ? find(addr) : nullptr;
        const uint16_t v = e ? e->v : 0;
        portEXIT_CRITICAL(&sMux);
        return v;
    }

    uint8_t cs2Direction(uint16_t addr)
    {
        portENTER_CRITICAL(&sMux);
        const Entry *e = addr ? find(addr) : nullptr;
        const bool fwd = e ? e->forward : true;
        portEXIT_CRITICAL(&sMux);
        return fwd ? 1 : 2;
    }

    bool functionOn(uint16_t addr, uint8_t fn)
    {
        portENTER_CRITICAL(&sMux);
        const Entry *e = addr ? find(addr) : nullptr;
        const bool on = e && fn < 32 && (e->fn >> fn & 1);
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

    uint8_t round(Dcc::Packet *out, uint8_t max)
    {
        uint8_t n = 0;
        portENTER_CRITICAL(&sMux);
        // The orders first, half of the round at most
        while (sUrgentCount && n < (max + 1) / 2)
        {
            out[n++] = sUrgent[sUrgentHead];
            sUrgentHead = (uint8_t)((sUrgentHead + 1) % URGENT);
            sUrgentCount--;
        }
        // Then one packet per locomotive, in turn
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
            Serial.printf("  DCC %5u : %3u steps%s, speed %4u/1000 %s%s, functions %08lX\n", e.addr, steps(e.prot),
                          Dcc::isLong(e.addr, forceLong(e.prot)) ? " (long)" : "        ", e.v,
                          e.forward ? "forward " : "backward", e.emergency ? " (emergency stop)" : "",
                          (unsigned long)e.fn);
        }
        Serial.printf("DCC: %u locomotives in the refresh, DCC %s\n", n, sEnabled ? "on" : "off (master)");
    }
}
