/*

   Discovery.cpp

*/

#include "Discovery.h"
#include "Centrale.h"
#include "Feedback.h"
#include "Loco.h"
#include "Message.h"
#include "MfxFrame.h"
#include "MfxConfig.h"
#include "Cs2Config.h"
#include "LocoStore.h"
#include "MfxSignal.h"
#include <unordered_map>
#include <Preferences.h>

// Addresses of the known locomotives, sorted (at most max)
static uint8_t knownAddresses(uint16_t *out, uint8_t max)
{
    static Loco copies[32];
    const uint8_t n = Loco::list(copies, max < 32 ? max : 32);
    for (uint8_t i = 0; i < n; i++)
        out[i] = copies[i].addr();
    return n;
}

static const uint32_t PERIOD_MS = 2000;        // search for new decoders
static const uint32_t SAME_DECODER_MS = 30000; // pause if a decoder answers again
static const uint8_t TRIES = 4;                // a missed answer gives a wrong bit
// Pause before asking again a question without answer: the 040 DE (ESU)
// stays silent to about one question in three, in a regular pattern, and
// a question asked again 30 ms later was lost as well (test bench, 2 Nov:
// asked again after 70 ms, it answers; never 3 silences in a row)
static const uint32_t MISS_PAUSE_MS = 70;
static const uint32_t POWERED_MS = 3000;       // track on before reading the configurations
static const uint32_t RETRY_MS = 20000;        // a decoder that did not answer is tried again

void Discovery::setup()
{
    if (!Feedback::ready())
    {
        Serial.println("Discovery: no feedback, automatic discovery disabled");
        return;
    }
    xTaskCreatePinnedToCore(task, "Discovery", 4 * 1024, NULL, 4, NULL, 0);
}

/*
 Search: do decoders not yet registered have a UID starting with the first
 `count` bits of `uid`? A "no" is asked TRIES times, MISS_PAUSE_MS apart:
 one missed answer would give a wrong bit (the final check would then fail,
 and the search restart).

 After an answer, the next question waits ANSWER_PAUSE_MS, as the Gleisbox
 does (130 to 175 ms measured, October 2026; 38 ms only after a "no"): an
 ESU decoder (040 DE 9) that has just answered ignores a question sent
 30 ms later, which made the search lose a bit every time.
*/
static const uint32_t ANSWER_PAUSE_MS = 150;
// Presence: a weak answer counts too (see verifyNext()): a locomotive that
// runs answers with 34 to 148 regular intervals (its own motor spoils its
// answer, test bench 2 Nov), the reference window holding at most 22; a
// locomotive taken off the track gives at most 4, even with another one
// running fast with its sounds.
static const uint16_t WEAK_ANSWER_REGULAR = 20;
static const uint16_t WEAK_ANSWER_MARGIN = 10; // more than in the reference window
// A reference window with this many regular intervals: the measure is not
// reliable, a missing answer is not counted against the locomotive
static const uint16_t NOISY_REFERENCE = 10;

static bool weakAnswer(const Feedback::Result &r)
{
    return r.windowRegular >= WEAK_ANSWER_REGULAR && r.windowRegular >= r.referenceRegular + WEAK_ANSWER_MARGIN;
}

bool Discovery::probe(uint8_t count, uint32_t uid)
{
    MfxFrame f;
    f.broadcast();
    f.search(count, uid);
    f.finish();
    for (uint8_t i = 0; i < TRIES; i++)
    {
        if (Feedback::ask(f))
        {
            vTaskDelay(pdMS_TO_TICKS(ANSWER_PAUSE_MS));
            return true;
        }
        if (i + 1 < TRIES)
            vTaskDelay(pdMS_TO_TICKS(MISS_PAUSE_MS));
    }
    return false;
}

/*
 As the Gleisbox (recorded, October 2026): each bit is first asked at 1
 (the rest of the UID all ones). An answer keeps the 1. Without an answer
 the bit is 0, and the question is asked again with the 0: the decoder must
 answer it, otherwise the search stops (a missed answer, or the decoder
 left the track) and starts again later.
*/
bool Discovery::findUid(uint32_t *found)
{
    // Count 0: every decoder not yet registered answers, whatever the UID.
    // The UID is all ones, as a Marklin MS2 sends it.
    if (!probe(0, 0xFFFFFFFFUL))
        return false;

    uint32_t uid = 0xFFFFFFFFUL;
    for (uint8_t count = 1; count <= 32; count++)
    {
        if (probe(count, uid))
            continue; // the bit is 1
        uid &= ~(1UL << (32 - count));
        if (!probe(count, uid))
        {
            // neither 1 nor 0: start again (printed, to follow a decoder
            // that does not complete its search)
            const uint32_t known = count > 1 ? uid & ~((1UL << (33 - count)) - 1) : 0;
            Serial.printf("Discovery: search stopped at bit %u (UID so far %08lX, %u bits)\n", count,
                          (unsigned long)known, count - 1);
            return false;
        }
    }
    *found = uid;
    return true;
}

/*
 The noise of a running motor spoils the data feedback (a third of the
 reads failed on a 150 Y at speed): the configuration is read only while
 the locomotive stands still. If it starts during the reading, the reading
 stops and is done again later.
*/
static volatile bool sMoving = false;

static bool readFromTrack(void *ctx, uint16_t cv, uint8_t index, uint8_t bytes, uint8_t *out)
{
    const uint16_t sid = (uint16_t)(uintptr_t)ctx;
    if (sMoving)
        return false;
    Loco *l = Loco::findLoco(sid);
    if (!Centrale::power() || (l != nullptr && l->speed() != 0))
    {
        sMoving = true; // running, or the track was switched off: later
        return false;
    }
    // Read 2 bytes at a time, as the MS2 does. With the 040 DE 9 (ESU) about
    // half of the first tries stay silent, 2 bytes as well as 4 (October
    // 2026): the cause is elsewhere; the 150 X 5 (ESU) answered 60 reads
    // out of 60 this way.
    if (bytes == 4)
        return Feedback::readCV(sid, cv, index, 2, out) && Feedback::readCV(sid, cv, index + 2, 2, out + 2);
    return Feedback::readCV(sid, cv, index, bytes, out);
}

// Name of the locomotive, printable (UTF-8 kept, control bytes replaced)
static void printable(const char *in, char *out, size_t size)
{
    size_t i = 0;
    for (; in[i] && i + 1 < size; i++)
        out[i] = ((uint8_t)in[i] < 0x20 || in[i] == 0x7F) ? '?' : in[i];
    out[i] = 0;
}

bool Discovery::readConfig(uint16_t sid, char *nameOut, size_t size)
{
    static MfxConfig c; // static: keeps the task stack small (one task only)
    const uint32_t t0 = millis();
    const uint32_t tries0 = Feedback::readTries, retried0 = Feedback::readRetried;
    sMoving = false;
    // Nothing else is printed until the end: about one second per CV
    // block, a few minutes for a decoder with many functions
    Serial.printf("Discovery: reading the configuration of address %u (a few minutes)...\n", sid);
    const MfxConfig::Status st = c.read(readFromTrack, (void *)(uintptr_t)sid);
    if (sMoving)
    {
        Serial.printf("Discovery: address %u is running (or the track is off), configuration read later\n", sid);
        return false;
    }
    if (st == MfxConfig::NO_ANSWER)
        return false;

    char name[17];
    printable(c.name, name, sizeof(name));

    // Kept with the locomotive, for the loco list sent to Rocrail
    MfxInfo info;
    memset(&info, 0, sizeof(info));
    info.valid = true;
    info.complete = (st == MfxConfig::OK && c.failures == 0);
    info.av = c.acceleration;
    info.bv = c.braking;
    info.vmin = c.vmin;
    info.vmax = c.vmax;
    info.tachoMax = c.tachoKmh;
    info.functions = c.functionCount;
    for (uint8_t f = 0; f < c.functionCount && f < 16; f++)
    {
        const MfxConfig::Function &fn = c.functions[f];
        info.fnType[f] = fn.present ? Cs2Config::cs2FunctionType(f, fn.momentary, fn.group, fn.s1, fn.s2) : 0;
    }
    if (Loco *l = Loco::findLoco(sid))
    {
        l->mfxInfo(info);
        // A decoder found without its name ("mfx <UID>"): the name read now
        if (name[0] && l->name().startsWith("mfx "))
            l->name(name);
        // Kept in the NVS with the locomotive: not read again at the next
        // start-up if every read succeeded
        LocoStore::save();
        // No bind announced to Rocrail for now: once a configuration was read
        // completely, the announce made the locomotives stop answering speed
        // and function commands (October 2026, found by testing the commits
        // one by one). To be looked at with a capture of what Rocrail does.
    }
    if (nameOut && size)
    {
        strncpy(nameOut, name, size - 1);
        nameOut[size - 1] = 0;
    }

    Serial.printf("Decoder at address %u: \"%s\"", sid, name);
    if (c.haveManufacturer)
        Serial.printf(", manufacturer %02lX%s, product %08lX", (unsigned long)c.manufacturer,
                      c.manufacturer == 0x83 ? " (Marklin/Trix)" : (c.manufacturer == 0x97 ? " (ESU)" : ""),
                      (unsigned long)c.product);
    Serial.println();
    if (st == MfxConfig::NO_BLOCK_TABLE)
        Serial.println("  no block table at CV 004: structure not read");

    Serial.print("  blocks:");
    for (uint8_t i = 0; i < c.blockCount; i++)
        Serial.printf(" %02X@%03X", c.blocks[i].type, c.blocks[i].cv);
    Serial.println();

    for (uint8_t f = 0; f < c.functionCount; f++)
    {
        const MfxConfig::Function &fn = c.functions[f];
        if (!fn.present)
            continue;
        Serial.printf("  F%-2u %-20s symbol %02X %02X %02X%s\n", f, MfxConfig::groupName(fn.group),
                      fn.group, fn.s1, fn.s2, fn.momentary ? ", momentary" : "");
    }
    if (c.haveDriving)
        Serial.printf("  acceleration %.2f s, braking %.2f s, Vmin %u, Vmax %u, tachometer %u km/h\n",
                      c.acceleration * 0.25f, c.braking * 0.25f, c.vmin, c.vmax, c.tachoKmh);
    Serial.printf("  block 01 (%u CAs), types:", c.block01Size);
    for (uint8_t k = 0; k < c.block01Size && k < sizeof(c.block01Types); k++)
        if (c.block01Types[k] == 0xFF)
            Serial.print(" --");
        else
            Serial.printf(" %02X", c.block01Types[k]);
    Serial.println();
    if (const MfxConfig::Block *f = c.block(0x04))
        Serial.printf("  block 04: %u groups of %u CAs, %u functions read\n", f->groups, f->perGroup,
                      c.functionCount);
    Serial.printf("  %u reads, %u failed, %.1f s; on the track %lu tries, %lu reads needed more than one",
                  c.reads, c.failures, (millis() - t0) / 1000.0f, (unsigned long)(Feedback::readTries - tries0),
                  (unsigned long)(Feedback::readRetried - retried0));
    for (uint8_t i = 0; i < c.failures && i < MfxConfig::MAX_FAILED; i++)
        Serial.printf("%s %03X/%02X", i ? "," : " (failed:", c.failedCv[i], c.failedIndex[i]);
    Serial.println(c.failures ? ")" : "");
    return true;
}

/*
 Test of the reading of CVs: the configuration of each locomotive declared
 in main.cpp. Only the decoders registered with this command station answer.
 A decoder that does not answer is tried again later (READ_TRIES in all):
 it may be off the track, or still starting.
 Returns true when nothing is left to try.
*/
static const uint8_t READ_TRIES = 3;
static std::unordered_map<uint16_t, uint8_t> sReadTries; // tries per address, READ_TRIES once read
static volatile bool sRereadConfigs = false; // serial command C

bool Discovery::readDeclaredConfigs()
{
    bool left = false;
    uint16_t list[32];
    const uint8_t count = knownAddresses(list, 32);
    for (uint8_t i = 0; i < count; i++)
    {
        const uint16_t a = list[i];
        Loco *l = Loco::findLoco(a);
        if (l == nullptr || sReadTries[a] >= READ_TRIES)
            continue;
        if (l->mfxInfo().complete)
        {
            // Read completely before (kept in the NVS): not read again
            sReadTries[a] = READ_TRIES;
            continue;
        }
        if (!Centrale::power())
            return false; // the track was switched off: later
        if (l->speed() != 0)
        {
            left = true; // running: later, without counting a try
            continue;
        }
        sReadTries[a]++;
        if (readConfig(a))
            sReadTries[a] = READ_TRIES; // done
        else if (sMoving)
        {
            sReadTries[a]--; // started during the reading: not a try
            left = true;
        }
        else
        {
            // Printed once, after the last try: absent locomotives would
            // otherwise give a line every RETRY_MS
            if (sReadTries[a] < READ_TRIES)
                left = true;
            else
                Serial.printf("Discovery: address %u (%s): no answer after %u tries, configuration not read\n",
                              a, l->name().c_str(), READ_TRIES);
        }
    }
    return !left;
}

/*
 As the Gleisbox (recorded, October 2026): each bind is sent 4 times, about
 43 ms apart. Sent once, a bind was missed by the 150 Y 17 (Marklin), which
 then kept answering the search and never got its address.
*/
static const uint8_t BIND_REPEAT = 4;
static const uint32_t BIND_SPACING_MS = 43;

static void sendBind(Loco *l, bool verify = true)
{
    MfxFrame f;
    f.broadcast();
    f.bind(l->addr(), l->uid32());
    f.finish();
    uint8_t buff[BUFFER_SIZE];
    if (!f.toBuffer(buff, sizeof(buff)))
        return;
    // The queue is often full (refresh of every locomotive): waiting is
    // needed, a bind dropped with a short timeout never reached the track
    uint8_t lost = 0;
    for (uint8_t i = 0; i < BIND_REPEAT; i++)
    {
        if (xQueueSendToFront(mfxQueue, buff, pdMS_TO_TICKS(200)) != pdPASS)
            lost++;
        vTaskDelay(pdMS_TO_TICKS(BIND_SPACING_MS));
    }
    if (lost)
        Serial.printf("Discovery: %u bind(s) of address %u not sent (queue full)\n", lost, l->addr());

    if (!verify)
        return; // the master sends its own verify (CAN 0x03)

    // As the MS2: the bind is followed by an existence query at the new
    // address (C1: bind, then verify, which the decoder answers)
    MfxFrame v;
    v.address(l->addr());
    v.verify(l->uid32());
    v.finish();
    Feedback::Result r;
    const bool yes = Feedback::ask(v, &r);
    Serial.printf("Discovery: bind of %s (UID %08lX) at address %u %s (verify: %u regular intervals)\n",
                  l->name().c_str(), (unsigned long)l->uid32(), l->addr(), yes ? "confirmed" : "NOT confirmed",
                  r.windowRegular);
    if (yes)
        vTaskDelay(pdMS_TO_TICKS(ANSWER_PAUSE_MS));
}


/*
 Commands typed in the serial monitor (one letter, then Enter):
   R : re-registration of every decoder. The re-registration counter
       (Krauss 3.2.6) is increased and kept in NVS, then the command station
       restarts: the decoders see a new counter when the track comes back
       and register again. Declared locomotives get their address back; the
       others get a free address.
   V : existence query (ping, Krauss 3.2.5) of every declared locomotive,
       with the counts of the 1-bit feedback: does its decoder answer, and
       how strongly?
   C : read the configurations of the locomotives again (otherwise a
       configuration read completely is kept in the NVS and not read again).
   L : list of the locomotives (address, UID, name), with those in memory.
   E : erase the locomotives in memory (LocoStore); the counter is raised
       and the command station restarts, so that every decoder registers
       again.
   ? : this help.
*/
static void verifyAll()
{
    uint16_t list[32];
    const uint8_t count = knownAddresses(list, 32);
    for (uint8_t i = 0; i < count; i++)
    {
        const uint16_t a = list[i];
        Loco *l = Loco::findLoco(a);
        if (l == nullptr || l->uid32() == 0)
            continue;
        MfxFrame f;
        f.address(a);
        f.verify(l->uid32());
        f.finish();
        Feedback::Result r;
        const bool yes = Feedback::ask(f, &r);
        Serial.printf("Verify address %u (%s, UID %08lX): %s, %u regular intervals in the window, %u in the reference\n",
                      a, l->name().c_str(), (unsigned long)l->uid32(), yes ? "ANSWER" : "no answer",
                      r.windowRegular, r.referenceRegular);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Presence of each locomotive (verifyNext())
struct Presence
{
    bool absent;
    uint8_t misses;
    TickType_t since;
};
static std::unordered_map<uint16_t, Presence> sPresence; // by address

/*
 Deletes a locomotive, as the MS2 does (1 Nov, Gleisbox capture: "addr 11 |
 verify UID 00000000", twice): an existence query at its address with the
 UID 0 makes its decoder forget its address at once (Krauss 4.3). The
 command station forgets it too. A decoder on the track then answers the
 search again and is registered as a new locomotive (address, configuration,
 name), exactly as on an MS2.
*/
/*
 Decoders to make forget their address. A decoder keeps its address while
 it hears this command station (UID and counter), and then no longer
 answers the search: if the command station forgets it but the decoder did
 not hear the order to forget (track off, locomotive off the track), it
 stays invisible for good (241 A, 2-3 Nov 2026). So each deletion is kept
 here (NVS, namespace "locos", key "forget"): every FORGET_CHECK_MS the
 command station asks each address whether its decoder is still there with
 that UID, and if so sends the order to forget again. The entry goes when
 the decoder is found by the search (it forgot) or bound again by the
 master. An entry without a UID (D on an unknown address, track off) is
 sent once, as soon as the track is on.
*/
struct ForgetEntry
{
    uint32_t uid; // 0: unknown, sent once
    uint16_t addr;
};
static const uint8_t MAX_FORGET = 16;
static const uint32_t FORGET_CHECK_MS = 30000;
static ForgetEntry sForget[MAX_FORGET];
static uint8_t sForgetCount = 0;
static bool sForgetLoaded = false;

static void forgetSave()
{
    Preferences prefs;
    if (!prefs.begin("locos", false))
        return;
    if (sForgetCount)
        prefs.putBytes("forget", sForget, sForgetCount * sizeof(ForgetEntry));
    else if (prefs.isKey("forget"))
        prefs.remove("forget");
    prefs.end();
}

static void forgetLoad()
{
    if (sForgetLoaded)
        return;
    sForgetLoaded = true;
    Preferences prefs;
    if (!prefs.begin("locos", false))
        return;
    if (prefs.isKey("forget"))
    {
        const size_t len = prefs.getBytesLength("forget");
        if (len % sizeof(ForgetEntry) == 0 && len <= sizeof(sForget))
        {
            prefs.getBytes("forget", sForget, len);
            sForgetCount = len / sizeof(ForgetEntry);
        }
    }
    prefs.end();
    if (sForgetCount)
        Serial.printf("Discovery: %u deleted decoder(s) to check (they may still have their address)\n",
                      sForgetCount);
}

static void forgetRemoveAt(uint8_t i)
{
    for (uint8_t j = i + 1; j < sForgetCount; j++)
        sForget[j - 1] = sForget[j];
    sForgetCount--;
}

// The decoder of this UID forgot its address (found by the search) or got
// a new one from the master: nothing more to do for it
static void forgetDrop(uint32_t uid)
{
    forgetLoad();
    bool changed = false;
    for (uint8_t i = 0; i < sForgetCount;)
        if (uid != 0 && sForget[i].uid == uid)
        {
            forgetRemoveAt(i);
            changed = true;
        }
        else
            i++;
    if (changed)
        forgetSave();
}

static void forgetAdd(uint32_t uid, uint16_t addr)
{
    forgetLoad();
    for (uint8_t i = 0; i < sForgetCount;)
        if ((uid != 0 && sForget[i].uid == uid) || sForget[i].addr == addr)
            forgetRemoveAt(i);
        else
            i++;
    if (sForgetCount == MAX_FORGET)
        forgetRemoveAt(0); // the oldest
    sForget[sForgetCount++] = {uid, addr};
    forgetSave();
}

// Verify with the UID 0: the decoder with this address forgets it
static void sendForget(uint16_t a)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        MfxFrame f;
        f.address(a);
        f.verify(0);
        f.finish();
        Feedback::ask(f);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void forgetCheck()
{
    static uint32_t last = 0;
    forgetLoad();
    if (sForgetCount == 0 || !Centrale::power() || Centrale::poweredForMs() < POWERED_MS)
        return;
    if (last != 0 && millis() - last < FORGET_CHECK_MS)
        return;
    last = millis();
    bool changed = false;
    for (uint8_t i = 0; i < sForgetCount;)
    {
        const ForgetEntry e = sForget[i];
        if (e.uid == 0)
        {
            sendForget(e.addr);
            Serial.printf("Discovery: address %u sent to the decoders (UID 0), the track being on now\n", e.addr);
            forgetRemoveAt(i);
            changed = true;
            continue;
        }
        if (Loco *l = Loco::findLoco(e.addr))
            if (l->uid32() == e.uid)
            {
                // Registered here again at the same address: nothing to do
                forgetRemoveAt(i);
                changed = true;
                continue;
            }
        MfxFrame f;
        f.address(e.addr);
        f.verify(e.uid);
        f.finish();
        Feedback::Result r = {};
        if (Feedback::ask(f, &r))
        {
            sendForget(e.addr);
            Serial.printf("Discovery: deleted decoder %08lX still had the address %u: told again to forget it\n",
                          (unsigned long)e.uid, e.addr);
        }
        i++;
    }
    if (changed)
        forgetSave();
}

static Loco *locoAt(uint32_t uid, uint16_t sid);

static void deleteLoco(uint16_t a)
{
    Loco *l = Loco::findLoco(a);
    if (l == nullptr)
    {
        // Unknown here (bound by the master in another session, or forgotten
        // while the track was off): its decoder may still have the address
        if (!Centrale::power())
        {
            forgetAdd(0, a);
            Serial.printf("\nDelete: address %u unknown, and the track is off: sent as soon as the track is on\n", a);
            return;
        }
        sendForget(a);
        Serial.printf("\nDelete: address %u sent to the decoders (UID 0): a decoder with this address registers again\n",
                      a);
        return;
    }
    const String name = l->name();
    const uint32_t uid = l->uid32();
    if (Centrale::power())
        sendForget(a);
    forgetAdd(uid, a); // checked again later: the decoder may not have heard it
    Message::removeLoco(a);
    Loco::removeLoco(a);
    sPresence.erase(a);
    sReadTries.erase(a);
    LocoStore::save();
    Serial.printf("\nDeleted: %s (UID %08lX) at address %u%s. Delete it in Rocrail too.\n", name.c_str(),
                  (unsigned long)uid, a,
                  Centrale::power() ? "" : " (track off: its decoder is told to forget it once the track is on)");
}

/*
 Test bench for the feedback (serial commands M and N): the same requests
 as in service, repeated, with the counts of each window, to measure what
 the noise of other locomotives (motors, sounds) does to the answers of
 this one. Run it with the locomotive on the track (no answer must be
 missed) and taken off (no answer must be seen).
*/
static void printStats(const char *what, uint16_t *v, uint16_t n)
{
    // sorted for the median (insertion sort, n <= 500)
    for (uint16_t i = 1; i < n; i++)
    {
        const uint16_t x = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > x)
        {
            v[j + 1] = v[j];
            j--;
        }
        v[j + 1] = x;
    }
    Serial.printf("  %-22s min %3u  median %3u  max %3u\n", what, v[0], v[n / 2], v[n - 1]);
}

static void benchAsk(const MfxFrame &f, const char *what, uint16_t n, uint32_t pauseMs = ANSWER_PAUSE_MS)
{
    if (n > 500)
        n = 500;
    uint16_t *col[6];
    for (uint8_t k = 0; k < 6; k++)
        col[k] = (uint16_t *)malloc(n * sizeof(uint16_t));
    uint16_t answers = 0, weak = 0;
    uint16_t ends[3] = {0, 0, 0}, endAnswers[3] = {0, 0, 0}; // per end of the request
    static const char *const END_NAMES[3] = {"none ", "flip ", "pulse"};
    Serial.printf("\nBench: %u %s, %lu ms between two questions, pulse %u us%s\n", n, what,
                  (unsigned long)pauseMs + 30, MfxSignal::pulseUs, MfxSignal::answerPulse ? "" : " (off)");
    Serial.println("  #   window edges/regular/run   reference edges/regular/run   end    decision");
    for (uint16_t i = 0; i < n; i++)
    {
        Feedback::Result r = {};
        const bool yes = Feedback::ask(f, &r);
        const bool w = !yes && weakAnswer(r);
        answers += yes;
        weak += w;
        if (col[0])
        {
            col[0][i] = r.window < 0 ? 0 : r.window;
            col[1][i] = r.windowRegular;
            col[2][i] = r.windowRun;
            col[3][i] = r.reference < 0 ? 0 : r.reference;
            col[4][i] = r.referenceRegular;
            col[5][i] = r.referenceRun;
        }
        const uint8_t end = MfxSignal::lastEnd < 3 ? MfxSignal::lastEnd : 0;
        ends[end]++;
        endAnswers[end] += yes;
        Serial.printf("  %3u   %4d / %3u / %3u           %4d / %3u / %3u            %s  %s\n", i + 1, r.window,
                      r.windowRegular, r.windowRun, r.reference, r.referenceRegular, r.referenceRun,
                      END_NAMES[end], yes ? "answer" : w ? "weak (counted as present)" : "no answer");
        vTaskDelay(pdMS_TO_TICKS(pauseMs));
    }
    Serial.printf("Bench: %u answers, %u weak, %u no answer, out of %u\n", answers, weak, n - answers - weak, n);
    for (uint8_t e = 0; e < 3; e++)
        if (ends[e])
            Serial.printf("  end %s: %u answers out of %u\n", END_NAMES[e], endAnswers[e], ends[e]);
    if (col[0] && col[5])
    {
        printStats("window edges", col[0], n);
        printStats("window regular", col[1], n);
        printStats("window longest run", col[2], n);
        printStats("reference edges", col[3], n);
        printStats("reference regular", col[4], n);
        printStats("reference longest run", col[5], n);
    }
    for (uint8_t k = 0; k < 6; k++)
        free(col[k]);
}

/*
 Test scenario (serial command X): verifies of one locomotive, the periodic
 refresh of the locomotives on and off in turn, rounds x per questions for
 each case. Alternating keeps a slow drift (heating, position on the track)
 from favouring one case. One line per block, a summary at the end; the
 refresh is left as it was.
*/
static void scenarioRefresh(uint16_t a, uint16_t rounds, uint16_t per)
{
    Loco *l = Loco::findLoco(a);
    MfxFrame f;
    f.address(a);
    f.verify(l->uid32());
    f.finish();
    const bool wasQuiet = Message::quiet;
    uint32_t got[2] = {0, 0}, asked[2] = {0, 0};
    uint16_t worst[2] = {0xFFFF, 0xFFFF};
    const uint32_t t0 = millis();
    Serial.printf("\nScenario: %s (address %u), %u rounds of %u verifies with the refresh on, then off\n",
                  l->name().c_str(), a, rounds, per);
    for (uint16_t r = 1; r <= rounds && Centrale::power(); r++)
        for (uint8_t q = 0; q < 2; q++)
        {
            Message::quiet = (q == 1);
            vTaskDelay(pdMS_TO_TICKS(300)); // the queue empties
            uint16_t ok = 0;
            for (uint16_t i = 0; i < per; i++)
            {
                if (Feedback::ask(f))
                    ok++;
                vTaskDelay(pdMS_TO_TICKS(ANSWER_PAUSE_MS));
            }
            got[q] += ok;
            asked[q] += per;
            if (ok < worst[q])
                worst[q] = ok;
            Serial.printf("Scenario: round %2u, refresh %s: %2u/%u\n", r, q ? "off" : "on ", ok, per);
        }
    Message::quiet = wasQuiet;
    Serial.printf("Scenario: summary, %.0f s\n", (millis() - t0) / 1000.0f);
    for (uint8_t q = 0; q < 2; q++)
        if (asked[q])
            Serial.printf("  refresh %s: %lu/%lu answers (%.1f %% silent), worst round %u/%u\n", q ? "off" : "on ",
                          (unsigned long)got[q], (unsigned long)asked[q],
                          100.0f * (asked[q] - got[q]) / asked[q], worst[q], per);
}

static void benchVerify(uint16_t a, uint16_t n)
{
    Loco *l = Loco::findLoco(a);
    MfxFrame f;
    f.address(a);
    f.verify(l->uid32());
    f.finish();
    char what[64];
    snprintf(what, sizeof(what), "verifies of %s (address %u)", l->name().c_str(), a);
    benchAsk(f, what, n);
}

// Search question with the whole UID (32 bits): only the decoder with this
// UID may answer, if it has no address yet (the 040 DE, for example)
static void benchSearch(uint32_t uid, uint16_t n, uint32_t pauseMs)
{
    MfxFrame f;
    f.broadcast();
    f.search(32, uid);
    f.finish();
    char what[64];
    snprintf(what, sizeof(what), "search questions for the UID %08lX", (unsigned long)uid);
    benchAsk(f, what, n, pauseMs);
}

/*
 Search questions for every count from 0 to 32, with the UID given (exact)
 or with its first `count` bits followed by ones (as the search asks them):
 up to TRIES tries each, MISS_PAUSE_MS apart. Shows which counts the
 decoder answers (the 040 DE stops at count 8, 2 Nov).
*/
static void benchSweep(uint32_t uid, bool ones)
{
    Serial.printf("\nBench: search questions for the UID %08lX, count 0 to 32, %s\n", (unsigned long)uid,
                  ones ? "the other bits at 1 (as the search)" : "whole UID");
    uint8_t answered = 0;
    for (uint8_t count = 0; count <= 32; count++)
    {
        uint32_t q = uid;
        if (ones && count < 32)
            q |= (count == 0) ? 0xFFFFFFFFUL : ((1UL << (32 - count)) - 1);
        MfxFrame f;
        f.broadcast();
        f.search(count, q);
        f.finish();
        uint8_t got = 0;
        Feedback::Result r = {};
        for (uint8_t t = 1; t <= TRIES && !got; t++)
        {
            if (Feedback::ask(f, &r))
                got = t;
            else if (t < TRIES)
                vTaskDelay(pdMS_TO_TICKS(MISS_PAUSE_MS));
        }
        answered += got != 0;
        if (got)
            Serial.printf("  count %2u  question %08lX  answer at try %u (%u regular, run %u)\n", count,
                          (unsigned long)q, got, r.windowRegular, r.windowRun);
        else
            Serial.printf("  count %2u  question %08lX  NO ANSWER in %u tries\n", count, (unsigned long)q, TRIES);
        vTaskDelay(pdMS_TO_TICKS(ANSWER_PAUSE_MS));
    }
    Serial.printf("Bench: %u counts answered out of 33\n", answered);
}

static void benchRead(uint16_t a, uint16_t n)
{
    Loco *l = Loco::findLoco(a);
    if (n > 100)
        n = 100;
    uint16_t ok = 0;
    Serial.printf("\nBench: %u reads of the name of %s (address %u)\n", n, l->name().c_str(), a);
    for (uint16_t i = 0; i < n; i++)
    {
        char name[17];
        memset(name, 0, sizeof(name));
        bool good = true;
        for (uint8_t k = 0; k < 16 && good; k += 2)
        {
            uint8_t v[2] = {0, 0};
            good = Feedback::readCV(a, 3, 1 + k, 2, v);
            name[k] = (char)v[0];
            name[k + 1] = (char)v[1];
            vTaskDelay(pdMS_TO_TICKS(Feedback::READ_PAUSE_MS));
        }
        for (uint8_t k = 0; k < 16; k++)
            if (name[k] != 0 && (name[k] < 0x20 || name[k] >= 0x7F))
                name[k] = '?';
        ok += good;
        Serial.printf("  %3u  %s  \"%s\"\n", i + 1, good ? "ok    " : "failed", good ? name : "");
    }
    Serial.printf("Bench: %u reads ok out of %u\n", ok, n);
}

static void console()
{
    while (Serial.available() > 0)
    {
        const int c = Serial.read();
        if (c == 'R')
        {
            const uint16_t next = Centrale::counter() + 1;
            Centrale::setCounter(next);
            Serial.printf("\nRe-registration of every decoder: counter %u, restart...\n", next);
            Serial.flush();
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP.restart();
        }
        else if (c == 'V')
        {
            if (Centrale::power())
                verifyAll();
            else
                Serial.println("Verify: the track is off");
        }
        else if (c == 'L')
            LocoStore::print();
        else if (c == 'C')
        {
            // Read the configurations again (a decoder changed, or to test
            // the reading): forgotten, then read as at start-up.
            // C<address>: this locomotive only (the others are kept)
            vTaskDelay(pdMS_TO_TICKS(50)); // the rest of the line
            uint16_t only = 0;
            while (Serial.available() > 0 && isdigit(Serial.peek()))
                only = only * 10 + (Serial.read() - '0');
            if (only != 0 && Loco::findLoco(only) == nullptr)
            {
                Serial.printf("\nC%u: no locomotive at this address (L lists them)\n", only);
                continue;
            }
            uint16_t list[32];
            const uint8_t count = knownAddresses(list, 32);
            for (uint8_t i = 0; i < count; i++)
                if (Loco *l = Loco::findLoco(list[i]))
                {
                    const uint16_t a = list[i];
                    if (only != 0 && a != only)
                    {
                        sReadTries[a] = READ_TRIES; // kept, not read
                        continue;
                    }
                    MfxInfo none;
                    memset(&none, 0, sizeof(none));
                    l->mfxInfo(none);
                    sReadTries[a] = 0;
                }
            LocoStore::save();
            sRereadConfigs = true;
            if (only)
                Serial.printf("\nConfiguration of address %u forgotten: read again\n", only);
            else
                Serial.println("\nConfigurations forgotten: read again");
        }
        else if (c == 'E')
        {
            // Forgotten by the command station: the decoders must forget it
            // too, so the counter is raised (they register again)
            LocoStore::erase();
            const uint16_t next = Centrale::counter() + 1;
            Centrale::setCounter(next);
            Serial.printf("\nLocomotives erased from memory, counter %u, restart...\n", next);
            Serial.flush();
            vTaskDelay(pdMS_TO_TICKS(500));
            ESP.restart();
        }
        else if (c == 'D')
        {
            // D followed by the address, D4 for example
            vTaskDelay(pdMS_TO_TICKS(50)); // the rest of the line
            uint16_t a = 0;
            bool digits = false;
            while (Serial.available() > 0 && isdigit(Serial.peek()))
            {
                a = a * 10 + (Serial.read() - '0');
                digits = true;
            }
            if (!digits || a < 1 || a > 16383)
                Serial.println("\nDelete: D followed by the address (1 to 16383), D4 for example");
            else
                deleteLoco(a);
        }
        else if (c == 'M' || c == 'N')
        {
            // Test bench: M<address>[,count] = count verifies (50), N<address>[,count]
            // = count reads of the name (10); statistics of the feedback
            vTaskDelay(pdMS_TO_TICKS(50)); // the rest of the line
            uint16_t a = 0, n = 0;
            bool digits = false;
            while (Serial.available() > 0 && isdigit(Serial.peek()))
            {
                a = a * 10 + (Serial.read() - '0');
                digits = true;
            }
            if (Serial.available() > 0 && Serial.peek() == ',')
            {
                Serial.read();
                while (Serial.available() > 0 && isdigit(Serial.peek()))
                    n = n * 10 + (Serial.read() - '0');
            }
            if (!digits || Loco::findLoco(a) == nullptr)
                Serial.printf("\n%c: %c followed by the address of a known locomotive (L lists them), %c404 or %c404,100\n",
                              c, c, c, c);
            else if (!Centrale::power())
                Serial.println("\nTest bench: the track is off");
            else if (c == 'M')
                benchVerify(a, n ? n : 50);
            else
                benchRead(a, n ? n : 10);
        }
        else if (c == 'X')
        {
            // Test scenario: X<address>[,rounds[,per round]], X2575 = 10 rounds
            // of 20 verifies with the refresh on, 20 with it off
            vTaskDelay(pdMS_TO_TICKS(50));
            uint16_t a = 0, r = 0, n = 0;
            while (Serial.available() > 0 && isdigit(Serial.peek()))
                a = a * 10 + (Serial.read() - '0');
            if (Serial.available() > 0 && Serial.peek() == ',')
            {
                Serial.read();
                while (Serial.available() > 0 && isdigit(Serial.peek()))
                    r = r * 10 + (Serial.read() - '0');
            }
            if (Serial.available() > 0 && Serial.peek() == ',')
            {
                Serial.read();
                while (Serial.available() > 0 && isdigit(Serial.peek()))
                    n = n * 10 + (Serial.read() - '0');
            }
            if (Loco::findLoco(a) == nullptr)
                Serial.println("\nX: X<address of a known locomotive>[,rounds[,verifies per round]], X2575 or X2575,10,20");
            else if (!Centrale::power())
                Serial.println("\nX: the track is off");
            else
                scenarioRefresh(a, r ? r : 10, n ? n : 20);
        }
        else if (c == 'Q')
        {
            Message::quiet = !Message::quiet;
            Serial.printf("\nPeriodic refresh of the locomotives: %s\n",
                          Message::quiet ? "STOPPED (test, Q to restart)" : "running");
        }
        else if (c == 'B')
        {
            // Test: B<address>,<new address> = the locomotive gets another
            // address (bind sent to its decoder), to compare the answers
            // with a short address (7 bits, below 128) and a long one.
            // The master is not told: drive it again at its old address
            // only after B<new>,<old>.
            vTaskDelay(pdMS_TO_TICKS(50));
            uint16_t a = 0, n = 0;
            while (Serial.available() > 0 && isdigit(Serial.peek()))
                a = a * 10 + (Serial.read() - '0');
            if (Serial.available() > 0 && Serial.peek() == ',')
            {
                Serial.read();
                while (Serial.available() > 0 && isdigit(Serial.peek()))
                    n = n * 10 + (Serial.read() - '0');
            }
            Loco *l = Loco::findLoco(a);
            if (l == nullptr || n == 0 || n > 16383 || Loco::findLoco(n) != nullptr)
                Serial.println("\nB: B<address of a known locomotive>,<free new address 1..16383>, B2575,10 for example");
            else if (!Centrale::power())
                Serial.println("\nB: the track is off");
            else
            {
                const uint32_t uid = l->uid32();
                Loco *m = locoAt(uid, n);
                sendBind(m);
                LocoStore::save();
                Serial.printf("\nTest: %08lX moved from address %u to %u (Rocrail is not told; B%u,%u to go back)\n",
                              (unsigned long)uid, a, n, n, a);
            }
        }
        else if (c == 'S' || c == 'T')
        {
            // S<UID in hex>[,count]: search questions with the whole UID (50).
            // The whole line is read (up to the end of line, 1 s at most):
            // the letters of the UID must not be taken for commands (E!)
            char line[24];
            uint8_t len = 0;
            const uint32_t t0 = millis();
            while (millis() - t0 < 1000)
            {
                if (Serial.available() == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(5));
                    continue;
                }
                const int ch = Serial.read();
                if (ch == '\n' || ch == '\r')
                    break;
                if (len < sizeof(line) - 1)
                    line[len++] = (char)ch;
            }
            line[len] = 0;
            uint32_t uid = 0;
            uint16_t n = 0;
            uint8_t digits = 0;
            const char *q = line;
            for (; isxdigit((unsigned char)*q); q++, digits++)
                uid = uid << 4 | (uint32_t)(isdigit((unsigned char)*q) ? *q - '0' : (toupper((unsigned char)*q) - 'A' + 10));
            uint32_t pause = ANSWER_PAUSE_MS;
            if (*q == ',')
                for (q++; isdigit((unsigned char)*q); q++)
                    n = n * 10 + (*q - '0');
            if (*q == ',')
            {
                pause = 0;
                for (q++; isdigit((unsigned char)*q); q++)
                    pause = pause * 10 + (*q - '0');
            }
            if (digits != 8)
                Serial.println("\nS: S followed by the 8 hexadecimal digits of the UID, SF9E610E0 or SF9E610E0,100 or SF9E610E0,50,300 (pause in ms);"
                               " T<UID> = every count, whole UID; T<UID>,1 = every count, the other bits at 1");
            else if (!Centrale::power())
                Serial.println("\nTest bench: the track is off");
            else if (c == 'T')
                benchSweep(uid, n != 0);
            else
                benchSearch(uid, n ? n : 50, pause);
        }
        else if (c == 'A')
        {
            MfxSignal::answerPulse = !MfxSignal::answerPulse;
            Serial.printf("\nPulse after the bits 0011 when they end at the window level: %s\n",
                          MfxSignal::answerPulse ? "on (as the Gleisbox, default)" : "off");
        }
        else if (c == 'I')
        {
            // Test: I<µs> = length of the pulse after 0011 (I10 = default, about 2 on the Gleisbox)
            vTaskDelay(pdMS_TO_TICKS(50));
            uint16_t us = 0;
            bool digits = false;
            while (Serial.available() > 0 && isdigit(Serial.peek()))
            {
                us = us * 10 + (Serial.read() - '0');
                digits = true;
            }
            if (!digits || us == 0 || us > 100)
                Serial.printf("\nI: I followed by the pulse length in µs, 1 to 100 (I10 = default), now %u\n",
                              MfxSignal::pulseUs);
            else
            {
                MfxSignal::pulseUs = us;
                Serial.printf("\nPulse after the bits 0011: %u µs\n", us);
            }
        }
        else if (c == 'F')
        {
            // Test: F<µs> = flip before the answer window after this time
            // (2 = as the Gleisbox was measured), F0 = no flip
            vTaskDelay(pdMS_TO_TICKS(50));
            uint16_t us = 0;
            bool digits = false;
            while (Serial.available() > 0 && isdigit(Serial.peek()))
            {
                us = us * 10 + (Serial.read() - '0');
                digits = true;
            }
            if (!digits)
                Serial.printf("\nF: F followed by the time in µs (F2 = default, F0 = no flip), now %u\n",
                              MfxSignal::answerFlipUs);
            else
            {
                MfxSignal::answerFlipUs = us;
                Serial.printf("\nFlip before the answer window: %s %u µs\n", us ? "after" : "none", us);
            }
        }
        else if (c == '?')
            Serial.printf("\nCommands: R = re-registration of every decoder (counter %u -> %u, restart), "
                          "V = verify every declared locomotive, L = list the locomotives, C = read the configurations again (C404: one locomotive), "
                          "D<address> = delete one locomotive (D4), M<address>[,n] = test bench, n verifies (M404,50), "
                          "N<address>[,n] = test bench, n reads of the name (N404,10), "
                          "S<UID>[,n[,pause]] = test bench, n search questions with the whole UID, pause in ms after each (SF9E610E0,50,150), "
                          "T<UID>[,1] = test bench, search questions for every count (,1: other bits at 1), "
                          "F<us> = flip before the answer window (F2 default, F0 none), A = pulse after 0011 when already at the window level, as the Gleisbox (on/off), I<us> = length of this pulse (I10 default), B<addr>,<new> = move a locomotive to another address (test), Q = stop/restart the refresh of the locomotives (test), X<addr>[,rounds,per] = scenario: verifies with the refresh on and off in turn, E = erase the locomotives in memory (counter + 1, restart), ? = help\n",
                          Centrale::counter(), Centrale::counter() + 1);
    }
}

/*
 Existence query (Krauss 3.2.5, 4.1 and 4.3), one locomotive at a time, in
 turn with the search, as the MS2 does ("search, query SID 1, search, query
 SID 2..."). A decoder that receives no query for its own address during 5
 to 10 minutes is ready to register again: without them our decoders lost
 their registration.

 A locomotive that does not answer ABSENT_MISSES times in a row is not on
 the track (one answer may be missed, or weak when the decoder has just
 started: 64 edges instead of 300 for the 241 A, 1 Nov): it gets no bind
 (if it comes back without its address it answers the search, and gets its
 bind there) and is queried again only every ABSENT_RETRY_MS, so that the
 locomotives in a box do not take the track time. Its arrival and its
 departure are written once in the log.
*/
static const uint32_t ABSENT_RETRY_MS = 60000;
static const uint8_t ABSENT_MISSES = 3;

static void verifyNext()
{
    static uint16_t last = 0; // address queried last time: the next one follows
    const TickType_t now = xTaskGetTickCount();
    uint16_t list[32];
    const uint8_t count = knownAddresses(list, 32);
    uint8_t start = 0;
    while (start < count && list[start] <= last)
        start++;
    for (uint8_t k = 0; k < count; k++)
    {
        const uint16_t a = list[(start + k) % count];
        last = a;
        Loco *l = Loco::findLoco(a);
        if (l == nullptr)
            continue;
        if (l->uid32() == 0)
            continue; // UID unknown (added by an order of the master): a verify with UID 0 would delete it
        Presence &p = sPresence[a];
        if (p.absent && now - p.since < pdMS_TO_TICKS(ABSENT_RETRY_MS))
            continue; // queried again later
        MfxFrame f;
        f.address(a);
        f.verify(l->uid32());
        f.finish();
        Feedback::Result r;
        // Presence only: a weak answer counts too (the 241 A answers some
        // queries with 64 or 105 edges instead of 300, 1 Nov; the noise
        // gives at most 5 regular intervals)
        const bool yes = Feedback::ask(f, &r) || weakAnswer(r);
        if (yes)
        {
            if (p.absent)
                Serial.printf("Discovery: %s (address %u) is on the track\n", l->name().c_str(), a);
            p.absent = false;
            p.misses = 0;
            vTaskDelay(pdMS_TO_TICKS(ANSWER_PAUSE_MS));
        }
        else if (p.absent)
            p.since = now;
        else if (r.referenceRegular >= NOISY_REFERENCE)
            ; // the reference window is noisy: no conclusion, asked again next turn
        else if (++p.misses >= ABSENT_MISSES)
        {
            Serial.printf("Discovery: %s (address %u) does not answer, queried again every minute\n",
                          l->name().c_str(), a);
            p.absent = true;
            p.since = now;
        }
        return;
    }
}

/*
 Orders of the master (Rocrail as master, or an MS2), executed as a
 Gleisbox does (recorded on the CAN bus, 31 Oct):
   bind 0x02 (UID, SID): the bind sent 4 times on the track, then the same
     6 bytes back, response bit set;
   verify 0x03 (UID, SID): existence query on the track; answer with 7
     bytes (UID, SID, quality FF) if the decoder answers, else 6 bytes with
     the SID set to 0. With the UID 0, the decoder at SID forgets its address
     (deletion by the MS2); answer: UID 0, SID 0.
 The master gives the addresses: the locomotive at SID is created (or moved
 there) with the UID, so that it is refreshed and queried.
*/
struct MasterOrder
{
    uint8_t type; // 2 bind, 3 verify, 7 read config
    uint32_t uid; // read config: the CS2 locomotive identifier
    uint16_t sid;
    uint16_t cv;
    uint8_t index;
    uint8_t count;
};
static QueueHandle_t sOrders = nullptr;

static void postOrder(uint8_t type, uint32_t uid, uint16_t sid, uint16_t cv = 0, uint8_t index = 0,
                      uint8_t count = 0)
{
    if (sOrders == nullptr)
        sOrders = xQueueCreate(8, sizeof(MasterOrder));
    const MasterOrder o = {type, uid, sid, cv, index, count};
    if (sOrders == nullptr || xQueueSend(sOrders, &o, 0) != pdPASS)
        Serial.println("Discovery: order of the master lost (queue full)");
}

void Discovery::requestBind(uint32_t uid, uint16_t sid) { postOrder(2, uid, sid); }
void Discovery::requestVerify(uint32_t uid, uint16_t sid) { postOrder(3, uid, sid); }
void Discovery::requestRead(uint32_t locId, uint16_t cv, uint8_t index, uint8_t count)
{
    postOrder(7, locId, (uint16_t)(locId & 0x3FFF), cv, index, count);
}

/*
 Read config 0x07 (CAN 0x07, recorded between an MS2 and a Gleisbox):
 order "locomotive identifier, index << 10 | CV, count"; the Gleisbox
 answers each byte in a frame of 7 bytes: identifier, index << 10 | CV
 (the index of this byte), value. Read on the track 2 bytes at a time, as
 the MS2 does. A byte that could not be read is answered with 6 bytes
 (no value), as the Gleisbox does; address 0 (broadcast) is not read.
*/
// A byte not read: identifier and index << 10 | CV only, 6 bytes (as the
// Gleisbox answers, recorded 2 Nov 2026)
static void readFailed(const MasterOrder &o, uint8_t from)
{
    for (uint8_t i = from; i < o.count; i++)
    {
        const uint16_t ic = (uint16_t)((o.index + i) & 0x3F) << 10 | (o.cv & 0x3FF);
        const uint8_t d[6] = {(uint8_t)(o.uid >> 24), (uint8_t)(o.uid >> 16), (uint8_t)(o.uid >> 8),
                              (uint8_t)o.uid, (uint8_t)(ic >> 8), (uint8_t)ic};
        Message::answer(CAN_READ_CONFIG, 6, d);
    }
}

/*
 Registration by the master, end. After the bind, Rocrail reads the name and
 CVs 2 to 70, then asks a verify; the answer makes it create the locomotive
 in its list. With the 040 DE, the Serie 150X and the 150 X 1518 it never
 asked the verify (2-3 Nov 2026), with the 241 A it always did: the
 decision is in Rocrail. When the master bound a decoder, read it and
 asked nothing more for PENDING_VERIFY_MS, the command station verifies the
 decoder itself and gives the answer to the master, as if it had asked.
 Rocrail then creates the locomotive (or finds it by its address).
*/
static const uint32_t PENDING_VERIFY_MS = 8000;
static struct
{
    bool active;
    bool read; // the master read at least one CV since the bind
    uint32_t uid;
    uint16_t sid;
    uint32_t last; // millis() of the last order of the master for it
} sPending = {};

static void pendingTouch(uint16_t sid)
{
    if (sPending.active && sPending.sid == sid)
    {
        sPending.last = millis();
        sPending.read = true;
    }
}

static void readForMaster(const MasterOrder &o)
{
    pendingTouch(o.sid);
    // Address 0 is the broadcast: every decoder would answer together
    if (o.sid == 0)
    {
        Serial.printf("Discovery: read of CV %u at address 0 refused (every decoder would answer)\n", o.cv);
        readFailed(o, 0);
        return;
    }
    // A running locomotive cannot be read: its own motor spoils its answer
    // (test bench, 2 Nov: 0 reads out of 10, at speed 12 and 63)
    if (Loco *running = Loco::findLoco(o.sid))
        if (running->speed() != 0)
        {
            Serial.printf("Discovery: read of CV %u at address %u refused: the locomotive is running\n", o.cv, o.sid);
            readFailed(o, 0);
            return;
        }
    // The name of the decoder (CV 3, index 1, 16 characters), read by the
    // master: kept as the name of the locomotive (shown on an MS2)
    const bool isName = (o.cv == 3 && o.index == 1 && o.count >= 2 && o.count <= 16);
    char name[17];
    memset(name, 0, sizeof(name));
    const uint32_t tries0 = Feedback::readTries;
    uint8_t done = 0;
    while (done < o.count)
    {
        const uint8_t n = (o.count - done) >= 2 ? 2 : 1;
        uint8_t v[2] = {0, 0};
        const uint8_t index = (uint8_t)(o.index + done);
        if (!Feedback::readCV(o.sid, o.cv, index, n, v))
        {
            Serial.printf("Discovery: read of CV %u index %u at address %u for the master: no answer\n", o.cv,
                          index, o.sid);
            readFailed(o, done);
            return;
        }
        for (uint8_t k = 0; k < n; k++)
        {
            const uint16_t ic = (uint16_t)((index + k) & 0x3F) << 10 | (o.cv & 0x3FF);
            const uint8_t d[7] = {(uint8_t)(o.uid >> 24), (uint8_t)(o.uid >> 16), (uint8_t)(o.uid >> 8),
                                  (uint8_t)o.uid, (uint8_t)(ic >> 8), (uint8_t)ic, v[k]};
            Message::answer(CAN_READ_CONFIG, 7, d);
            if (isName)
                name[done + k] = (char)v[k];
        }
        done += n;
    }
    if (isName && name[0] >= 0x20)
    {
        bool printable = true;
        for (uint8_t i = 0; name[i]; i++)
            printable = printable && name[i] >= 0x20 && name[i] < 0x7F;
        Loco *l = Loco::findLoco(o.sid);
        if (printable && l != nullptr && l->name() != String(name))
        {
            Serial.printf("Discovery: locomotive %u named \"%s\" (read in the decoder)\n", o.sid, name);
            l->name(String(name));
            LocoStore::save();
        }
    }
    Serial.printf("Discovery: CV %u index %u..%u at address %u read for the master (%lu tries on the track)\n",
                  o.cv, o.index, o.index + o.count - 1, o.sid, (unsigned long)(Feedback::readTries - tries0));
}

// The locomotive with this UID, at address sid (created or moved there)
static Loco *locoAt(uint32_t uid, uint16_t sid)
{
    Loco *l = Loco::findByUid(uid);
    if (l != nullptr && l->addr() == sid)
        return l;
    String name = l != nullptr ? l->name() : String("mfx ") + String(uid, HEX);
    if (l != nullptr)
    {
        const uint16_t old = l->addr();
        Message::removeLoco(old);
        Loco::removeLoco(old);
    }
    if (Loco *other = Loco::findLoco(sid))
    {
        Serial.printf("Discovery: address %u was %s, given by the master to %08lX\n", sid, other->name().c_str(),
                      (unsigned long)uid);
        Message::removeLoco(sid);
        Loco::removeLoco(sid);
    }
    l = Loco::createLoco(sid, name, uid >> 24, uid >> 16, uid >> 8, uid);
    if (!Message::addLoco(l))
        Serial.println("Discovery: too many locomotives, see MAX_LOCOS in main.cpp");
    return l;
}

void Discovery::masterOrders()
{
    MasterOrder o;
    while (sOrders != nullptr && xQueueReceive(sOrders, &o, 0) == pdPASS)
    {
        if (o.type == 7)
        {
            readForMaster(o);
            continue;
        }
        const uint8_t d[7] = {(uint8_t)(o.uid >> 24), (uint8_t)(o.uid >> 16), (uint8_t)(o.uid >> 8), (uint8_t)o.uid,
                              (uint8_t)(o.sid >> 8), (uint8_t)o.sid, 0xFF};
        if (o.sid > 16383 || (o.sid == 0 && o.uid != 0))
        {
            Serial.printf("Discovery: SID %u not valid (1 to 16383)\n", o.sid);
            continue;
        }
        if (o.type == 2)
        {
            Loco *l = locoAt(o.uid, o.sid);
            sendBind(l, false);
            LocoStore::save();
            Message::answer(CAN_MFX_BIND, 6, d);
            Serial.printf("Discovery: bind of %08lX at address %u by the master, done\n", (unsigned long)o.uid,
                          o.sid);
            sPending = {true, false, o.uid, o.sid, millis()};
            forgetDrop(o.uid);
            continue;
        }
        // verify
        if (sPending.active && sPending.uid == o.uid)
            sPending.active = false; // asked by the master: nothing to add
        MfxFrame f;
        f.address(o.sid);
        f.verify(o.uid);
        f.finish();
        if (o.uid == 0)
        {
            // Deletion by the master: the decoder at sid forgets its address
            if (Centrale::power())
                sendForget(o.sid);
            if (Loco *l = Loco::findLoco(o.sid))
            {
                forgetAdd(l->uid32(), o.sid); // checked again later
                Message::removeLoco(o.sid);
                Loco::removeLoco(o.sid);
                sPresence.erase(o.sid);
                sReadTries.erase(o.sid);
                LocoStore::save();
            }
            else if (!Centrale::power())
                forgetAdd(0, o.sid);
            const uint8_t none[6] = {0, 0, 0, 0, 0, 0};
            Message::answer(CAN_MFX_VERIFY, 6, none);
            Serial.printf("Discovery: address %u deleted by the master\n", o.sid);
            continue;
        }
        Feedback::Result r;
        const bool yes = Feedback::ask(f, &r) || r.windowRegular >= 20;
        if (yes)
        {
            Message::answer(CAN_MFX_VERIFY, 7, d);
            vTaskDelay(pdMS_TO_TICKS(ANSWER_PAUSE_MS));
        }
        else
        {
            const uint8_t no[6] = {d[0], d[1], d[2], d[3], 0, 0};
            Message::answer(CAN_MFX_VERIFY, 6, no);
        }
        Serial.printf("Discovery: verify of %08lX at address %u by the master: %s (%u regular intervals)\n",
                      (unsigned long)o.uid, o.sid, yes ? "answer" : "no answer", r.windowRegular);
    }

    // The master read the decoder it bound, then stopped without a verify
    if (sPending.active && sPending.read && millis() - sPending.last >= PENDING_VERIFY_MS)
    {
        sPending.active = false;
        if (!Centrale::power())
            return;
        MfxFrame f;
        f.address(sPending.sid);
        f.verify(sPending.uid);
        f.finish();
        bool yes = false;
        Feedback::Result r = {};
        for (uint8_t t = 0; t < TRIES && !yes; t++)
        {
            yes = Feedback::ask(f, &r) || r.windowRegular >= 20;
            if (!yes)
                vTaskDelay(pdMS_TO_TICKS(MISS_PAUSE_MS));
        }
        if (yes)
        {
            const uint8_t d[7] = {(uint8_t)(sPending.uid >> 24), (uint8_t)(sPending.uid >> 16),
                                  (uint8_t)(sPending.uid >> 8), (uint8_t)sPending.uid,
                                  (uint8_t)(sPending.sid >> 8), (uint8_t)sPending.sid, 0xFF};
            Message::answer(CAN_MFX_VERIFY, 7, d);
        }
        Serial.printf("Discovery: %08lX at address %u read by the master, which asked no verify: %s\n",
                      (unsigned long)sPending.uid, sPending.sid,
                      yes ? "verified here, answer sent to the master (it creates the locomotive)"
                          : "no answer on the track, nothing sent");
    }
}

void Discovery::task(void *)
{
    vTaskDelay(pdMS_TO_TICKS(3000)); // let the decoders start
    uint32_t last = 0;
    TickType_t lastTime = 0;
    // As a Gleisbox: the configurations are read by the master (CAN 0x07),
    // not here; serial command C reads them all the same (test)
    bool configsDone = true;
    TickType_t lastConfigs = 0;

    for (;;)
    {
        console();
        masterOrders();
        if (sRereadConfigs)
        {
            sRereadConfigs = false;
            configsDone = false;
            lastConfigs = 0;
        }

        // The configurations are read once the track has been on for
        // POWERED_MS without a break (the decoders must have started),
        // then again every RETRY_MS for the decoders that did not answer.
        const TickType_t now = xTaskGetTickCount();
        if (!configsDone && Centrale::poweredForMs() >= POWERED_MS &&
            (lastConfigs == 0 || now - lastConfigs >= pdMS_TO_TICKS(RETRY_MS)))
        {
            configsDone = readDeclaredConfigs();
            lastConfigs = xTaskGetTickCount();
        }
        if (Centrale::power())
        {
            uint32_t uid = 0;
            if (findUid(&uid))
            {
                const TickType_t now = xTaskGetTickCount();
                if (uid == last && now - lastTime < pdMS_TO_TICKS(SAME_DECODER_MS))
                {
                    // It still answers after its Bind: do not search all the time
                    Serial.printf("Discovery: decoder %08lX still answers the search, pause\n",
                                  (unsigned long)uid);
                    vTaskDelay(pdMS_TO_TICKS(SAME_DECODER_MS));
                }
                else
                {
                    // As a Gleisbox: the UID found goes to the master, which
                    // chooses the address and sends the bind (CAN 0x02)
                    const uint8_t d[5] = {(uint8_t)(uid >> 24), (uint8_t)(uid >> 16), (uint8_t)(uid >> 8),
                                          (uint8_t)uid, 32};
                    const bool sent = Message::answer(CAN_LOCO_DISCOVERY, sizeof(d), d);
                    Serial.printf("Discovery: decoder %08lX found, reported to the master%s\n",
                                  (unsigned long)uid, sent ? "" : " (NOT connected)");
                    forgetDrop(uid);
                }
                last = uid;
                lastTime = xTaskGetTickCount();
            }
            if (Centrale::power())
                verifyNext();
            forgetCheck();
        }
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}
