/*
  MfxSignal.h

  Turns mfx frames into the track signal, as a list of items of two
  50 µs halves (Stefan Krauss, "Schienenformat mfx", §2.2.1):
    - bit 0 : a transition at the start, then 100 µs without transition;
    - bit 1 : a transition at the start and another one in the middle;
    - synchronisation : 5 items (500 µs) with deliberate rule violations;
    - bit stuffing : a 0 is inserted after 8 consecutive 1 bits.

  The level of the track is kept from one call to the next, so that
  consecutive chunks join without a missing or spurious transition.

  Feedback (decoder -> command station): after a frame that asks for an
  answer, feedbackWindows() adds the request sequence and two windows in
  which the track keeps a constant level. The decoder answers in the
  first window by drawing a current at about 52.6 kHz; the second window
  is a reference (no answer expected) to measure the noise.

  Data feedback (reading CVs): after the read command, dataWindow() adds
  the request sequence and a long pause in which the command station gives
  the clock to the decoder with short pulses (Krauss 2.2.8): 23 pulses
  every 912 µs (the decoder sends 1 bits), then a pulse every 456 µs for
  the preamble, the data, the checksum and 4 spare clocks. The decoder
  answers during the whole pause, by phase modulation of its 52.6 kHz
  current (see MfxDemod).

  Pure C++, no ESP32 dependency: tested on a PC (pio test -e native).
  MfxRMT copies the items into rmt_item32_t (1 µs per tick).

  mfx is a registered trademark of Gebr. Marklin & Cie GmbH.
*/

#pragma once

#include <stdint.h>

struct MfxItem
{
    bool level0;        // first half
    bool level1;        // second half
    uint16_t duration0; // µs, 50 for a bit or a synchronisation
    uint16_t duration1; // µs
};

class MfxSignal
{
public:
    static const uint16_t HALF_US = 50; // half of a bit

    // Feedback request, as a Marklin MS2 with its Gleisbox sends it (recorded
    // on the track, October 2026): the question (search, verify, read CV),
    // a synchronisation, the command station frame (111 101), then
    // 2 synchronisations, the bits 0011 and windows of 6.0 ms.
    // A recent ESU decoder (040 DE 9) answers this request, but did not
    // answer the former one (11 synchronisations, windows of 6.4 ms).
    static const uint8_t FEEDBACK_SYNCS = 2;         // synchronisations before 0011
    static const uint16_t FEEDBACK_WINDOW_US = 6000; // window of the answer
    static const uint8_t FEEDBACK_END_SYNCS = 1;     // synchronisations at the end
    static const uint8_t DATA_END_SYNCS = 2;         // after the data pause
    // After a feedback request (windows or data pause) and after a bind, the
    // Gleisbox holds the track still: 1.59 ms from the last transition of the
    // final synchronisation (its last 100 µs half-bit included). Measured,
    // October 2026. It is also where the short idle time between two RMT
    // chunks falls, out of the synchronisations.
    static const uint16_t END_HOLD_US = 1494;
    // Level of the track during the answer window (the data pause too), and
    // the opposite level during the reference window. The Gleisbox always
    // holds its answer window at the same polarity of the track, whatever
    // the bits sent before (recorded, October 2026: 237 requests out of 237):
    // when the bits 0011 end at the other level, it adds a transition right
    // after the last one (FORCE_US later). A recent ESU decoder answers only
    // at that polarity. Which level of the RMT output it is depends on the
    // wiring of the H-bridge to the track: set it with the sniffer, so that
    // its R and D lines show the windows at the level the Gleisbox uses.
    static bool windowLevel;
    static const uint16_t FORCE_US = 2; // the two halves of the flip item
    // Test (serial command F): the flip before the answer window. 0: no
    // flip (the window stays at the level the bits 0011 end at); else the
    // time in µs from the end of the last bit to the flip. The 040 DE
    // (ESU) does not answer when this flip is there (an odd number of
    // stuffed bits in the frame), test bench 2 Nov.
    static uint16_t answerFlipUs;
    // The bits 0011 always followed by a transition, as on the Gleisbox
    // (recorded with the MS2, 2 Nov 2026: every request, both cases): when
    // the bits end at windowLevel already, it adds a pulse of about 2 µs at
    // the other level, the answer window still being at windowLevel. The
    // 040 DE (ESU) answers only with this transition, the 150 X 1518 and the
    // 150 Y only at windowLevel. Serial command A switches it off and on.
    static bool answerPulse;

    // Frame inserted between the question and the request (the command
    // station frame: 9 + 6 + 32 + 16 + 8 = 71 bits), at most
    static const uint8_t INSERT_MAX_BITS = 71;

    // Data feedback (Krauss 2.2.8, timing measured on a Marklin Gleisbox)
    static const uint16_t DATA_CLOCK_US = 912;   // one clock = one bit
    static const uint16_t DATA_PULSE_US = 25;    // clock pulse, opposite level
    static const uint8_t DATA_LEAD_CLOCKS = 23;  // clocks without data (1 bits)
    static const uint8_t DATA_MAX_BYTES = 8;
    // Clocks with a pulse in the middle: preamble 010, data, checksum, 4 spare
    static uint16_t dataClocks(uint8_t bytes) { return 3 + 8 * bytes + 8 + 4; }
    // Length of the pause (the decoder answers in it), in µs
    static uint32_t dataPauseUs(uint8_t bytes)
    {
        return (uint32_t)(DATA_LEAD_CLOCKS + dataClocks(bytes)) * DATA_CLOCK_US;
    }

    // Room for the longest chunk: sync + 128-bit frame + stuffing
    //   + sync + inserted frame + stuffing
    //   + data feedback of 8 bytes: syncs, 4 bits, pause (one item per
    //   pulse, plus one), sync. The 1-bit feedback is shorter.
    static const uint16_t MAX_ITEMS = 5 + 128 + 16 + 5 + INSERT_MAX_BITS + INSERT_MAX_BITS / 8 +
                                      5 * FEEDBACK_SYNCS + 4 + 1 + DATA_LEAD_CLOCKS +
                                      2 * (3 + 8 * DATA_MAX_BYTES + 8 + 4) + 1 + 5 * DATA_END_SYNCS + 1;

    MfxSignal() : mCount(0), mLevel(false), mOverflow(false) {}

    // Empties the item list. The track level is kept.
    void clear()
    {
        mCount = 0;
        mOverflow = false;
    }

    // Level of the track at the end of the last item.
    bool level() const { return mLevel; }
    void setLevel(bool level) { mLevel = level; }

    // Synchronisation pattern, 500 µs. It ends at the level it started from.
    void sync();

    // Frame in the format of MfxFrame::toBuffer():
    // buff[0] = number of bits, buff[1..n] = one bit per byte.
    // Exactly buff[0] bits are sent, plus the stuffing bits.
    void frame(const uint8_t *buff);

    void bit0();
    void bit1();

    // Keeps the current level for us µs, without any transition (one item,
    // two halves of us/2). us <= 65534.
    void hold(uint32_t us);

    // Request for a 1-bit feedback, to add after the frame:
    //   [synchronisation, the frame `insert` (format of MfxFrame::toBuffer()),
    //   if not null: the MS2 sends its command station frame there,]
    //   FEEDBACK_SYNCS synchronisations, bits 0011,
    //   window of 6.0 ms at windowLevel (the decoder answers here),
    //   synchronisation, reference window of 6.0 ms at !windowLevel (no answer),
    //   FEEDBACK_END_SYNCS synchronisation, END_HOLD_US without transition.
    // A window at the wrong level after the bits before it starts with a flip
    // item: FORCE_US at the current level, then FORCE_US at the right one.
    // windowUs and referenceUs receive the start time of the two windows,
    // in µs from the start of the chunk.
    void feedbackWindows(uint32_t *windowUs, uint32_t *referenceUs, const uint8_t *insert = nullptr);

    // Request for a data feedback of `bytes` bytes (1, 2, 4 or 8), to add
    // after a read CV frame:
    //   [synchronisation and the frame `insert`, as for feedbackWindows(),]
    //   FEEDBACK_SYNCS synchronisations, bits 0011, [flip item,]
    //   pause at windowLevel with a short pulse of the opposite level at
    //   the end of each period: 23 periods of 912 µs, then 2 x dataClocks()
    //   periods of 456 µs, then 50 µs,
    //   DATA_END_SYNCS synchronisations, END_HOLD_US without transition.
    // pauseUs receives the start time of the pause, in µs from the start of
    // the chunk. It lasts dataPauseUs(bytes).
    void dataWindow(uint8_t bytes, uint32_t *pauseUs, const uint8_t *insert = nullptr);

    // Duration of the items, in µs
    uint32_t durationUs() const;

    uint16_t count() const { return mCount; }
    const MfxItem &item(uint16_t i) const { return mItems[i]; }
    bool overflow() const { return mOverflow; }

private:
    // Start of a feedback request: optional inserted frame, syncs, 0011
    void request(const uint8_t *insert);
    // Brings the track to `level`: nothing if it is already there, else one
    // item of 2 x FORCE_US with the transition in the middle
    void forceLevel(bool level, uint16_t delayUs = FORCE_US);
    void push(bool level0, bool level1, uint16_t duration0 = HALF_US, uint16_t duration1 = HALF_US);

    MfxItem mItems[MAX_ITEMS];
    uint16_t mCount;
    bool mLevel;
    bool mOverflow;
};
