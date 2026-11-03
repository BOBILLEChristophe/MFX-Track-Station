/*

   Feedback.h

   Feedback from the decoders (1-bit feedback, Krauss "Schienenformat mfx").

   Hardware: a current sensor on one track wire (iron powder toroid, tuned
   circuit at about 53 kHz, comparator MCP6541). Its output, 0 / 3.3 V, goes
   to a GPIO of the ESP32. A decoder that answers draws a current at
   52.6 kHz: the comparator then gives a square wave of 19 µs period.

   How it works:
   - ask() hands a frame to the signal task (MfxRMT), which sends it followed
     by the feedback request and two windows of 6.4 ms at constant level
     (MfxSignal::feedbackWindows()).
   - When the chunk starts, MfxRMT calls windowsStarted(). Timers then count
     the rising edges with the PCNT, in the answer window and in the
     reference window (no answer expected: it measures the noise).
   - The first 200 µs of each window are not counted: the tuned circuit
     still rings after the last polarity change of the track (60 to 80 µs
     measured).
   - A decoder has answered when the answer window holds many edges at the
     carrier period (19 µs apart, timed by the interrupt), and many more than
     the reference window. The noise of a running motor is mostly at random
     intervals; at some speeds it gives regular intervals too, but as many
     in both windows.

   Data feedback (readCV(), Krauss 2.2.8 and 2.3.3):
   - readCV() hands a read CV frame to the signal task, which sends it
     followed by the data request and a long pause with clock pulses
     (MfxSignal::dataWindow(): 49 ms for 2 bytes).
   - During the pause, an interrupt stores the time of each rising edge of
     the comparator (cycle counter of the CPU).
   - MfxDemod then finds the phase changes of the carrier, the bits, the
     bytes, and checks the checksum.

*/

#pragma once

#include <Arduino.h>
#include "MfxFrame.h"

class Feedback
{
public:
    struct Result
    {
        int16_t window;    // rising edges in the answer window
        int16_t reference; // rising edges in the reference window
        // Intervals of one carrier period (17 to 21 µs) between two rising
        // edges: about 300 for an answer, few for the noise of a motor
        uint16_t windowRegular;
        uint16_t referenceRegular;
        // Longest run of consecutive regular intervals in each window
        uint16_t windowRun;
        uint16_t referenceRun;
        bool answered;
    };

    // Not counted at the start and at the end of each window, in µs
    static const uint32_t SKIP_START_US = 200;
    static const uint32_t SKIP_END_US = 100;
    // About 52.6 edges per ms over 6.1 ms: 320 for a full answer, of which
    // about 300 at regular intervals (measured on recorded answers).
    // A motor at some speeds (150X at speed 101) gives up to 125 regular
    // intervals in both windows; their difference stayed below 60.
    static const uint16_t ANSWER_MIN_REGULAR = 150; // in the answer window
    static const uint16_t ANSWER_MARGIN = 120;      // more than in the reference
    // Or an answer spoilt by the noise of another locomotive (116 regular
    // intervals, 2 Nov): one long run of carrier periods, which the noise
    // never gives (4 at most measured, motors and sounds, 2 Nov)
    static const uint16_t ANSWER_RUN_MIN = 15;         // consecutive regular intervals
    static const uint16_t ANSWER_RUN_MIN_REGULAR = 80;
    static const uint16_t ANSWER_RUN_MARGIN = 60;

    // Pause between two reads of CVs (Krauss 4.9: about 120 ms for the
    // first ESU decoders), in ms
    static const uint32_t READ_PAUSE_MS = 250; // the MS2 waits about 240 ms between reads of an ESU decoder

    Feedback() = delete;

    // pin: output of the comparator. Call before RMTChannel::setup().
    static void setup(gpio_num_t pin);
    static bool ready() { return sReady; }

    // Sends the frame (address + command + CRC, see MfxFrame::finish())
    // followed by a 1-bit feedback request, and waits for the result.
    // Returns true if a decoder answered. Blocks about 30 ms.
    // Several tasks may call it: the requests are sent one by one.
    static bool ask(const MfxFrame &frame, Result *result = nullptr);

    // Reads `bytes` CVs (1, 2, 4 or 8) of the decoder at rail address sid,
    // from cv/index on (Krauss 3.2.1), into out[0..bytes-1]. Tries twice.
    // Returns true if the answer was received with a good checksum.
    // Blocks about 60 to 250 ms. Several tasks may call it.
    static bool readCV(uint16_t sid, uint16_t cv, uint8_t index, uint8_t bytes, uint8_t *out);
    // Counts of readCV() since the start: tries sent on the track (the first
    // one and the repeats) and reads that needed more than one try
    static uint32_t readTries;
    static uint32_t readRetried;

    // Serial port trace. cv: one line per CV read (and the details of a
    // failed answer). search: the edge counts of each 1-bit request (two
    // lines every 2 s during the automatic discovery). Both off by default.
    static void setTrace(bool cv, bool search = false)
    {
        sTrace = cv;
        sTraceSearch = search;
    }

    // Trigger for an oscilloscope: the pin goes high during the feedback
    // windows. ONE_BIT: from the start of the answer window (+200 µs) to the
    // end of the reference window (about 12.8 ms). DATA: during the capture
    // of a CV answer (49 ms for 2 bytes, 64 ms for 4). BOTH: either.
    // sid: only the reads of CVs of this rail address (0: every address).
    // The edges come from the esp_timer task: a few tens of µs late at most.
    enum ScopeMode : uint8_t
    {
        SCOPE_OFF,
        ONE_BIT,
        DATA,
        BOTH
    };
    static void setScope(gpio_num_t pin, ScopeMode mode, uint16_t sid = 0);

    // --- Used by MfxRMT (signal task) ---
    // Takes the pending request, if any (frame in the format of mfxQueue).
    // dataBytes: 0 for a 1-bit feedback, else the bytes of a data feedback.
    static bool takeRequest(uint8_t *frame, size_t size, uint8_t *dataBytes);
    // The chunk with the request started at startUs (esp_timer_get_time());
    // windowUs, referenceUs: start of the two windows from the chunk start.
    static void windowsStarted(int64_t startUs, uint32_t windowUs, uint32_t referenceUs);
    // The chunk with a data request started at startUs; the pause starts
    // pauseUs later (MfxSignal::dataWindow()).
    static void dataStarted(int64_t startUs, uint32_t pauseUs, uint8_t bytes);
    // The request could not be sent (frame too long)
    static void cancel();

private:
    static void onTimer(void *arg);
    static bool readOnce(const uint8_t *frame, uint8_t bytes, uint8_t *out);

    static bool sReady;
    static bool sTrace;       // reading of CVs
    static bool sTraceSearch; // 1-bit feedback
};
