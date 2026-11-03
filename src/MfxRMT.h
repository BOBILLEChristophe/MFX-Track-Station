/*

   MfxRMT.h

   Track signal generator: one task owns the RMT channel and keeps it busy.
   Frames come from mfxQueue; when there is none, synchronisation patterns
   fill the line, so the signal never stops.
   A frame that asks for a feedback (Feedback::ask()) goes first; it is
   followed by the feedback request and its two windows.

*/

#pragma once

#include <Arduino.h>
#include "driver/rmt.h"
#include "MfxSignal.h"

extern QueueHandle_t mfxQueue;
#define BUFFER_SIZE 100 // size of one frame in mfxQueue (see MfxFrame::toBuffer)

class RMTChannel
{
public:
    RMTChannel() = delete;
    static void setup(gpio_num_t IN1, gpio_num_t IN2);
    // Test (serial command H): after each frame of the refresh, a
    // synchronisation and a hold of the track of END_HOLD_US, as the
    // Gleisbox does (with an MS2: a pause of 1 to 2 ms after nearly every
    // frame, 1667 in 30 s; our station sent syncs without a break)
    static volatile bool framePause;

private:
    static void signalTask(void *parameter);
    static int64_t send(const MfxSignal &signal); // returns the start time (µs)

    // Two buffers: one is being sent while the next chunk is built in the other
    static rmt_item32_t sItems[2][MfxSignal::MAX_ITEMS];
    static uint8_t sCurrent;
};
