/*

   Feedback.cpp

   PCNT (pulse counter) and esp_timer, ESP-IDF 4.4 API (Arduino core 2.0.x).
   With the move to the core 3.x, the PCNT will go to driver/pulse_cnt.h.

   Data feedback: the rising edges are timed with the cycle counter of the
   CPU, in the interrupt (always on the core that called setup()).

*/

#include "Feedback.h"
#include "MfxRMT.h" // BUFFER_SIZE
#include "MfxSignal.h"
#include "MfxDemod.h"
#include "MfxFrame.h"
#include "driver/pcnt.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_idf_version.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "esp_cpu.h"
#define CYCLES() esp_cpu_get_cycle_count()
#else
#include "hal/cpu_hal.h"
#define CYCLES() cpu_hal_get_cycle_count()
#endif

bool Feedback::sReady = false;
bool Feedback::sTrace = false;
bool Feedback::sTraceSearch = false;

static const pcnt_unit_t UNIT = PCNT_UNIT_0;

// A request for the signal task: a frame, and the kind of feedback
struct Request
{
    uint8_t dataBytes; // 0: 1-bit feedback, else bytes of a data feedback
    uint8_t frame[BUFFER_SIZE];
};
static QueueHandle_t sRequest = NULL; // one request waiting for the signal task
static QueueHandle_t sResult = NULL;  // Feedback::Result
static SemaphoreHandle_t sMutex = NULL;

// Steps of the measurement, one esp_timer each
enum Step : uint8_t
{
    CLEAR_WINDOW,
    READ_WINDOW,
    CLEAR_REFERENCE,
    READ_REFERENCE,
    DATA_START, // data feedback: start and end of the capture of the edges
    DATA_END,
    STEPS
};
static esp_timer_handle_t sTimers[STEPS];
static volatile int16_t sWindowEdges = 0;

// Data feedback: times of the rising edges during the pause
static const uint16_t EDGES_MAX = 6000; // 8 bytes: 93 ms x 52.6 edges/ms, and spare
static uint32_t *sEdges = NULL;
static volatile uint16_t sEdgeCount = 0;
static volatile bool sCapture = false;
static SemaphoreHandle_t sDataDone = NULL; // given at the end of the pause
static MfxDemod sDemod;                    // used under sMutex

// 1-bit feedback: intervals of one carrier period (17 to 21 µs) between
// two rising edges, counted in each window. The noise of a running motor
// gives many edges, but at random intervals.
static volatile bool sCountRegular = false;
static volatile uint32_t sLastEdge = 0;   // cycles
static volatile uint16_t sRegular = 0;
static uint32_t sRegularMin = 17 * 240, sRegularMax = 21 * 240; // cycles, set in setup()
static volatile uint16_t sWindowRegular = 0;
// Longest run of consecutive regular intervals: an answer is one long
// carrier (about 300 periods in a row); noise gives scattered intervals
static volatile uint16_t sRun = 0, sRunMax = 0;
static volatile uint16_t sWindowRun = 0;

// Trigger for an oscilloscope (setScope())
static gpio_num_t sScopePin = GPIO_NUM_NC;
static Feedback::ScopeMode sScopeMode = Feedback::SCOPE_OFF;
static uint16_t sScopeSid = 0;          // 0: every address
static volatile uint16_t sReadSid = 0;  // address of the read in progress
static inline void scope(bool data, bool level)
{
    if (sScopePin == GPIO_NUM_NC)
        return;
    if (data && level && sScopeSid != 0 && sReadSid != sScopeSid)
        return;
    if (sScopeMode == Feedback::BOTH || sScopeMode == (data ? Feedback::DATA : Feedback::ONE_BIT))
        gpio_set_level(sScopePin, level);
}

static void IRAM_ATTR onEdge()
{
    const uint32_t c = CYCLES();
    if (sCountRegular)
    {
        const uint32_t dt = c - sLastEdge;
        if (dt >= sRegularMin && dt <= sRegularMax)
        {
            sRegular = sRegular + 1;
            const uint16_t run = sRun + 1;
            sRun = run;
            if (run > sRunMax)
                sRunMax = run;
        }
        else
            sRun = 0;
    }
    sLastEdge = c;
    if (sCapture)
    {
        const uint16_t n = sEdgeCount;
        if (n < EDGES_MAX)
        {
            sEdges[n] = c;
            sEdgeCount = n + 1;
        }
    }
}

void Feedback::setup(gpio_num_t pin)
{
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = pin;
    cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
    cfg.channel = PCNT_CHANNEL_0;
    cfg.unit = UNIT;
    cfg.pos_mode = PCNT_COUNT_INC; // rising edge: +1
    cfg.neg_mode = PCNT_COUNT_DIS; // falling edge: nothing
    cfg.lctrl_mode = PCNT_MODE_KEEP;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.counter_h_lim = 32767;
    cfg.counter_l_lim = -32768;
    if (pcnt_unit_config(&cfg) != ESP_OK)
    {
        Serial.println("Feedback: PCNT error, feedback disabled");
        return;
    }
    // Ignores pulses shorter than 1 µs (80 APB cycles at 80 MHz)
    pcnt_set_filter_value(UNIT, 80);
    pcnt_filter_enable(UNIT);
    pcnt_counter_pause(UNIT);
    pcnt_counter_clear(UNIT);
    pcnt_counter_resume(UNIT);

    for (uint8_t i = 0; i < STEPS; i++)
    {
        esp_timer_create_args_t args = {};
        args.callback = onTimer;
        args.arg = (void *)(uintptr_t)i;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "feedback";
        if (esp_timer_create(&args, &sTimers[i]) != ESP_OK)
        {
            Serial.println("Feedback: timer error, feedback disabled");
            return;
        }
    }

    const uint32_t mhz = getCpuFrequencyMhz();
    sRegularMin = 17 * mhz;
    sRegularMax = 21 * mhz;
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), onEdge, RISING);

    sRequest = xQueueCreate(1, sizeof(Request));
    sResult = xQueueCreate(1, sizeof(Result));
    sMutex = xSemaphoreCreateMutex();
    sDataDone = xSemaphoreCreateBinary();
    sEdges = (uint32_t *)malloc(EDGES_MAX * sizeof(uint32_t));
    if (sEdges == NULL)
        Serial.println("Feedback: out of memory, reading of CVs disabled");
    if (sRequest == NULL || sResult == NULL || sMutex == NULL || sDataDone == NULL)
    {
        Serial.println("Feedback: out of memory, feedback disabled");
        return;
    }
    sReady = true;
    Serial.printf("Feedback: comparator on GPIO %d\n", (int)pin);
}

/*
 Runs in the esp_timer task. The windows last 6.4 ms: a delay of a few tens
 of µs of this task does not matter, the first 200 µs are not counted anyway.
*/
void Feedback::onTimer(void *arg)
{
    const uint8_t step = (uint8_t)(uintptr_t)arg;
    int16_t edges = 0;
    switch (step)
    {
    case CLEAR_WINDOW:
    case CLEAR_REFERENCE:
        if (step == CLEAR_WINDOW)
            scope(false, true);
        pcnt_counter_clear(UNIT);
        sRegular = 0;
        sRun = 0;
        sRunMax = 0;
        sCountRegular = true;
        break;
    case READ_WINDOW:
        pcnt_get_counter_value(UNIT, &edges);
        sCountRegular = false;
        sWindowEdges = edges;
        sWindowRegular = sRegular;
        sWindowRun = sRunMax;
        break;
    case DATA_START:
        sEdgeCount = 0;
        sCapture = sEdges != NULL;
        scope(true, true);
        break;
    case DATA_END:
        sCapture = false;
        scope(true, false);
        xSemaphoreGive(sDataDone);
        break;
    case READ_REFERENCE:
    {
        pcnt_get_counter_value(UNIT, &edges);
        sCountRegular = false;
        scope(false, false);
        Result r = {};
        r.window = sWindowEdges;
        r.reference = edges;
        r.windowRegular = sWindowRegular;
        r.referenceRegular = sRegular;
        r.windowRun = sWindowRun;
        r.referenceRun = sRunMax;
        r.answered = false; // decided in ask()
        xQueueOverwrite(sResult, &r);
        break;
    }
    }
}

void Feedback::setScope(gpio_num_t pin, ScopeMode mode, uint16_t sid)
{
    sScopeSid = sid;
    if (sScopePin != GPIO_NUM_NC)
        gpio_set_level(sScopePin, 0);
    sScopeMode = mode;
    sScopePin = mode == SCOPE_OFF ? GPIO_NUM_NC : pin;
    if (sScopePin == GPIO_NUM_NC)
        return;
    // Low before it becomes an output, and no pull-up: gpio_reset_pin()
    // would enable the pull-up and give a short false trigger at boot
    gpio_set_level(pin, 0);
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << pin;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(pin, 0);
    Serial.printf("Feedback: oscilloscope trigger on GPIO %d (%s", (int)pin,
                  mode == ONE_BIT ? "1-bit feedback" : mode == DATA ? "reading of CVs" : "both");
    if (sid != 0 && mode != ONE_BIT)
        Serial.printf(", reads of address %u only", sid);
    Serial.println(")");
}

void Feedback::windowsStarted(int64_t startUs, uint32_t windowUs, uint32_t referenceUs)
{
    if (!sReady)
        return;
    const uint32_t length = MfxSignal::FEEDBACK_WINDOW_US;
    const int64_t at[READ_REFERENCE + 1] = {
        startUs + windowUs + SKIP_START_US,
        startUs + windowUs + length - SKIP_END_US,
        startUs + referenceUs + SKIP_START_US,
        startUs + referenceUs + length - SKIP_END_US,
    };
    const int64_t now = esp_timer_get_time();
    for (uint8_t i = 0; i <= READ_REFERENCE; i++)
    {
        esp_timer_stop(sTimers[i]); // in case a previous measurement is still pending
        const int64_t delay = at[i] - now;
        esp_timer_start_once(sTimers[i], delay > 0 ? (uint64_t)delay : 0);
    }
}

/*
 The capture starts at the start of the pause (the decoder starts its
 carrier there) and ends 300 µs before its end: the final synchronisation
 and the next chunk would give false edges.
*/
void Feedback::dataStarted(int64_t startUs, uint32_t pauseUs, uint8_t bytes)
{
    if (!sReady)
        return;
    const int64_t at[2] = {
        startUs + pauseUs,
        startUs + pauseUs + MfxSignal::dataPauseUs(bytes) - 300,
    };
    const int64_t now = esp_timer_get_time();
    for (uint8_t i = 0; i < 2; i++)
    {
        esp_timer_handle_t t = sTimers[DATA_START + i];
        esp_timer_stop(t);
        const int64_t delay = at[i] - now;
        esp_timer_start_once(t, delay > 0 ? (uint64_t)delay : 0);
    }
}

bool Feedback::takeRequest(uint8_t *frame, size_t size, uint8_t *dataBytes)
{
    if (!sReady || size < BUFFER_SIZE)
        return false;
    Request r;
    if (xQueueReceive(sRequest, &r, 0) != pdPASS)
        return false;
    memcpy(frame, r.frame, BUFFER_SIZE);
    if (dataBytes)
        *dataBytes = r.dataBytes;
    return true;
}

void Feedback::cancel()
{
    if (!sReady)
        return;
    Result r = {};
    xQueueOverwrite(sResult, &r);
    sCapture = false;
    sEdgeCount = 0;
    xSemaphoreGive(sDataDone);
}

bool Feedback::ask(const MfxFrame &frame, Result *result)
{
    Result r = {};
    if (!sReady)
    {
        if (result)
            *result = r;
        return false;
    }

    Request req;
    req.dataBytes = 0;
    if (!frame.toBuffer(req.frame, sizeof(req.frame)))
    {
        if (result)
            *result = r;
        return false;
    }

    xSemaphoreTake(sMutex, portMAX_DELAY);
    xQueueReset(sResult);
    xQueueSend(sRequest, &req, 0);
    // The chunk lasts about 30 ms, plus the end of the chunk being sent
    const bool done = xQueueReceive(sResult, &r, pdMS_TO_TICKS(200)) == pdPASS;
    if (!done)
        xQueueReset(sRequest); // never sent
    xSemaphoreGive(sMutex);

    // An answer: many intervals of one carrier period in the answer window,
    // clearly more than in the reference window (no answer expected there)
    r.answered = done && ((r.windowRegular >= ANSWER_MIN_REGULAR &&
                           r.windowRegular >= r.referenceRegular + ANSWER_MARGIN) ||
                          (r.windowRun >= ANSWER_RUN_MIN && r.windowRegular >= ANSWER_RUN_MIN_REGULAR &&
                           r.windowRegular >= r.referenceRegular + ANSWER_RUN_MARGIN));
    if (sTraceSearch)
    {
        Serial.printf("Feedback: window %d edges (%u regular, run %u), reference %d edges (%u regular, run %u) -> %s\n",
                      r.window, r.windowRegular, r.windowRun, r.reference, r.referenceRegular, r.referenceRun,
                      !done ? "no result (timeout)" : r.answered ? "answer" : "no answer");
    }
    if (result)
        *result = r;
    return r.answered;
}

/*
 One data request: sends it, waits for the end of the pause, decodes.
 Called under sMutex.
*/
bool Feedback::readOnce(const uint8_t *frame, uint8_t bytes, uint8_t *out)
{
    Request req;
    req.dataBytes = bytes;
    memcpy(req.frame, frame, BUFFER_SIZE);
    xSemaphoreTake(sDataDone, 0); // forget an old end of pause
    sEdgeCount = 0;
    xQueueSend(sRequest, &req, 0);
    // The chunk: frame and synchronisations (~10 ms), the pause (up to
    // 93 ms), plus the end of the chunk being sent
    const uint32_t waitMs = 50 + MfxSignal::dataPauseUs(bytes) / 1000 + 100;
    if (xSemaphoreTake(sDataDone, pdMS_TO_TICKS(waitMs)) != pdPASS)
    {
        xQueueReset(sRequest); // never sent
        sCapture = false;
        if (sTrace)
            Serial.println("Feedback: read CV, no end of pause (timeout)");
        return false;
    }
    const uint16_t n = sEdgeCount;
    const MfxDemod::Status st = sDemod.decode(sEdges, n, (float)getCpuFrequencyMhz(), bytes, out);
    // Every read is traced, with the number of edges captured: tells a
    // comparator that sees nothing from a demodulation problem
    if (sTrace)
        Serial.printf("Feedback: read CV, %u edges, carrier %.2f us, %u runs -> %s\n  bits: %s\n", n,
                      sDemod.periodUs(), sDemod.runs(), MfxDemod::statusText(st), sDemod.bits());
    return st == MfxDemod::OK;
}

bool Feedback::readCV(uint16_t sid, uint16_t cv, uint8_t index, uint8_t bytes, uint8_t *out)
{
    if (!sReady || sEdges == NULL || out == NULL)
        return false;
    if (bytes != 1 && bytes != 2 && bytes != 4 && bytes != 8)
        return false;

    MfxFrame f;
    f.address(sid);
    f.readCV(cv, index, bytes);
    f.finish();
    uint8_t frame[BUFFER_SIZE];
    if (!f.toBuffer(frame, sizeof(frame)))
        return false;

    /*
     Krauss 4.9: answering heats the decoder (it draws about 140 mA during
     the whole answer). The first ESU decoders needed about 120 ms between
     two reads; newer Marklin decoders do not, but the pause is kept for
     every decoder, as a precaution. The MS2 waits about 240 ms before each
     read of an ESU decoder: so do we. 3 tries.
    */
    static TickType_t lastRead = 0;
    bool ok = false;
    xSemaphoreTake(sMutex, portMAX_DELAY);
    sReadSid = sid;
    for (uint8_t i = 0; i < 3 && !ok; i++)
    {
        const TickType_t since = xTaskGetTickCount() - lastRead;
        if (lastRead != 0 && since < pdMS_TO_TICKS(READ_PAUSE_MS))
            vTaskDelay(pdMS_TO_TICKS(READ_PAUSE_MS) - since);
        ok = readOnce(frame, bytes, out);
        lastRead = xTaskGetTickCount();
    }
    xSemaphoreGive(sMutex);

    if (sTrace)
    {
        Serial.printf("  CV %03X/%02X @%u:", cv, index, sid);
        if (ok)
            for (uint8_t i = 0; i < bytes; i++)
                Serial.printf(" %02X", out[i]);
        else
            Serial.print(" no answer");
        Serial.println();
    }
    return ok;
}
