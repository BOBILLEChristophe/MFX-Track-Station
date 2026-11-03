/*

   Message.h


*/

#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "Loco.h"
#include "MfxRMT.h"

//----------------------------------------------------------------------------------------
//  Commandes
//----------------------------------------------------------------------------------------
const byte BUFF_MSG_SIZE = 100;

#define CAN_SYSTEM_CONTROL 0x00
#define CAN_LOCO_SPEED 0x04
#define CAN_LOCO_DIREC 0x05
#define CAN_LOCO_FUNCT 0x06
#define CAN_MFX_BIND 0x02           // assignment of a rail address (UID, SID)
// Period of the command station frame (111 101, UID and counter), in ms.
// A Marklin MS2 with its Gleisbox sends it every 46 ms (recorded on the
// track); it was 500 ms before.
#define STATION_PERIOD_MS 50
#define CAN_LOCO_DISCOVERY 0x01      // search of decoders (Lok Discovery)
#define CAN_MFX_VERIFY 0x03          // existence query (UID, SID)
#define CAN_READ_CONFIG 0x07          // read of CVs (locomotive identifier, index/CV, count)
#define CAN_STATUS_DATA 0x1D        // description of a device (Statusdaten Konfiguration)
#define CAN_PING 0x18               // who is on the bus? (answered as a CS2)
#define CAN_CONFIG_DATA 0x20        // request of a configuration file ("loks"...)
#define CAN_CONFIG_DATA_STREAM 0x21 // the file

// Defined in main.cpp: sends bytes (whole 13-byte frames) to Rocrail.
// Returns false if Rocrail is not connected. Safe from several tasks.
bool sendToRocrail(const uint8_t *buf, size_t len);

class Message
{
private:
    static uint32_t id;
    static uint8_t prio;
    static uint8_t cmd;
    static uint8_t response;
    static uint16_t hash;
    static uint8_t length;
    static uint8_t data[8];
    static bool trace;         // Debug: internal tasks (very verbose)
    static bool traceCommands; // Commands received from Rocrail
    static bool traceBus;      // Frames received from the CAN bus
    static bool answerMs2Conf; // "ms2-conf" answered by the command station
    static bool simulateGui;   // ping answer of a CS2 GUI for an MS2
    static byte buff[BUFF_MSG_SIZE];
    static Loco **loco;
    static volatile byte locoSize; // locomotives in loco[]
    static byte locoCapacity;      // size of loco[]

public:
    // Test (serial command Q): the periodic refresh of the locomotives is
    // stopped, only the frame of the command station and the requests go
    // to the track
    static volatile bool quiet;
    // locoArray: room for capacity locomotives, the first count are filled
    static void setup(Loco *locoArray[], byte capacity, byte count);
    // Adds a locomotive to the refresh (drive frames and Bind).
    // Returns false if the array is full.
    static bool addLoco(Loco *l);
    // Removes the locomotive at this address from the refresh. Returns false
    // if it was not there.
    static bool removeLoco(uint16_t address);
    // Prints each command received from Rocrail (speed, direction, functions)
    static void setTrace(bool on) { traceCommands = on; }
    // Origin of the frame being parsed: true for the CAN bus (an MS2...),
    // false for Rocrail (TCP). Set by rxTask before parse().
    static bool fromBus;
    // Trace of the frames received from the CAN bus (pings left out)
    static void setBusTrace(bool on) { traceBus = on; }
    static bool busTrace() { return traceBus; }
    // Test: the command station answers itself the request "ms2-conf" of an
    // MS2 (format of can2udp, G. Bertelsmann), instead of Rocrail
    static void setAnswerMs2Conf(bool on) { answerMs2Conf = on; }
    static bool answersMs2Conf() { return answerMs2Conf; }
    // Test: when an MS2 answers a ping, a ping answer of a CS2 GUI (type
    // FFFF) is sent on the bus, as can2udp -g does to make it a CS2 slave
    static void setSimulateGui(bool on) { simulateGui = on; }
    static bool simulatesGui() { return simulateGui; }
    static void decodeMsg(byte *);
    static void parse();
    static void periodic(void *);
    static void driveFrame(Loco *l, byte *out, size_t outSize);
    // fn >= 16: the function fn is sent alone (MfxFrame::function()),
    // else the drive frame (speed, direction, F0 to F15)
    static void sendNow(Loco *l, int fn = -1);
    // One function of a locomotive (F16 and beyond), in the format of mfxQueue
    static void functionFrame(Loco *l, uint8_t fn, byte *out, size_t outSize);
    // The refresh repeats one of the functions F16 to F31 of a locomotive
    // every HIGH_REFRESH_CYCLES cycles (50 ms each), in turn
    static const uint8_t HIGH_REFRESH_CYCLES = 4;
    static void centrale(void *);


    // Sends to Rocrail an answer of this device (response bit set, hash of
    // its UID): command cmd, len bytes of d.
    static bool answer(uint8_t cmd, uint8_t len, const uint8_t *d);

private:
    static void configRequest();
    static void pingAnswer();
    // As a Gleisbox: confirms an order to a locomotive, answers a request
    static void locoAnswer(Loco *l);
    // As a Gleisbox: answers a system command (echo, or the identifier)
    static void systemAnswer();
    // As a Gleisbox: the description of the device (command 0x1D, index 0)
    static void statusDataAnswer();
    static void sendLocoList();
    static void sendMs2Conf();
    static bool busConfigRequest();
    static void traceUnknown();
};
