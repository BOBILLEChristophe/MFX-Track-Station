/*

  Message.cpp


*/

#include "Message.h"
#include "CanBus.h"
#include "Centrale.h"
#include "MfxFrame.h"
#include "Cs2Config.h"
#include "Discovery.h"

Loco **Message::loco = nullptr;
volatile byte Message::locoSize = 0;
byte Message::locoCapacity = 0;

bool Message::trace = false;
bool Message::traceCommands = false;

uint32_t Message::id = 0;
uint8_t Message::length = 0;
uint8_t Message::prio = 0;
uint8_t Message::cmd = 0;
uint8_t Message::response = 0;
uint16_t Message::hash = 0;
bool Message::fromBus = false;
bool Message::traceBus = true;
bool Message::answerMs2Conf = true;
uint8_t Message::data[8] = {0};
byte Message::buff[BUFF_MSG_SIZE] = {0};

SemaphoreHandle_t taskSemaphore;

void Message::setup(Loco *locoArray[], byte capacity, byte count)
{
    loco = locoArray;
    locoCapacity = capacity;
    locoSize = count < capacity ? count : capacity;
    trace = false;

    taskSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(taskSemaphore); // Libère le sémaphore pour la première tâche

    xTaskCreatePinnedToCore(periodic, "Periodic", 2 * 1024, (void *)loco, 6, NULL, 0);
    xTaskCreatePinnedToCore(centrale, "Centrale", 2 * 1024, (void *)loco, 5, NULL, 0);
}

/*
 Called by the decoder discovery (Discovery.cpp). The periodic tasks read
 loco[] while they hold taskSemaphore: the new entry is written before the
 count, under the same semaphore.
*/
bool Message::addLoco(Loco *l)
{
    bool added = false;
    if (xSemaphoreTake(taskSemaphore, portMAX_DELAY) == pdTRUE)
    {
        if (locoSize < locoCapacity)
        {
            loco[locoSize] = l;
            locoSize = locoSize + 1;
            added = true;
        }
        xSemaphoreGive(taskSemaphore);
    }
    return added;
}

bool Message::removeLoco(uint16_t address)
{
    bool removed = false;
    if (xSemaphoreTake(taskSemaphore, portMAX_DELAY) == pdTRUE)
    {
        for (byte i = 0; i < locoSize; i++)
            if (loco[i] != nullptr && loco[i]->addr() == address)
            {
                for (byte j = i; j + 1 < locoSize; j++)
                    loco[j] = loco[j + 1];
                locoSize = locoSize - 1;
                loco[locoSize] = nullptr; // a task may still read this slot
                removed = true;
                break;
            }
        xSemaphoreGive(taskSemaphore);
    }
    return removed;
}

/*
 Drive + functions F0..F15 of a locomotive (Krauss 3.1), in the format of
 mfxQueue. Used by the periodic refresh and by sendNow().
*/
void Message::driveFrame(Loco *l, byte *out, size_t outSize)
{
    uint16_t functions = 0;
    for (byte n = 0; n < 16; n++)
        if (l->funct(n))
            functions |= (uint16_t)(1u << n);
    MfxFrame frame;
    frame.address(l->addr());
    frame.drive(l->dir() != 0, l->speed());
    frame.functions16(functions);
    frame.finish();
    frame.toBuffer(out, outSize);
}

/*
 A command from Rocrail is sent at once, at the head of the queue, instead
 of waiting for the next refresh (up to 50 ms). The refresh repeats it.
*/
void Message::sendNow(Loco *l)
{
    byte frame[BUFF_MSG_SIZE];
    driveFrame(l, frame, sizeof(frame));
    xQueueSendToFront(mfxQueue, frame, 0);
}

void Message::periodic(void *p)
{
    Loco **loco;
    loco = (Loco **)p;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    // byte buff[BUFF_MSG_SIZE] = {0};

    for (;;)
    {
        for (byte i = 0; i < locoSize; i++)
        {
            if (xSemaphoreTake(taskSemaphore, portMAX_DELAY) == pdTRUE)
            {
                if (loco[i] != nullptr)
                {
                    if (trace)
                        Serial.printf("Periodic- %s\n", loco[i]->name().c_str());

                    driveFrame(loco[i], buff, sizeof(buff));

                    xQueueSend(mfxQueue, &buff, 0);
                }
                vTaskDelay(pdMS_TO_TICKS(5));
            }
            xSemaphoreGive(taskSemaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void Message::centrale(void *p)
{
    /*
 3.2.6 Commande 111 101 : centrale
 111 101 UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU ZZZZZZZZZZZZZZZZ
 La centrale envoie cette commande à intervalles réguliers avec l'adresse de diffusion (broadcast) 0 indiquant ainsi son UID (U) et le compteur de nouvelles inscriptions (Z).
 */

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (xSemaphoreTake(taskSemaphore, portMAX_DELAY) == pdTRUE)
        {
            if (trace)
                Serial.println("Centrale");

            // Broadcast: command station UID and re-registration counter,
            // every STATION_PERIOD_MS (a Marklin MS2 sends it every 46 ms).
            // The counter only changes on explicit request: a decoder that
            // sees it change registers again.
            MfxFrame frame;
            frame.broadcast();
            frame.centrale(Centrale::gUid(), Centrale::counter());
            frame.finish();
            frame.toBuffer(buff, sizeof(buff));
            xQueueSend(mfxQueue, &buff, 0);
            xSemaphoreGive(taskSemaphore);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(STATION_PERIOD_MS));
    }
}


void Message::decodeMsg(byte *rxBuffer)
{
    id = (uint32_t)rxBuffer[0] << 24 | (uint32_t)rxBuffer[1] << 16 | (uint32_t)rxBuffer[2] << 8 | rxBuffer[3];

    length = rxBuffer[4];
    if (length > 8)
        length = 8;
    prio = (id & 0x1E000000) >> 25;
    cmd = (id & 0x1FE0000) >> 17;
    response = (id & 0x10000) >> 16;
    hash = id & 0xFFFF;

    // On efface data[] : sinon une trame courte (une requête) garde les
    // octets de la trame précédente, qui sont alors pris pour un ordre.
    memset(data, 0, sizeof(data));
    for (byte i = 0; i < length; i++)
        data[i] = rxBuffer[i + 5];
}

/*
 Les trames CS2 se distinguent par leur longueur :
   vitesse   : 4 octets = requête, 6 octets = ordre
   direction : 4 octets = requête, 5 octets = ordre
   fonction  : 5 octets = requête, 6 octets = ordre
 Les requêtes sont ignorées pour l'instant (les réponses à Rocrail viendront
 à l'étape 3) : surtout, elles ne modifient plus l'état d'une loco.
*/
void Message::parse()
{
    if (response) // écho ou réponse d'un autre appareil : rien à exécuter
        return;

    if (cmd == CAN_CONFIG_DATA)
    {
        // A Gleisbox does not answer: a device of the bus (an MS2) asks
        // Rocrail, which gets the request through the command station
        if (fromBus)
        {
            char name[9];
            memcpy(name, data, 8);
            name[8] = 0;
            if (answerMs2Conf && strcmp(name, "ms2-conf") == 0)
            {
                sendMs2Conf();
                return;
            }
            Serial.printf("CAN: a device asks Rocrail for the configuration \"%s\"\n", name);
            return;
        }
        configRequest();
        return;
    }

    if (cmd == CAN_PING)
    {
        pingAnswer();
        return;
    }

    if (cmd == CAN_SYSTEM_CONTROL)
    {
        if (length < 5)
            return;
        if (traceCommands)
        {
            // Every byte, to compare with the MS2 captures (sub-command 0x20 =
            // system time, sent every minute by Rocrail, is left out)
            if (data[4] != 0x20)
            {
                Serial.printf("Rocrail: system 0x%02X (%u bytes:", data[4], length);
                for (byte i = 0; i < length; i++)
                    Serial.printf(" %02X", data[i]);
                Serial.println(")");
            }
        }
        switch (data[4]) // sous-commande
        {
        case 0x00: // Arrêt du système
            Centrale::setPower(false);
            break;
        case 0x01: // Démarrage du système (seul moyen de sortir d'une surcharge)
            Centrale::setPower(true);
            break;
        case 0x02: // System Halt : à implémenter (étape 3)
            break;
        case 0x03: // Arrêt d'urgence de la locomotive : à implémenter (étape 3)
            break;
        case 0x09: // re-registration counter, set by the master (MS2: at power on)
            if (length >= 7)
            {
                const uint16_t counter = (uint16_t)data[5] << 8 | data[6];
                if (counter != Centrale::counter())
                {
                    Centrale::setCounter(counter);
                    Serial.printf("Rocrail: re-registration counter %u (from the master)\n", counter);
                }
            }
            break;
        }
        systemAnswer();
        return;
    }

    if (cmd == CAN_STATUS_DATA)
    {
        statusDataAnswer();
        return;
    }

    // Read of CVs ordered by the master: identifier, index << 10 | CV, count
    if (cmd == CAN_READ_CONFIG && length >= 7)
    {
        const uint32_t locId = (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 | (uint32_t)data[2] << 8 | data[3];
        const uint16_t ic = (uint16_t)data[4] << 8 | data[5];
        Serial.printf("Rocrail: read CV %u index %u, %u bytes, identifier %04lX\n", ic & 0x3FF, ic >> 10, data[6],
                      (unsigned long)locId);
        if (locId >= 0x4000 && locId < 0x8000 && data[6] > 0)
            Discovery::requestRead(locId, ic & 0x3FF, (uint8_t)(ic >> 10), data[6]);
        return;
    }

    // Orders of the master for the registration of the decoders (a Gleisbox
    // executes them): bind and verify, UID then SID
    if ((cmd == CAN_MFX_BIND || cmd == CAN_MFX_VERIFY) && length >= 6)
    {
        const uint32_t uid = (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 | (uint32_t)data[2] << 8 | data[3];
        const uint16_t sid = (uint16_t)data[4] << 8 | data[5];
        Serial.printf("Rocrail: %s UID %08lX SID %u\n", cmd == CAN_MFX_BIND ? "bind" : "verify",
                      (unsigned long)uid, sid);
        if (cmd == CAN_MFX_BIND)
            Discovery::requestBind(uid, sid);
        else
            Discovery::requestVerify(uid, sid);
        return;
    }

    if (cmd < CAN_LOCO_SPEED || cmd > CAN_LOCO_FUNCT || length < 4)
    {
        if (!(cmd == 0x21 && traceBus)) // config streams: printed by rxTask
            traceUnknown();
        return;
    }

    // CS2 locomotive identifier: mfx from 0x4000 (0x4000 + SID), else the
    // low byte as before (MM2)
    const uint16_t locId = (uint16_t)data[2] << 8 | data[3];
    const uint16_t address = (locId >= 0x4000 && locId < 0x8000) ? locId - 0x4000 : data[3];
    Loco *currentLoco = Loco::findLoco(address);
    if (currentLoco == nullptr)
    {
        // As a Gleisbox: every order of the master goes to the track, even
        // for a locomotive bound in another session (its decoder kept its
        // address). The locomotive is added to the refresh, UID unknown.
        if (locId >= 0x4000 && locId < 0x8000 && address != 0)
        {
            currentLoco = Loco::createLoco(address, "mfx " + String(address), 0, 0, 0, 0);
            if (!addLoco(currentLoco))
            {
                Serial.printf("Loco %u : trop de locomotives (MAX_LOCOS), commande ignoree\n", address);
                return;
            }
            Serial.printf("Loco %u ajoutee (ordre du maitre, UID inconnu)\n", address);
        }
        else
        {
            Serial.printf("Loco inconnue : identifiant %04X, commande ignoree\n", locId);
            return;
        }
    }

    switch (cmd)
    {
    case CAN_LOCO_SPEED:
        if (length >= 6)
        {
            uint16_t tempSpeed = (data[4] << 8) | data[5];
            if (tempSpeed > 1000)
                tempSpeed = 1000;
            tempSpeed = map(tempSpeed, 0, 1000, 0, 127);
            if (tempSpeed == 1) // 1 = arrêt d'urgence en MFX
                tempSpeed = 0;
            currentLoco->speed(tempSpeed);
        }
        break;

    case CAN_LOCO_DIREC:
        if (length >= 5)
        {
            // CS2 : 0 = garder, 1 = avant, 2 = arrière, 3 = inverser
            const uint8_t before = currentLoco->dir();
            switch (data[4])
            {
            case 1:
                currentLoco->dir(0);
                break;
            case 2:
                currentLoco->dir(1);
                break;
            case 3:
                currentLoco->toggleDir();
                break;
            }
            // Comme une CS2 : un changement de direction arrête la loco.
            if (currentLoco->dir() != before)
                currentLoco->speed(0);
        }
        break;

    case CAN_LOCO_FUNCT:
        if (length >= 6)
            currentLoco->funct(data[4], data[5] != 0);
        break;
    }

    // An order (not a request) changed the locomotive: send it now
    const bool order = (cmd == CAN_LOCO_SPEED && length >= 6) ||
                       (cmd == CAN_LOCO_DIREC && length >= 5) ||
                       (cmd == CAN_LOCO_FUNCT && length >= 6);
    if (order)
        sendNow(currentLoco);
    locoAnswer(currentLoco);

    if (traceCommands)
    {
        Serial.printf("cmd %u, loco %s (adresse %u) : vitesse %u, direction %u\n",
                      cmd, currentLoco->name().c_str(), currentLoco->addr(),
                      currentLoco->speed(), currentLoco->dir());
    }
}

/*
 A command not handled: printed once per command number (traceCommands),
 to see what Rocrail asks.
*/
void Message::traceUnknown()
{
    // The first 5 of each command (Rocrail as master: discovery, bind...)
    static uint8_t seen[256] = {0};
    if (!traceCommands || seen[cmd] >= 5)
        return;
    seen[cmd]++;
    Serial.printf("Rocrail: command 0x%02X not handled (%u bytes:", cmd, length);
    for (byte i = 0; i < length; i++)
        Serial.printf(" %02X", data[i]);
    Serial.printf(")%s\n", seen[cmd] == 5 ? ", printed 5 times" : "");
}

// One 13-byte CS2 frame: id (big endian), length, 8 data bytes
static void putFrame(uint8_t *f, uint32_t id, uint8_t len, const uint8_t *d)
{
    f[0] = (uint8_t)(id >> 24);
    f[1] = (uint8_t)(id >> 16);
    f[2] = (uint8_t)(id >> 8);
    f[3] = (uint8_t)id;
    f[4] = len;
    memset(f + 5, 0, 8);
    if (len > 8)
        len = 8;
    memcpy(f + 5, d, len);
}

/*
 Hash of the CS2 protocol for a UID: it replaces the UID in the CAN
 identifier (bits 7 to 9 at 110, as in the Marklin documentation).
*/
static uint16_t cs2Hash(uint32_t uid)
{
    uint16_t h = (uint16_t)((uid >> 16) ^ (uid & 0xFFFF));
    return (uint16_t)(((h << 3) & 0xFC00) | 0x0300 | (h & 0x007F));
}

/*
 Answers of a Gleisbox to the system commands (recorded on the CAN bus of an
 MS2 and a Gleisbox, 31 Oct): the same bytes with the response bit, for STOP,
 GO, track protocols (0x08), re-registration counter (0x09)... The identifier
 (0x0C, "Kennung", asked by Rocrail as master) is answered with 7 bytes:
 UID, 0x0C, identifier. Only the commands for this device or for all
 (UID 0) are answered.
*/
bool Message::answer(uint8_t cmd, uint8_t len, const uint8_t *d)
{
    uint8_t frame[13];
    putFrame(frame, ((uint32_t)cmd << 17) | 0x00010000UL | cs2Hash(Centrale::gUid()), len, d);
    return sendToRocrail(frame, sizeof(frame));
}

/*
 As a Gleisbox: an order to a locomotive (speed, direction, function) is
 confirmed with the same bytes and the response bit; a request (shorter) is
 answered with the state of the locomotive. An MS2 on the bus and Rocrail
 both see the result.
*/
void Message::locoAnswer(Loco *l)
{
    uint8_t d[8];
    memcpy(d, data, 8);
    uint8_t len = length;
    switch (cmd)
    {
    case CAN_LOCO_SPEED:
        if (length < 6)
        {
            const uint16_t v = l->speed() == 0 ? 0 : (uint16_t)(((uint32_t)l->speed() * 1000 + 63) / 127);
            d[4] = (uint8_t)(v >> 8);
            d[5] = (uint8_t)v;
            len = 6;
        }
        break;
    case CAN_LOCO_DIREC:
        if (length < 5)
        {
            d[4] = l->dir() == 0 ? 1 : 2;
            len = 5;
        }
        break;
    case CAN_LOCO_FUNCT:
        if (length == 5)
        {
            d[5] = l->funct(data[4]) ? 1 : 0;
            len = 6;
        }
        else if (length < 5)
            return;
        break;
    default:
        return;
    }
    answer(cmd, len, d);
}

static const uint16_t KENNUNG = 0x0001; // 0 and FFFF are not valid

void Message::systemAnswer()
{
    const uint32_t own = Centrale::gUid();
    const uint32_t to = (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 | (uint32_t)data[2] << 8 | data[3];
    if (to != 0 && to != own)
        return;
    uint8_t d[8];
    memcpy(d, data, 8);
    uint8_t len = length;
    switch (data[4])
    {
    case 0x00: // STOP
    case 0x01: // GO
    case 0x02: // halt
    case 0x08: // track protocols
    case 0x09: // re-registration counter
        break;
    case 0x0C: // identifier: asked (5 bytes) -> answered with it (7 bytes)
        d[5] = KENNUNG >> 8;
        d[6] = KENNUNG & 0xFF;
        len = 7;
        break;
    default:
        return; // not answered (yet)
    }
    uint8_t frame[13];
    putFrame(frame, ((uint32_t)CAN_SYSTEM_CONTROL << 17) | 0x00010000UL | cs2Hash(own), len, d);
    sendToRocrail(frame, sizeof(frame));
}

/*
 Command 0x1D (Statusdaten Konfiguration), index 0: the description of the
 device, sent in packets of 8 bytes (the hash field of each one is its
 number, from 1), then a frame of 6 bytes: UID, index, number of packets
 (CS2 CAN protocol 2.0, 6.2). Content: number of measurement channels,
 number of configuration channels, 2 free bytes, serial number (4 bytes),
 article number (8 characters), name (ended by 0). No channel: Rocrail
 asks no more.
*/
void Message::statusDataAnswer()
{
    const uint32_t own = Centrale::gUid();
    const uint32_t to = (uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 | (uint32_t)data[2] << 8 | data[3];
    if (length < 5 || to != own)
        return;
    const uint8_t index = data[4];
    if (index != 0)
        return; // no channel
    uint8_t desc[32];
    memset(desc, 0, sizeof(desc));
    desc[0] = 0; // measurement channels
    desc[1] = 0; // configuration channels
    desc[4] = (uint8_t)(own >> 24);
    desc[5] = (uint8_t)(own >> 16);
    desc[6] = (uint8_t)(own >> 8);
    desc[7] = (uint8_t)own;
    memcpy(desc + 8, "60116\0\0\0", 8);
    const char *name = "MFX Station"; // 16 bytes at most, with the final 0
    strncpy((char *)desc + 16, name, 15);
    const uint8_t packets = 4; // 32 bytes
    uint8_t frames[13 * 5];
    for (uint8_t p = 0; p < packets; p++)
        putFrame(frames + 13 * p, ((uint32_t)CAN_STATUS_DATA << 17) | 0x00010000UL | (uint16_t)(p + 1), 8,
                 desc + 8 * p);
    const uint8_t end[6] = {(uint8_t)(own >> 24), (uint8_t)(own >> 16), (uint8_t)(own >> 8), (uint8_t)own, index,
                            packets};
    putFrame(frames + 13 * packets, ((uint32_t)CAN_STATUS_DATA << 17) | 0x00010000UL | cs2Hash(own), 6, end);
    const bool ok = sendToRocrail(frames, sizeof(frames));
    Serial.printf("Rocrail: device description sent (index 0, %u packets)%s\n", packets, ok ? "" : ", not sent");
}

/*
 Command 0x18 (ping): every device of the bus answers with its UID, its
 software version and its type. The command station answers as the
 master of a CS2 (type 0xFFFF, version 3.8, as the CS2 GUI emulated in
 the railroad project): Rocrail then deals with it as with a CS2.
*/
void Message::pingAnswer()
{
    static bool told = false;
    const uint32_t uid = Centrale::gUid();
    const uint8_t d[8] = {(uint8_t)(uid >> 24), (uint8_t)(uid >> 16), (uint8_t)(uid >> 8), (uint8_t)uid,
                          1, 0x2F, 0x00, 0x11}; // as a Gleisbox: version 1.47, type 0x0011 (recorded)
    uint8_t frame[13];
    putFrame(frame, ((uint32_t)CAN_PING << 17) | 0x00010000UL | cs2Hash(uid), 8, d);
    const bool ok = sendToRocrail(frame, sizeof(frame));
    if (!told)
    {
        Serial.printf("Rocrail: ping answered as a Gleisbox (UID %08lX, version 1.47, type 0011)%s\n", (unsigned long)uid, ok ? "" : ", not sent");
        told = true;
    }
}


/*
 Command 0x20: Rocrail asks for a configuration file, by its name in the 8
 data bytes. Only "loks" (the loco list) is answered; as a CS2, the
 request is first sent back with the response bit.
*/
void Message::configRequest()
{
    char name[9];
    memcpy(name, data, 8);
    name[8] = 0;
    Serial.printf("Rocrail asks for the configuration \"%s\"\n", name);

    uint8_t frame[13];
    putFrame(frame, id | 0x00010000UL, length, data);
    sendToRocrail(frame, sizeof(frame));

    if (strcmp(name, "loks") == 0)
        sendLocoList();
}

/*
 Answer to "ms2-conf" (an MS2 in slave mode asks it to its master), as
 can2udp (ms2-loco-list.c, G. Bertelsmann) does: the request with the
 response bit, then the plain text in a config data stream with the hash of
 the MS2. Length without the padding, CRC with it. Sent on the bus only.
 Rocrail answers ".magpro=0 .lokpro=7", length with the padding: the MS2
 asks again and again.
*/
void Message::sendMs2Conf()
{
    static const char text[] = "[config]\n .magprot=0\n .lokprot=7\n";
    const size_t len = sizeof(text) - 1;
    const size_t padded = (len + 7) / 8 * 8;
    uint8_t data[48];
    memset(data, 0, sizeof(data));
    memcpy(data, text, len);
    const uint16_t crc = Cs2Config::crcCcitt(data, padded, 0xFFFF);

    uint8_t frame[13];
    putFrame(frame, ((uint32_t)CAN_CONFIG_DATA << 17) | 0x00010000UL | cs2Hash(Centrale::gUid()), 8, Message::data);
    CanBus::send(frame, 13);
    const uint32_t streamId = ((uint32_t)CAN_CONFIG_DATA_STREAM << 17) | hash;
    const uint8_t head[6] = {0, 0, 0, (uint8_t)len, (uint8_t)(crc >> 8), (uint8_t)crc};
    putFrame(frame, streamId, 6, head);
    CanBus::send(frame, 13);
    for (size_t i = 0; i < padded; i += 8)
    {
        vTaskDelay(pdMS_TO_TICKS(2));
        putFrame(frame, streamId, 8, data + i);
        CanBus::send(frame, 13);
    }
    Serial.printf("CAN: \"ms2-conf\" answered by the command station (%u bytes, CRC %04X)\n", (unsigned)len, crc);
}

/*
 The loco list as the file lokomotive.cs2 of a CS2, in a config data
 stream (Cs2Config.h): a frame of 6 bytes (length, CRC), then the data in
 frames of 8 bytes, with the hash of the request.
*/
void Message::sendLocoList()
{
    static const uint8_t MAX = 32;
    Loco *copies = new Loco[MAX];
    Cs2Config::Loco *entries = new Cs2Config::Loco[MAX];
    String *names = new String[MAX];
    const uint8_t n = Loco::list(copies, MAX);
    for (uint8_t i = 0; i < n; i++)
    {
        names[i] = copies[i].name();
        Cs2Config::Loco &e = entries[i];
        memset(&e, 0, sizeof(e));
        e.name = names[i].c_str();
        e.sid = copies[i].addr();
        e.mfxUid = copies[i].uid32();
        const MfxInfo info = copies[i].mfxInfo();
        e.haveConfig = info.valid;
        if (info.valid)
        {
            e.av = info.av;
            e.bv = info.bv;
            e.vmin = info.vmin;
            e.vmax = info.vmax;
            e.tachoMax = info.tachoMax;
            e.functions = info.functions;
            memcpy(e.fnType, info.fnType, sizeof(e.fnType));
        }
    }

    const size_t textRoom = 200 + (size_t)n * (300 + Cs2Config::FUNCTIONS * 40);
    char *text = (char *)malloc(textRoom);
    size_t textLen = text ? Cs2Config::locoFile(entries, n, text, textRoom) : 0;
    const size_t room = Cs2Config::streamSize(textLen);
    uint8_t *stream = textLen ? (uint8_t *)malloc(room) : nullptr;
    uint32_t streamLen = 0;
    uint16_t crc = 0;
    const size_t padded = stream ? Cs2Config::stream((const uint8_t *)text, textLen, stream, room, &streamLen, &crc) : 0;

    if (padded)
    {
        const uint32_t streamId = (id & 0xFE00FFFFUL) | ((uint32_t)CAN_CONFIG_DATA_STREAM << 17);
        uint8_t head[6] = {(uint8_t)(streamLen >> 24), (uint8_t)(streamLen >> 16), (uint8_t)(streamLen >> 8),
                           (uint8_t)streamLen, (uint8_t)(crc >> 8), (uint8_t)crc};
        uint8_t frames[13 * 16];
        putFrame(frames, streamId, 6, head);
        bool ok = sendToRocrail(frames, 13);
        for (size_t i = 0; ok && i < padded;)
        {
            size_t k = 0;
            for (; k < 16 && i < padded; k++, i += 8)
                putFrame(frames + 13 * k, streamId, 8, stream + i);
            ok = sendToRocrail(frames, 13 * k);
        }
        Serial.printf("Loco list sent to Rocrail: %u locomotives, %u bytes of text, %u frames%s\n", n,
                      (unsigned)textLen, (unsigned)(1 + padded / 8), ok ? "" : " (not connected)");
    }
    else
        Serial.println("Loco list: out of memory, not sent");

    free(stream);
    free(text);
    delete[] names;
    delete[] entries;
    delete[] copies;
}
