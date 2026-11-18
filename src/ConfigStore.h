/*

   ConfigStore.h

   Configuration files of the command station, in LittleFS (partition
   "spiffs" of partitions.csv, 768 KB), one JSON file per subject:

     /config/l88.json   Link S88 (LinkS88.h)
     ...                one file per new subject (web pages, WebUi.h)

   Each file holds a "version" number, so that a later version of the
   program can read an older file. A file is written to a temporary file
   first, then renamed: a reset during the write leaves the old file.

   The NVS (Preferences) keeps what it held before (locomotives, mfx
   counter...): only the new settings go to LittleFS.

   Several tasks may read and write (console, web server): one mutex.

*/

#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

namespace ConfigStore
{
    // Mounts LittleFS (formats it on the first start, when it cannot be
    // mounted). Returns false if it is not available: the settings then
    // keep their default values and are not saved.
    bool setup();
    bool available();

    // Reads /config/<name>.json into doc. False if absent or unreadable.
    bool load(const char *name, JsonDocument &doc);

    // Writes doc to /config/<name>.json (temporary file, then rename)
    bool save(const char *name, const JsonDocument &doc);

    // Raw content of /config/<name>.json (empty if absent): download, backup
    String raw(const char *name);

    // Names of the files of /config, with their size: {"l88": 52, ...}
    void list(JsonObject out);

    // Space used and total, in bytes
    size_t usedBytes();
    size_t totalBytes();
}
