/*

   ConfigStore.cpp

   See ConfigStore.h.

*/

#include "ConfigStore.h"
#include <LittleFS.h>

namespace ConfigStore
{
    static const char *DIR = "/config";
    static bool sMounted = false;
    static SemaphoreHandle_t sMutex = nullptr;

    struct Lock
    {
        Lock() { xSemaphoreTake(sMutex, portMAX_DELAY); }
        ~Lock() { xSemaphoreGive(sMutex); }
    };

    static String pathOf(const char *name, const char *ext = ".json")
    {
        return String(DIR) + "/" + name + ext;
    }

    // Only letters, digits, '-' and '_': names come from web requests too
    static bool validName(const char *name)
    {
        if (name == nullptr || *name == 0 || strlen(name) > 24)
            return false;
        for (const char *p = name; *p; p++)
            if (!isalnum((unsigned char)*p) && *p != '-' && *p != '_')
                return false;
        return true;
    }

    bool setup()
    {
        if (sMutex == nullptr)
            sMutex = xSemaphoreCreateMutex();
        // Not formatted on purpose when it fails: first try to mount
        if (!LittleFS.begin(false))
        {
            Serial.println("LittleFS: no file system yet, formatting (first start)");
            if (!LittleFS.format() || !LittleFS.begin(false))
            {
                Serial.println("LittleFS: not available, settings not saved");
                return false;
            }
        }
        if (!LittleFS.exists(DIR))
            LittleFS.mkdir(DIR);
        sMounted = true;
        Serial.printf("LittleFS: %u of %u bytes used\n", (unsigned)LittleFS.usedBytes(),
                      (unsigned)LittleFS.totalBytes());
        return true;
    }

    bool available()
    {
        return sMounted;
    }

    bool load(const char *name, JsonDocument &doc)
    {
        if (!sMounted || !validName(name))
            return false;
        Lock lock;
        File f = LittleFS.open(pathOf(name), "r");
        if (!f)
            f = LittleFS.open(pathOf(name, ".tmp"), "r"); // reset between remove and rename
        if (!f)
            return false;
        const DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err)
        {
            Serial.printf("LittleFS: %s unreadable (%s), default values\n", pathOf(name).c_str(), err.c_str());
            return false;
        }
        return true;
    }

    bool save(const char *name, const JsonDocument &doc)
    {
        if (!sMounted || !validName(name))
            return false;
        Lock lock;
        const String tmp = pathOf(name, ".tmp");
        File f = LittleFS.open(tmp, "w");
        if (!f)
            return false;
        const size_t n = serializeJsonPretty(doc, f);
        f.close();
        if (n == 0)
        {
            LittleFS.remove(tmp);
            return false;
        }
        const String path = pathOf(name);
        LittleFS.remove(path);
        if (!LittleFS.rename(tmp, path))
        {
            Serial.printf("LittleFS: %s not written\n", path.c_str());
            return false;
        }
        return true;
    }

    String raw(const char *name)
    {
        if (!sMounted || !validName(name))
            return String();
        Lock lock;
        File f = LittleFS.open(pathOf(name), "r");
        if (!f)
            return String();
        String s = f.readString();
        f.close();
        return s;
    }

    void list(JsonObject out)
    {
        if (!sMounted)
            return;
        Lock lock;
        File dir = LittleFS.open(DIR);
        if (!dir)
            return;
        for (File f = dir.openNextFile(); f; f = dir.openNextFile())
        {
            String n = f.name();
            if (n.endsWith(".json"))
                out[n.substring(0, n.length() - 5)] = (uint32_t)f.size();
            f.close();
        }
        dir.close();
    }

    size_t usedBytes()
    {
        return sMounted ? LittleFS.usedBytes() : 0;
    }

    size_t totalBytes()
    {
        return sMounted ? LittleFS.totalBytes() : 0;
    }
}
