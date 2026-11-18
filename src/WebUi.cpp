/*

   WebUi.cpp

   See WebUi.h.

*/

#include "WebUi.h"
#include "WebPage.h"
#include "ConfigStore.h"
#include "LinkS88.h"
#include <WebServer.h>
#include <ArduinoJson.h>

namespace WebUi
{
    static WebServer sServer(80);

    static void sendJson(int code, const JsonDocument &doc)
    {
        String body;
        serializeJson(doc, body);
        sServer.send(code, "application/json", body);
    }

    static void sendError(int code, const char *message)
    {
        JsonDocument doc;
        doc["error"] = message;
        sendJson(code, doc);
    }

    static void getL88()
    {
        JsonDocument doc;
        LinkS88::status(doc.to<JsonObject>());
        sendJson(200, doc);
    }

    static void postL88()
    {
        JsonDocument in;
        if (deserializeJson(in, sServer.arg("plain")))
            return sendError(400, "JSON invalide");
        JsonArrayConst bus = in["bus"];
        if (bus.size() != 3)
            return sendError(400, "bus : 3 nombres attendus (bus 1, 2, 3)");
        int v[3];
        for (uint8_t b = 0; b < 3; b++)
        {
            if (!bus[b].is<int>() || bus[b].as<int>() < 0 || bus[b].as<int>() > 31)
                return sendError(400, "nombre de modules : 0 à 31");
            v[b] = bus[b].as<int>();
        }
        const int cycle = in["cycleMs"] | 100;
        if (cycle < 10 || cycle > 1000)
            return sendError(400, "temps de cycle : 10 à 1000 ms");
        const bool saved = LinkS88::configure((uint8_t)v[0], (uint8_t)v[1], (uint8_t)v[2], (uint16_t)cycle);
        JsonDocument doc;
        JsonObject out = doc.to<JsonObject>();
        LinkS88::status(out);
        out["saved"] = saved;
        sendJson(200, doc);
    }

    static void getFiles()
    {
        JsonDocument doc;
        doc["available"] = ConfigStore::available();
        doc["used"] = (uint32_t)ConfigStore::usedBytes();
        doc["total"] = (uint32_t)ConfigStore::totalBytes();
        ConfigStore::list(doc["files"].to<JsonObject>());
        sendJson(200, doc);
    }

    static void getConfig()
    {
        const String name = sServer.arg("name");
        const String body = ConfigStore::raw(name.c_str());
        if (body.length() == 0)
            return sendError(404, "fichier absent");
        sServer.sendHeader("Content-Disposition", "attachment; filename=\"" + name + ".json\"");
        sServer.send(200, "application/json", body);
    }

    static void task(void *)
    {
        while (true)
        {
            sServer.handleClient();
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }

    void setup()
    {
        sServer.on("/", HTTP_GET, [] { sServer.send_P(200, "text/html; charset=utf-8", WEB_PAGE); });
        sServer.on("/api/l88", HTTP_GET, getL88);
        sServer.on("/api/l88", HTTP_POST, postL88);
        sServer.on("/api/files", HTTP_GET, getFiles);
        sServer.on("/api/config", HTTP_GET, getConfig);
        sServer.onNotFound([] { sendError(404, "introuvable"); });
        sServer.begin();
        xTaskCreatePinnedToCore(task, "Web", 6 * 1024, NULL, 1, NULL, 0);
        Serial.println("Web: settings on http://<IP>/ (port 80)");
    }
}
