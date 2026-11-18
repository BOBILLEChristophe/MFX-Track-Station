/*

   WebUi.h

   Settings of the command station from a web browser: http://<IP>/ (port
   80, the same WiFi as Rocrail). The page is in the program (WebPage.h);
   the settings go through a small JSON API and are saved in LittleFS
   (ConfigStore.h), one file per subject.

     GET  /                  page
     GET  /api/l88           Link S88: configuration and state
     POST /api/l88           {"bus": [1, 0, 1], "cycleMs": 10}: new configuration
     GET  /api/files         files of /config, space used
     GET  /api/config?name=  raw content of /config/<name>.json (backup)

   The web page and the serial console call the same functions (here
   LinkS88::configure): one place for each setting.

   The server runs in its own task, of low priority, on core 0: the track
   signal (RMT, core 1) and the feedback are not slowed down.

   Next subjects, on the same model: RailCom, protection and current,
   locomotives, mfx.

*/

#pragma once

namespace WebUi
{
    // After WiFi is connected and ConfigStore::setup()
    void setup();
}
