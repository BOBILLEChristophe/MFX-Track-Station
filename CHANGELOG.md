# Changelog

## Unreleased

### Changed
- **Pause of 1.5 ms after each frame, as the Gleisbox** (`RMTChannel::framePause`, on by default): the Gleisbox with an MS2 leaves the track at rest for 1 to 2 ms after nearly every frame (1667 pauses in 30 s, sniffer), our station sent synchronisations without a break. Without the pause the Märklin decoders (mLD3/mSD3) of the 150 Y 17 and the 241 A missed 10 to 15 % of the questions, search as well as verify; with it, 400 verifies out of 400 (scenario `Y`). Ruled out before: the form of the request, the length of the address (7, 11, 14 bits), the refresh of the other locomotives, the track voltage (38.8 V peak to peak, as the Gleisbox).
- Test bench: scenarios `X` (refresh on/off) and `Y` (pause off/on) alternate two settings over many verifies and print a summary; `B<addr>,<new>` moves a locomotive to another address; `Q` stops the refresh; `H` switches the pause off and on.
- Reading of CVs: the counts of the tries on the track are printed (end of a configuration read, each read for the master). With the pulse of 10 µs after 0011, every read of the 040 DE succeeds at the first try (108 tries for 59 reads of 2 or 4 bytes, 0 repeated; 37.6 s instead of 85 s with the pulse of 2 µs, when about one read in two needed a second try), and so do the reads of Rocrail for the 150 X 1518.
- **Deleted locomotives are not lost any more**: a decoder that did not hear the order to forget its address (track off, locomotive off the track) kept it and never answered the search again (241 A, 2 Nov 2026). The command station keeps every deletion (serial `D` or master) in the NVS (key `forget`, 16 entries) and, every 30 s while the track is on, verifies each address with the UID of the deleted decoder; if it answers, the order to forget (verify with UID 0) is sent again. The entry goes when the decoder is found by the search or bound again by the master. `D` on an unknown address with the track off is sent as soon as the track is on.
- **Registration by the master, end**: Rocrail creates a locomotive in its list when it gets the answer to a verify (0x03), which it asks after reading the name and CVs 2 to 70. With the ESU decoders (040 DE, Serie 150X) it never asked it (3 tries each), with the Märklin 241 A it always did. When the master bound and read a decoder and asks nothing more for 8 s (`PENDING_VERIFY_MS`), the command station verifies the decoder itself (up to 4 tries) and sends the answer to the master: the 040 DE and the Serie 150X now appear in the list of Rocrail.
- **End of the feedback request as the Gleisbox** (recorded with an MS2, Nov 2026): the answer window always at the same level, and the bits 0011 always followed by a transition. When they already end at the window level, a pulse at the other level is added (`MfxSignal::answerPulse`, `MfxSignal::pulseUs`). The 040 DE (recent ESU decoder) answered only with a transition after 0011: it now registers by itself, and its configuration is read without a failure. The pulse lasts 10 µs: with 2 µs, as on the Gleisbox, the 040 answered 29 verifies out of 47 through our H-bridge; with 5, 10 and 25 µs, 50 out of 50. Verified with the 150 X 1518 and the ESU 150X (50/50).
- 1-bit feedback: an answer is also recognised by a long run of regular intervals (at least 15, `ANSWER_RUN_MIN`); a weak answer above the reference counts as present; a verify is not counted as a failure when the reference window is noisy; a search question is asked up to 4 times, 70 ms apart; a locomotive that runs is not read for the master (failure answered); the search prints the bit where it stops.

### Added
- Test bench of the feedback (serial commands `M`, `N`, `S`, `T`, `A`, `I`, `F`): verify questions, name reads and search questions repeated, with the counts of both windows, the end of each request and statistics. `C<address>`: reads the configuration of one locomotive again. A line is printed when the reading of a configuration starts.
- The command station frame (UID and counter) is sent every 50 ms (`STATION_PERIOD_MS`, 500 ms before), as the MS2 does (46 ms); the decoder search starts with the UID FFFFFFFF, as the MS2 sends it (the UID is not compared with a count of 0).
- **Feedback request as a Marklin MS2 sends it** (recorded on the track of a Gleisbox with an MS2, with the new track sniffer): the question (search, verify, read CV), a synchronisation, the command station frame (111 101, UID and counter), then 2 synchronisations (11 before), the bits 0011 and windows of 6.0 ms (6.4 ms before). The recent ESU decoder of the 040 DE 9 answers the MS2 (search, verify, 80 reads of CVs) but never answered the former request. `MfxSignal::feedbackWindows()` and `dataWindow()` take the inserted frame; `MfxSignal::MAX_ITEMS` 434 (512 in the RMT memory). New unit test.

## 0.11.0 — 2026-10-28

### Added (feedback)
- **1-bit feedback** (`src/Feedback.cpp`): `Feedback::ask()` sends a frame followed by the feedback request (11 synchronisations, bits 0011) and two windows of 6.4 ms at constant level. The PCNT counts the rising edges of the sensor comparator in the answer window and in a reference window, from 200 µs after the start of each window (the tuned circuit still rings 60 to 80 µs after the last polarity change). The decision uses the regularity criterion below. Sensor: see the README; comparator output on GPIO 26.
- 1-bit feedback, regularity criterion: the interrupt also counts, in each window, the intervals of one carrier period (17 to 21 µs) between two rising edges. A decoder answered when the answer window holds at least 150 of them (about 300 for a recorded answer) and at least 120 more than the reference window. The noise of a running motor is mostly at random intervals (0 or 1 regular interval per window at speeds 12 and 63 of an ESU 150X, none at full speed); at speed 101 it gave up to 125 regular intervals, but as many in both windows (difference below 60).
- **Automatic discovery** (`src/Discovery.cpp`): every 2 s while the track is powered, a decoder search with count 0; if a decoder answers, its UID is read bit by bit (Krauss 3.2.3) and checked. A new decoder gets the first free address, is added to the refresh and receives a Bind; the serial port prints its UID and the line to add to `main.cpp`.
- `MfxFrame::search()` (command 111 010), `MfxSignal::hold()` and `MfxSignal::feedbackWindows()`, with unit tests. Items now carry their duration.
- **Reading of CVs** (data feedback, Krauss 2.2.8, 2.3.3, 3.2.1): `Feedback::readCV(sid, cv, index, bytes, out)` sends the read command (`MfxFrame::readCV()`, 111 000 + CV + index + count), then the data request: 11 synchronisations, bits 0011, and a pause with 25 µs clock pulses, 23 every 912 µs then 2 x (3 + 8n + 8 + 4) every 456 µs (`MfxSignal::dataWindow()`; 49.2 ms for 2 bytes, as measured on a Marklin Gleisbox). An interrupt times the rising edges of the comparator during the pause with the CPU cycle counter; `MfxDemod` (`lib/MfxDemod`) measures the carrier period, follows its phase, finds the 180° phase changes, decodes the bi-phase bits, finds the preamble 010 and checks the checksum. Two tries per read, at least 150 ms between two reads (Krauss 4.9: answering heats the decoder).
- The read CV frames are identical, bit for bit, to frames recorded on the track of a Gleisbox; `MfxDemod` is tested on four answers recorded there (CV 000/00, 080/00, 003/01, 02F/01), on synthetic answers (noise, drift, false edges at the pulses) and on error cases.
- `MfxConfig` (`lib/MfxConfig`): reads the configuration of a decoder through its CVs (Krauss chapter 6): block 01 and the block table (CV 004), manufacturer and product, name (CA 18), symbols of F0 to F15 (block 04), acceleration, braking, Vmin, Vmax and tachometer (block 05). Most CAs are read 4 bytes at a time from index 0 (type and first values in one answer). Pure C++, tested with a simulated decoder.
- The configuration is read only while the locomotive stands still: the noise of a running motor spoiled a third of the reads (150 Y at speed). A locomotive that runs, or starts during the reading, is read again 20 s later, without counting a try.
- `Discovery::readConfig()`: reads and prints this configuration for each new decoder (its name goes in the line for `main.cpp`) and, as a test, for every declared locomotive when the track is first powered.
- **Loco list for Rocrail** (`lib/Cs2Config`, `Message::configRequest()`): the command station answers the config data request "loks" (command 0x20) as a CS2: echo with the response bit, then a config data stream (command 0x21, first frame: length and CRC; then 8-byte frames) holding `lokomotive.cs2`: name, address (loco id 0x4000 + SID), mfx UID and, when read in the decoder, acceleration, braking, Vmin, Vmax, tachometer and the CS2 type of each function (mapped from the mfx symbols). The data is in the zlib format with stored blocks (no compression library); format and CRC as in `can2lan` (railroad project of Gerhard Bertelsmann). `sendToRocrail()` in `main.cpp`. Commands from Rocrail that are not handled are printed once each (trace of the commands).
- The ping of Rocrail (command 0x18) is answered as the master of a CS2 (UID of the command station, version 3.8, type 0xFFFF): the `mbus` driver of Rocrail (which now also covers the CS2) then deals with the command station as with a CS2.
- `Loco` keeps the configuration read in its decoder (`MfxInfo`); a new decoder takes the name stored in it.
- `MfxFrame::verify()` (command 111 100, existence query), tested against the example of "Die mfx-Rahmenstruktur". Serial command `V`: verifies every declared locomotive and prints the counts of the 1-bit feedback.
- The reading of a configuration stops when the track is switched off, and is done again later.
- Serial command `R` (then Enter): re-registration of every decoder. The re-registration counter is increased (kept in NVS) and the command station restarts; `?` prints the help.
- Oscilloscope trigger (`Feedback::setScope(pin, mode)`, GPIO 25 in `main.cpp`): the pin is high during the windows of the 1-bit feedback, during the capture of a CV answer, or both.
- Cleanup of the diagnostics: `Feedback::printPinState()` and the interrupt counts of the 1-bit feedback removed (the trace of the discovery is one line per request, off by default); the current is printed only when it changes by at least 50 mA (`CURRENT_DISPLAY_STEP_MA`); a declared locomotive that does not answer is reported once, after its last try.
- A new decoder whose configuration was read at its registration is not read a second time with the declared locomotives. Tested: the ESU 150X is found by the search after `R`, gets address 5 and its configuration is read (60 reads, 0 failed), also while an ESU 040 runs at speed 101 on the same track.
- `Feedback::setTrace(cv, search)`: separate traces for the reading of CVs and for the 1-bit requests of the discovery (both off by default).
- Room for 16 locomotives (`MAX_LOCOS`): the declared ones and those found by the discovery. `Message::addLoco()`; the locomotive map is protected by a mutex.

### Changed
- RMT channel: the 8 memory blocks (512 items), so that a chunk with a data feedback of 8 bytes (up to 395 items) fits without refill.
- After a feedback request, the signal task sends one chunk from `mfxQueue` before the next request: the locomotives keep their refresh during a search.

### Fixed
- **Bit stuffing sent one bit too many** (new finding): after each inserted 0, the encoder read one more byte after the end of the frame and sent it on the track before the synchronisation. It concerned every frame with 8 consecutive 1 bits, for example the address assignment of the 150 X 5 (UID `FF E9 B1 94`).
- **Signal gaps** (#6): with nothing to send, the signal task sent a 0.5 ms synchronisation then slept 1 ms, leaving the track about 0.5 ms without transition at the RMT idle level; the level variable ignored this idle level, which could remove the expected transition at the start of the next chunk. The signal is now continuous (see below).
- The item buffer (100 items) could overflow with a long frame plus stuffing; chunks are now checked against their capacity.

### Added
- `MfxSignal` (`lib/MfxSignal`): turns frames into 50 µs half-bits (synchronisation, bits, stuffing) and keeps the track level from one chunk to the next. Pure C++, 5 unit tests.
- Orders from Rocrail (speed, direction, function) are sent at once, at the head of the queue (`Message::sendNow()`), instead of waiting for the next refresh (up to 50 ms). Requests send nothing.
- `MfxFrame` (`lib/MfxFrame`): builds the mfx frames (address on 7, 9, 11 or 14 bits, drive, functions F0-F15, Bind, command station broadcast, CRC). Pure C++, testable on a PC.
- Unit tests: `pio test -e native`. They check that `MfxFrame` gives exactly the same bits as the code of 0.10.0, and the CRC 0xD9 of a frame seen on an MS2.
- GitHub Actions: unit tests and ESP32 build on every push and pull request.

### Changed
- Track signal: one task keeps the RMT busy. It builds the next chunk while the current one is sent (two buffers), and sends synchronisations when there is nothing else, so the signal never stops. Each frame is sent between two synchronisations. The RMT idle level follows the level of the track, so the short pause between two chunks adds no transition. Whole chunks fit in the RMT memory (3 blocks).
- `Message::periodic()`, `centrale()` and `setSID()` build their frames with `MfxFrame`; `addr0()`, `addr()`, `CRC()` and `bCRC()` are removed. The track signal is unchanged.
- Project renamed **MFX Track Station** (formerly directMFX_ESP32); the startup banner shows the new name.
- README rewritten: what works today and what remains to do, compilation, Rocrail setup (CS2, port 15731), how to declare a locomotive; duplicated feature list removed.

## 0.10.0 — 2026-10-13

Corrections from the audit of 13 October 2026 (step 1 of the roadmap), without changing the architecture.

### Fixed
- **MFX re-registration counter** (#1): it was incremented by the default constructor of `Loco`, which `std::unordered_map::operator[]` calls. A Rocrail command for an unknown locomotive changed the counter and made every locomotive on the track register again. The counter now lives in `Centrale`, is sent on 16 bits, is kept in NVS and only changes on request (`Centrale::setCounter()`). Its default value, 5, is the one sent by 0.9.2 with 4 locomotives, so the update does not force a re-registration.
- **CS2 requests taken as orders** (#2): `data[]` kept the bytes of the previous frame, so a speed or function request (short frame) applied the value of the previous command, often to another locomotive. `data[]` is cleared, and requests no longer change anything.
- **Direction** (#7): CS2 values 0 (keep) and 3 (toggle) were read as "reverse". 0 keeps, 3 toggles, and a change of direction sets the speed to 0, as a CS2 does.
- **Unknown locomotives**: `Loco::findLoco()` returns `nullptr` instead of creating a locomotive that was never refreshed; the command is ignored with a message.
- **Overload** (#3, #4): the monitoring task had 1 KB of stack and printed a float at the moment of the overload (stack overflow). Stack is 4 KB, values are printed as integers. After an overload the track stays off until the next Go from Rocrail; it was powered again after 5 s, endlessly on a persistent short, and even after a Stop.
- **Current measurement** (#12): read in millivolts (`analogReadMilliVolts()`) and converted to mA with `CURRENT_MV_PER_A`, instead of comparing raw ADC counts to a value written in mV. Check `CURRENT_MV_PER_A` against your board.
- **Rocrail connection** (#5): the track signal and the tasks start at boot, without waiting for Rocrail. Rocrail can disconnect and reconnect without restarting the command station; the first frame is no longer lost. The network code is in `main.cpp` only, so `RxTask` no longer depends on WiFi.
- `1 << 31` on an `int` (undefined behaviour) when sending the UID of the command station (#16).
- The UID and counter of the command station are set before the first broadcast.
- `printf("%s")` was given a `String` object instead of `const char*` in the traces of `periodic()` and `setSID()` (crash as soon as `trace` was enabled).

### Changed
- TCP port 15731, the port of the Marklin CS2 used by default by Rocrail (it was 2560).
- Platform pinned to `espressif32@6.9.0` (Arduino core 2.0.17). Unpinned, PlatformIO now takes a platform with the Arduino core 3.x, where the project did not compile.
- IN2 inversion uses `esp_rom_gpio_pad_select_gpio()` and `esp_rom_gpio_connect_out_signal()`, available in the Arduino cores 2.x and 3.x, instead of `GPIO_PIN_MUX_REG` and `gpio_matrix_out()`, removed from the core 3.x.
- WiFi credentials go in `src/secrets.h`, ignored by Git (copy `src/secrets.h.example`).
- Removed dead code: second creation of `mfxQueue`, global `data[8]`, `TP` / `T_S88` / `Time`, `Pa[]`, `togglePin()`, `rrHash`, `Loco::zahler` and `locoActive`; unused `ACAN_ESP32` dependency (#15).
- `.DS_Store` files removed from the repository and ignored.
- Commands received from Rocrail are printed when `Message::setTrace(true)` is called (on by default in `main.cpp`); the very verbose traces of the internal tasks stay under `trace`.
- A locomotive declared with an address already used prints an error at boot (the second one replaces the first).
- UID of the 150 X 5 corrected: `FF E9 B1 94`, as seen on the layout with the MS2. Addresses: 150 X 5 on 5, 040 DE 9 moved to 4.

## 0.9.2

Version before the audit (commit `f02aa52`).
