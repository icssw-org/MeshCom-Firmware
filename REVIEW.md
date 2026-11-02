# REVIEW.md -- Review Standard for MeshCom Firmware

Status: 2026-11-02. This file is the review standard for every pull request against `dev`: for
the author's self-review before opening the PR, and for anyone reviewing it (human or AI tool).
AI review tools such as Claude Code `/code-review` read this file from the repository root.
Forks may keep additional rules in `docs/review-*.md`; a reviewer reads those too.

A review answers four mandatory questions first (section 1). Everything after that is the
supporting checklist. A PR whose description does not answer the four questions is not ready for
review.

Every rule has an ID (`R` mandatory questions, `F-` fleet compatibility, `H-` code hotspots, `E-`
evidence and tests). A review comment cites the ID; the reason is written here, not repeated in
each comment.

---

## 1. The four mandatory questions

| ID  | Question                                                                                      | A "yes" needs this evidence in the PR description                                               |
| --- | --------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| R1  | **Complete?** Does the change cover every platform, board and code path that has the feature? | List of every site that implements the feature, each marked changed / not affected (why)        |
| R2  | **Does every platform still build?**                                                          | The build command, the number of envs built, the number that failed (from the run, not memory)  |
| R3  | **Checked against every other implementation?** Web UI, displays, app, console, sanitizers.   | The surface table (1.3), each row marked changed / unaffected (why) / not checked               |
| R4  | **Within the LoRa length budget?** Byte count, not character count, worst case path.          | The byte calculation for the longest input on the longest path, and where the limit is enforced |

A "no" or a missing answer to any of the four is a blocker (section 6).

### 1.1 R1: Completeness -- not a fix for one platform

MeshCom runs on ESP32 (classic and S3) and nRF52840, with several display families and user
interfaces. A fix that works on the author's board is half a fix if the same logic lives elsewhere.

- **Twins.** Many functions exist once for ESP32 and once for nRF52: `esp32_main.cpp` /
  `nrf52_main.cpp` (serial command handling, gateway loop), `esp32_functions` / `nrf52_functions`,
  the UDP send and receive paths, network start, battery reading, BLE settings import. A diff on
  one side is a question to the other side. Either both change, or the description says why not.
- **Both arms of an `#if` are live.** For every `#if FOO ... #else` implementation pair, list which
  envs compile each arm before assuming one is dead. Example: two battery implementations are
  selected per variant, each used by about half the fleet; a fix in one file misses the other half.
- **Siblings.** After "make X safe" (atomic, locked, bounded, sanitized), grep for every sibling of
  the same shape with a different name and fix all of them. A partial fix reads as done.
- **Same code, different task context.** One platform may queue a handler (ESP32 BLE into a
  queue, drained by the loop task), the other runs it inline in a callback (nRF52 Bluefruit). Diff
  the task context, not only the code. Validation must run on both entry points (a settings value
  that ESP32 sanitizes at load and nRF52 saves unsanitized from BLE is an R1 failure).
- **Duplicated logic.** Formatters, parsers, callsign checks, JSON builders that exist more than
  once: diff the copies. The bug lives in the copy that did not get the fix.
- **Display families.** OLED (u8g2, Heltec / T-Beam / TTGO), e-paper GFX (`src/Displays/`:
  Vision Master E290 and E213, Wireless Paper), LVGL (T-Deck and T-Deck Plus in `src/t-deck/`,
  T-Deck Pro in `src/t-deck-pro/`, T5 e-paper in `src/t5-epaper/`). A display change names each
  family it touches and each it does not.

Method: grep the identifier, the user-visible string and the setting name across `src/`, not only
the directory of the change. The scope of a review follows the call graph of the event, not the
folder structure.

### 1.2 R2: Every platform still builds

- **All envs.** `pio run` without `-e` builds every env in `default_envs` (33 on 2026-11-02, plus
  the variants in `variants/*/platformio.ini`). For a change to shared code (headers in `src/`,
  `configuration_global.h`, fonts, `platformio.ini`) build all of them. Quote the count of built
  and failed envs from the run.
- **Minimum for a narrow change:** one env per class: `wiscore_rak4631` (nRF52),
  `heltec_wifi_lora_32_V3` (ESP32-S3), `ttgo_tbeam` (classic ESP32, tightest IRAM),
  `E22_XML-DevKitC` (tightest DRAM). For display or web changes add `t_deck`, `t_deck_pro`,
  `vision-master-e290`, `wireless-paper`. A shared header means every env that includes it.
- **Link regions, not the `RAM:` line.** On ESP32 read `iram0_0_seg` and `dram0_0_seg` in
  `firmware.map` for the tight envs. A single new libc call (for example `strtok_r`, `strptime`)
  can overflow IRAM on `ttgo_tbeam`, while PlatformIO still prints a low RAM percentage.
- **Flash budget.** nRF52 (RAK4631) runs at 80-95 % flash. Every new web page, font or large table
  states its flash delta on the tightest env.
- **Guards.** New `#if` tests `defined(...)`; many `BOARD_*` macros are bare product names. An env
  without the new flag compiles to the same code as before. `#if`/`#endif` counts match per file.
- **New env or variant:** also update the CI workflow (`.github/workflows/`) and any flasher or
  release table that lists envs. A new env that CI does not know about is a release that ships
  without that board.
- **Native tests.** If the PR touches a unit that has a `[env:native_*]` test, run it
  (`pio test -e native_<name>`). Never run bare `pio test`: it walks the board envs and uploads to
  attached hardware.

### 1.3 R3: Checked against every other implementation

A change to how text, a setting or a command is handled reaches more surfaces than the one the
author looked at. Example: national characters (glyphs) were fixed for one display, while the web
UI, the e-paper status lines, the serial echo, the app and the external sanitizers each handled the
same bytes differently.

The PR description carries this table, with every row filled in:

| Surface                     | Where (upstream/dev)                                                               | Typical question                                                    |
| --------------------------- | ---------------------------------------------------------------------------------- | ------------------------------------------------------------------- |
| On-air encoder (chokepoint) | `encodePayloadAPRS()` in `aprs_functions.cpp`, `charset_filter.cpp`                | Does every composer still pass through the one filter?              |
| On-air decoder              | `decodeAPRS()` in `aprs_functions.cpp`                                             | What does an old node send that this code now receives?             |
| Web UI                      | `src/web_functions/` (HTML, JS, JSON endpoints)                                    | Escaped before `innerHTML`? Same length limit as the firmware?      |
| OLED displays               | u8g2 paths in `loop_functions.cpp`                                                 | Does the font have the glyph? What is drawn when it does not?       |
| E-paper GFX displays        | `src/Displays/` (E290, E213, Wireless Paper)                                       | Message pages and status lines (temperature, SSID, callsign)?       |
| LVGL displays               | `src/t-deck/`, `src/t-deck-pro/`, `src/t5-epaper/`                                 | Font coverage per size, font flags vs `lv_conf.h`, input field max? |
| BLE to the app              | `phone_commands.cpp`, `mh_phone.cpp`, BLE JSON (244-byte budget)                   | Frame layout unchanged? JSON still fits?                            |
| Serial and network console  | `esp32_main.cpp` / `nrf52_main.cpp` command handling, `--` commands, `::` messages | Echo, help text, argument form unchanged?                           |
| External UDP (EXTUDP, 1799) | `extudp_functions.cpp` (in and out JSON)                                           | Key set, length checks, consumers (MCApp, MCProxy, loggers)?        |
| Server UDP (1990), APRS-IS  | gateway paths in `loop_functions.cpp`, `udp_functions`                             | Server-side parser and sanitizer, aprs.fi rendering?                |
| Message store, mailbox      | `msgstore_*`                                                                       | Stored length limit equals the on-air limit?                        |
| External consumers          | App, MCProxy, MCApp / mc-chat, mcmap, aprs.fi                                      | Does their sanitizer accept and strip the same code points?         |

Rules:

- **One code-point table.** A text change states which code points are accepted, folded, replaced
  and stripped, and checks that the firmware filter, the web UI and the known external sanitizers
  (MCProxy, the app) agree. Controls, bidi overrides and isolates (U+202A-U+202E, U+2066-U+2069),
  zero-width characters and invalid UTF-8 are the usual gaps.
- **A message that is empty after sanitizing** is dropped, on every path, not published as empty.
- **Fonts.** A glyph change lists the covered ranges per font and size, including the letters of
  the languages it claims (PL, DE, FR, ES, PT ...). LVGL fonts are generated without sub-pixel
  rendering (`--lcd`) when `LV_USE_FONT_SUBPX` is 0, otherwise every glyph draws blank.
- **Render proof per display family.** "Looks good on the T-Deck" covers the T-Deck. Each display
  family the PR claims needs a photo, screenshot or host render test; the others are listed under
  "not checked on hardware".
- **Settings and commands** follow the same idea: a setting is read and written by the web UI, the
  app (BLE JSON), the console and the JSON export; a command is spoken by the app, the web UI, the
  console and scripts. A change to one names the others.

### 1.4 R4: The LoRa length budget

The frame is at most 255 bytes (`UDP_TX_BUF_SIZE`). Layout: type byte, 4-byte `msg_id`, flag/hop
byte, source path (`>`-terminated, at most 120 bytes), destination (terminated by a repeat of the
type byte), payload, `0x00`, HW id, MOD byte, 2-byte FCS, 4-byte trailer.

- **`encodeAPRS()` clamps silently.** If header plus payload reach 245 bytes, the payload is cut at
  byte 245, without error and without regard to UTF-8 boundaries. The sender sees no failure; the
  receiver gets a shortened text, possibly ending in half a character.
- **The path grows on every hop.** Each relay appends `,<CALL>` (up to 10 bytes) to the source path
  and re-encodes. A text that fits on the first hop can be clipped on the fourth. Text messages
  start with 4 hops (`MAX_HOP_TEXT_DEFAULT`). Worst case for a text message:

  ```
  6 (header) + source path (9 + hops x 10 + 1) + destination (<= 9) + 1
    + text + "{NNN" (4, direct message with ack) <= 245
  ```

  With 4 hops that leaves about 175 bytes for text. The firmware limit is 160 bytes
  (`sendMessage()` rejects longer text). A PR that raises a limit, adds a tag to the payload or
  lengthens the path redoes this calculation.

- **Bytes, not characters.** All limits are UTF-8 byte counts. A national character is 2 bytes, many
  symbols 3, an emoji 4 or more. Red flags in review: `String::length()` compared against a limit
  that came from a character count, HTML `maxlength` (counts UTF-16 units), LVGL
  `lv_textarea_set_max_length()` (counts characters), Python `len(str)` and `text[:n]`, JavaScript
  `.length`. Count with `TextEncoder` / `encode("utf-8")` / `strlen()` on the UTF-8 buffer.
- **Every input path enforces the same byte limit:** web UI, app, serial and network console,
  T-Deck and T-Deck Pro keyboards, EXTUDP (`msg` <= 150 bytes), remote commands, message store.
  A path that accepts more than `sendMessage()` takes produces a silent reject; a path that
  truncates must cut on a code-point boundary (`charset_utf8_safe_truncate()`).
- **Payload tails have their own budgets:** position comment (`atxt`) 25 bytes, the combined tag
  tail of a position 100 bytes, BLE JSON 244 bytes. A new tag or key states its worst-case bytes
  against the budget it consumes.
- **Airtime is part of the budget.** A new on-air frame, or a larger one, states frames per hour per
  node in the default and the worst configuration, the minimum interval, and the relay
  multiplication.

Test: the PR includes or names a test at the limit: max-length input with multi-byte characters,
through the longest path, checked for byte count and for a valid UTF-8 ending.

---

## 2. Fleet compatibility

Nodes from 4.35 to the current release run side by side in the field, many without an update path.
Every change to a frame, a settings bit or a command is checked against "an old node receives
this" and "this node receives an old frame" first, and against "works on my bench" second.

| ID   | Rule                                                                                                                                                                                                               |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| F-01 | **The LoRa frame layout is frozen** (section 1.4). Decoders tolerate a missing trailer; encoders always write it. No new header byte, no new type without a fallback for receivers that do not know it.            |
| F-02 | **New information goes into an optional tag at the end of the payload** (`/R=`, `/U=`, `#name`). Unknown tags are ignored. Never parse with a plain `split('/')`.                                                  |
| F-03 | **No flag-day changes.** A destination field with characters outside the callsign pattern is dropped by older nodes and not relayed: old nodes become relay holes.                                                 |
| F-04 | **A settings bit is never "free"** because the current tree does not read it. Check `git log -S "0x2000" -- src`. Old bits stay set in flash; a default-off feature on a recycled bit is on after the update.      |
| F-05 | **Masks are 16 bits wide.** Every new mask names each bit it touches.                                                                                                                                              |
| F-06 | **Setting member names are the persistence contract.** Renaming a member or its NVS key silently resets the value to the default on the next boot. NVS keys are at most 15 characters.                             |
| F-07 | **No `FLASH_STRUCT_VERSION` bump** without a real layout change and a migration note: it wipes callsign, WiFi and sensors of every updating node.                                                                  |
| F-08 | **Commands are an API.** `--<cmd>` is used by the app, the web UI, the network console and scripts. No rename, no changed reply, no changed argument form without a before/after comparison of the console output. |
| F-09 | **BLE frames and the BLE JSON are a contract with the app.** New keys go at the end; the 244-byte budget is stated in typical and worst case.                                                                      |
| F-10 | **UDP schemas (server 1990, EXTUDP 1799) have external readers.** New keys go at the end; the consumer is named in the description.                                                                                |
| F-11 | **Region and frequency.** A change to a send path checks the region tables and the APRS frequency handling on both platforms.                                                                                      |
| F-12 | **What does the node do after the update, before anyone types a command?** Defaults, bits, new automatic frames. The answer is in the description.                                                                 |

---

## 3. Code hotspots

These classes caught real bugs in audits of this code base. Check them on every diff.

| ID   | Check                                                                                                                                                                                     |
| ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| H-01 | **Length comes from the producer**, validated against the destination buffer: every new `memcpy`, `snprintf`, `serializeJson`. A length prefix is wide enough for the largest payload.    |
| H-02 | **The write index bounds the loop**, not a parallel counter. Validate length before decoding, not after.                                                                                  |
| H-03 | **`read(buf, SIZE)` then `buf[len] = 0`** writes one past the end. Read `SIZE - 1` or size the buffer `+1`.                                                                               |
| H-04 | **Untrusted multi-byte fields:** check the whole field width against the remaining length before any fixed-offset access. Boundary tests 0, 1, max, max+1.                                |
| H-05 | **Format strings are literals.** Never `Serial.printf(buf)` with text that came from the mesh; a second emit of an already formatted buffer re-parses `%` from received text.             |
| H-06 | **Security flags default to deny.** The reject path runs unconditionally, not inside a loop over attacker-sized data. Empty received secrets are rejected explicitly.                     |
| H-07 | **Persist vs. apply.** A setting written to flash is also applied to the running subsystem, or the user is told a reboot is needed. A stored setting is applied at boot.                  |
| H-08 | **Success means the work was done.** A handler that returns OK must have performed the side effect. Stubs and TODOs in reachable handlers are findings.                                   |
| H-09 | **Checks before effects.** Auth, validation and gating happen before copy, echo, ring insert or reply. A function that can refuse returns a result, and the caller reads it.              |
| H-10 | **Locks cover every reader and writer**, including the free/reset path and read-only reporters (web, JSON, console). One unlocked writer defeats every locked reader.                     |
| H-11 | **nRF52 radio context.** Everything reachable from `OnRxDone` is lock-guarded, print-free and flash-write-free. No `%lld` on nRF52.                                                       |
| H-12 | **Blocking calls on the loop or radio task** (`delay(`, `scanNetworks`, `hostByName`, SD access, long critical sections) name their task and their upper bound.                           |
| H-13 | **Time arithmetic:** `(uint32_t)(now - t) >= X`; no `millis() + N` deadlines; no ids from `millis()`.                                                                                     |
| H-14 | **Web output:** everything from the mesh (callsign, path, text) is escaped before `innerHTML`; hand-built JSON is escaped; HTML `maxlength` matches the server limit in bytes (R4).       |
| H-15 | **Comments that claim safety are hypotheses.** "Never overlaps", "unchanged", "already sanitized": verify the code enforces it. A comment rewritten to make a bit look free is a blocker. |
| H-16 | **Copy instead of call:** a function that copies 25 lines of another drops exactly the branches that are missed later. Use a shared helper, or say why not.                               |

---

## 4. Tests

| ID   | Rule                                                                                                                                                                                             |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| E-01 | **Every bug fix has a regression test that fails before and passes after.** The failing run is on the final test, not on a first draft.                                                          |
| E-02 | **A test that cannot fail is worse than none.** No assert, unconditional skip, fallback values that mask a missing key, comparisons shortened so the test still passes. Review test helpers too. |
| E-03 | **Every changed source file is compiled by at least one test**, or the description says it is hardware-only. Check: revert the change in that file; some test must go red.                       |
| E-04 | **Run the checks on the tree that is committed**, after the last edit. A green run before a late fix does not cover the fix.                                                                     |
| E-05 | **Hardware proof for radio, display, network, settings and sleep changes:** board, env, version string from `--info`. What was not tested on hardware is listed under "Not checked on hardware". |
| E-06 | **Bench traffic** goes only to a test group or as a direct message to an own node, never as a broadcast; own callsigns only; minimum TX power.                                                   |

---

## 5. Review method

1. **Read the description against the diff.** File count, variant list and behaviour claims match
   `git diff --stat` and the code. A claim with no matching hunk is a finding.
2. **Answer R1-R4 yourself.** Do not accept "builds on all platforms" or "checked the web UI"
   without the evidence from section 1.
3. **Build before writing.** Build the PR on boards the author does not have (R2); quote the
   command and the `error:` lines.
4. **Flag, then try to refute.** Every candidate finding gets an attempt to disprove it. Verdicts:
   CONFIRMED (a concrete input reaches the defect), PLAUSIBLE (real hazard, states what would
   confirm it), REFUTED (recorded with the reason, so nobody re-reports it).
5. **Old bugs are not this PR.** What the PR does not touch goes into a separate "for information"
   block and does not gate the merge.
6. **One comment per review round:** verdict first; numbered findings with symptom, cause
   (file, function), consequence, fix proposal and the test you want; then minor notes; then the
   merge statement.
7. **Withdraw on evidence.** If the author disproves a finding with a build log or a hardware log,
   withdraw it in the thread and re-check the rest.

---

## 6. Severity

| Level        | Meaning                                                                                                                                                                                | Consequence                                             |
| ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| **Blocker**  | Build or link break on any env (R2), memory corruption, wire or settings break against old nodes, unintended transmission after update, data loss in flash, LoRa budget overrun (R4)   | No merge                                                |
| **Blocker**  | Feature fixed for one platform or path while its twin or sibling stays broken (R1), a surface in the R3 table changed without being checked, likely crash or misbehaviour in the field | Fix before merge, or a deferral with a reason in the PR |
| Not blocking | Latent defect (logging on the hot path, missing test, copy instead of helper)                                                                                                          | May go in, noted                                        |
| For info     | Style, naming, comments, old bugs outside the PR                                                                                                                                       | Never a reason to block                                 |

A finding without file, mechanism and evidence (grep, test, log) is a guess and is labelled as one.

---

## 7. PR description checklist

The description covers, in this order:

1. **Summary:** symptom, cause in one sentence, `Fixes #NNNN`.
2. **Changes:** one line per file and function: what and why.
3. **R1 completeness:** the sites that implement the feature and how each was handled.
4. **R2 builds:** command, envs built, envs failed; flash/RAM delta on the tightest env.
5. **R3 surfaces:** the table from 1.3.
6. **R4 length budget:** the byte calculation, or "no change to any on-air length".
7. **On the wire:** what changes on air, and what an old node does with it (section 2).
8. **Tests:** the regression test (red before, green after), native envs run.
9. **Not checked on hardware:** paths and reasons.
10. **Not in this PR:** related work left out.

---

## Appendix: quick commands

```sh
# Description vs diff
git diff --stat upstream/dev
git diff -w --stat upstream/dev          # same hunks: no reformatting as a side effect

# R1: find every site of the feature
git grep -n "<identifier>" -- src
git grep -n "<user-visible string>" -- src
git diff upstream/dev --name-only -- src/esp32 src/nrf52

# R2: build everything, or one env per class
pio run
for e in wiscore_rak4631 heltec_wifi_lora_32_V3 ttgo_tbeam E22_XML-DevKitC; do pio run -e $e || echo "FAIL $e"; done
grep -A3 "iram0_0_seg\|dram0_0_seg" .pio/build/ttgo_tbeam/firmware.map | head

# R4: character counts where byte counts belong
git diff upstream/dev | grep -nE "^\+.*(maxlength|set_max_length|\.length\(\) *[<>]|len\(|\[:[0-9]+\])"

# Code hotspots
git diff upstream/dev -- src | grep -nE "^\+.*(delay\(|scanNetworks|hostByName|SD\.)"
git diff upstream/dev -- src | grep -nE "^\+.*(sprintf|strcpy|strcat|strtok)\("
git diff upstream/dev -- src | grep -nE "^\+.*millis\(\) *\+"
git diff upstream/dev -- src | grep -nE "^\+#if " | grep -v defined
git diff upstream/dev -- src | grep -nE "^\+.*node_sset[0-9]?.*0x[0-9A-Fa-f]+"
```
