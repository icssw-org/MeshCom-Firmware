/**
 * RM GUI W2a -- server side of the Remote page, see web_rm_handlers.h for the route table.
 *
 * Secrets: a password (the node's own, or a target's) lives only in the fixed stack body buffer of the
 * handler that read it, is parsed in place (web_rm_parse.h), and is wiped with volatile stores
 * (rm_wipe) before the handler writes its answer. A saved node is call + SHA-256 key only: the key is
 * derived straight from the body buffer, held in a LOCAL RmNodes (never static) and scrubbed on every
 * path. No handler prints, logs or echoes a password or a key; answers carry static tokens only.
 *
 * Output limits: one write <= 512 B, every printf format below
 * is a short literal with numeric conversions only (nRF52 Print::printf has a 256 B buffer, ESP32
 * mallocs above 64 B); text from outside goes through rm_json_str() in <= 200 B pieces.
 */
#include <Arduino.h>

#include "web_functions.h"
#include "web_rm_util.h"
#include "web_rm_parse.h"
#include "web_rm_handlers.h"

#include "../command_functions.h" // nodePasswdApply()
#include "../remote_cmd.h"        // rmDeriveKey()
#include "../rm_nodes_store.h"
#include "../rm_format.h"         // RM_HEARD_WINDOW_MIN
#include "../rm_runtime.h"        // rmSendCommandKey()
#include "../rm_validate.h"
#include "../nbr_matrix.h"        // nbrMatrix
#include "../nbr_views.h"         // nbrMhRows(), nbrMhGet(), nbrHardwareName()
#include "../uptime_min.h"        // uptimeMin16()
#include "../nvs_diag.h"         // RM-NVS-01: NVS stats on a failed save

static_assert(RM_FORM_SLOTS == RM_NODES_SLOTS, "parser and store disagree on the slot count");

#define RM_BODY_MAX RM_FORM_BODY_MAX // 260
#define RM_BODY_TIMEOUT_MS 2000UL    // no byte for this long
#define RM_BODY_TOTAL_MS 3000UL      // absolute deadline for the whole body
#define RM_HEARD_MAX 12
#define RM_HEARD_SCAN 24             // rows looked at to find RM_HEARD_MAX with a routable call

const char *rmReadBody(char *buf, size_t cap, long content_length)
{
    buf[0] = '\0';
    if (content_length <= 0 || (size_t)content_length >= cap)
        return "size";

    long got = 0;
    unsigned long last = millis();
    const unsigned long start = last;
    // absolute deadline: a 200-byte body arrives in ms, a slow drip must not stall the loop task
    while (got < content_length && (millis() - last) <= RM_BODY_TIMEOUT_MS && (millis() - start) <= RM_BODY_TOTAL_MS)
    {
        yield();
        if (web_client.available())
        {
            int c = web_client.read();
            if (c < 0)
                break;
            buf[got++] = (char)c;
            last = millis();
        }
        else if (!web_client.connected())
            break;
    }
    buf[got] = '\0';
    return got == content_length ? nullptr : "short";
}

// RM-NVS-01: a failed rmNodesSave() logs the NVS fill level (ESP32 only; nRF52 has no nvs_get_stats)
static void rmNodesStoreFailLog(void)
{
#ifdef ESP32
    NvsDiagStats st;
    char line[96];
    if (nvsDiagRead(st))
    {
        nvsDiagFormat(line, sizeof(line), st);
        Serial.printf("[RMNODES];save_failed;%s\n", line);
    }
    else
        Serial.printf("[RMNODES];save_failed;nvs_stats_unavailable\n");
#else
    Serial.printf("[RMNODES];save_failed\n");
#endif
}

/** 200 {"ok":true} or 422 {"ok":false,"err":"<token>","msg":"<sentence>"}; err is a static token, never input,
 *  msg its sentence from rmTokenTable (rm_sender_policy.h), the one place the RM error texts live */
static void rmAnswer(const char *err)
{
    send_http_header(err == nullptr ? 200 : 422, RESPONSE_TYPE_JSON);
    if (err == nullptr)
        web_client.println("{\"ok\":true}");
    else
    {
        web_client.print("{\"ok\":false,\"err\":");
        rm_json_str(err);
        web_client.print(",\"msg\":");
        rm_json_str(rmErrTokenMessage(err));
        web_client.println("}");
    }
}

/* ---- POST /rmpasswd ----------------------------------------------------------------------------- */

void sub_rm_passwd(long content_length)
{
    char body[RM_BODY_MAX + 1];
    const char *err = rmReadBody(body, sizeof(body), content_length);

    if (err == nullptr)
    {
        const RmPwdReq req = rmParsePasswdBody(body); // req.pw points into body
        if (req.err != nullptr)
            err = req.err;
        else if (req.clear)
            nodePasswdApply(nullptr); // open access again (also re-keys net console and KISS on ESP32)
        else
            nodePasswdApply(req.pw);  // validated by rmValidatePasswordN()
    }

    rm_wipe(body, sizeof(body)); // the password is gone from here on
    rmAnswer(err);
}

/* ---- /rmnodes ----------------------------------------------------------------------------------- */

void sub_rm_nodes_get(void)
{
    RmNodes ns;               // local, ~145 B, holds keys: scrubbed below
    rmNodesLoad(ns);          // false = nothing valid, ns zeroed (every slot unused)

    send_http_header(200, RESPONSE_TYPE_JSON);
    web_client.print("{\"nodes\":[");
    for (int i = 0; i < RM_NODES_SLOTS; i++)
    {
        web_client.printf("%s{\"slot\":%d,\"used\":%d,\"call\":", i ? "," : "", i, ns.slot[i].used ? 1 : 0);
        rm_json_str(ns.slot[i].used ? ns.slot[i].call : "");
        web_client.print("}");
    }
    web_client.println("]}");

    rmNodesScrub(ns); // keys never leave this function, not even into the answer
}

void sub_rm_nodes_post(long content_length)
{
    char body[RM_BODY_MAX + 1];
    const char *err = rmReadBody(body, sizeof(body), content_length);

    if (err == nullptr)
    {
        const RmNodesReq req = rmParseNodesBody(body); // call/pw point into body
        if (req.err != nullptr)
            err = req.err;
        else if (req.act == RMN_FORGET)
        {
            RmNodes old; // forget every saved target BEFORE the store is wiped
            if (rmNodesLoad(old))
                for (int i = 0; i < RM_NODES_SLOTS; i++)
                    if (old.slot[i].used)
                        rmForgetTarget(old.slot[i].call);
            rmNodesScrub(old);
            rmNodesWipe();
        }
        else
        {
            RmNodes ns; // local, never static: holds the keys of all saved nodes
            const bool loaded = rmNodesLoad(ns); // false = empty or unreadable: ns is zeroed, fail closed

            if (req.act == RMN_DEL)
            {
                if (loaded && ns.slot[req.slot].used) // deleting an empty slot is a no-op, not an error
                {
                    rmForgetTarget(ns.slot[req.slot].call);
                    rmNodesScrubBuf(&ns.slot[req.slot], sizeof(ns.slot[req.slot]));
                    if (!rmNodesSave(ns))
                    {
                        err = "store";
                        rmNodesStoreFailLog();
                    }
                }
            }
            else // RMN_SAVE
            {
                for (int i = 0; i < RM_NODES_SLOTS; i++)
                    if (i != req.slot && ns.slot[i].used && strcmp(ns.slot[i].call, req.call) == 0)
                        err = "dup"; // one call, one slot

                if (err == nullptr)
                {
                    RmNodeSlot &s = ns.slot[req.slot];
                    // the call of an existing slot changes: the old target is forgotten; only the password
                    // changing keeps everything (the proof book sees the new key on the next send)
                    if (s.used && strcmp(s.call, req.call) != 0)
                        rmForgetTarget(s.call);
                    rmNodesScrubBuf(&s, sizeof(s));
                    strncpy(s.call, req.call, sizeof(s.call) - 1); // rest stays zero (rmNodesCallValid)
                    rmDeriveKey(req.pw, s.key);                    // K = SHA-256(password), straight from the body
                    s.used = true;
                    if (!rmNodesSave(ns))
                    {
                        err = "store";
                        rmNodesStoreFailLog();
                    }
                }
            }
            rmNodesScrub(ns);
        }
    }

    rm_wipe(body, sizeof(body));
    rmAnswer(err);
}

bool rmSendBySlot(int slot, const char *cmd, const char *args, char *err, size_t errN, uint32_t *ctrOut, bool *viaSync,
                  bool force, const char *expectCall)
{
    bool ok = false;
    RmNodes ns; // local: scrubbed on every path

    if (slot < 0 || slot >= RM_NODES_SLOTS || !rmNodesLoad(ns) || !ns.slot[slot].used ||
        (expectCall != nullptr && expectCall[0] != '\0' && !rmCallEq(expectCall, ns.slot[slot].call))) // guard: nothing derived, no counter
    {
        if (err != nullptr && errN > 0)
            snprintf(err, errN, "slot");
    }
    else // rmSendCommandKey() copies dst and the key it keeps; the pointers into ns are used only during the call
        ok = rmSendCommandKey(ns.slot[slot].call, ns.slot[slot].key, cmd, args, err, errN, ctrOut, viaSync, force);

    rmNodesScrub(ns);
    return ok;
}

bool rmSlotTargetInfo(int slot, uint32_t *retryS, bool *canForce, char *callOut, size_t callN)
{
    if (callOut != nullptr && callN > 0)
        callOut[0] = '\0';
    bool allowed = false;
    if (retryS != nullptr)
        *retryS = 0;
    if (canForce != nullptr)
        *canForce = false;
    RmNodes ns; // local: scrubbed on every path
    if (slot >= 0 && slot < RM_NODES_SLOTS && rmNodesLoad(ns) && ns.slot[slot].used)
    {
        allowed = rmTargetMaySend(ns.slot[slot].call, retryS, canForce); // the SAME decision as the send path
        if (callOut != nullptr && callN > 0)
            snprintf(callOut, callN, "%s", ns.slot[slot].call); // the call only, never the password
    }
    rmNodesScrub(ns);
    return allowed;
}

/* ---- GET /rmheard ------------------------------------------------------------------------------- */

void sub_rm_heard(void)
{
    const uint16_t now_min = uptimeMin16();
    uint8_t idx[RM_HEARD_SCAN];
    int total = nbrMhRows(nbrMatrix, now_min, RM_HEARD_WINDOW_MIN, idx, RM_HEARD_SCAN); // newest first
    if (total > RM_HEARD_SCAN)
        total = RM_HEARD_SCAN;

    send_http_header(200, RESPONSE_TYPE_JSON);
    web_client.print("{\"heard\":[");
    int shown = 0;
    for (int k = 0; k < total && shown < RM_HEARD_MAX; k++)
    {
        NbrMhView v;
        if (!nbrMhGet(nbrMatrix, idx[k], now_min, &v))
            continue;
        if (!rmValidateCall(v.call)) // a chip that RM could never address is no use to the page
            continue;

        web_client.printf("%s{\"call\":", shown ? "," : "");
        rm_json_str(v.call);
        web_client.print(",\"hw\":");
        rm_json_str(nbrHardwareName(v.hw));
        web_client.printf(",\"age_s\":%lu,\"rssi\":", (unsigned long)v.age_min * 60UL);
        if (v.rssi == NBR_MH_RSSI_UNKNOWN)
            web_client.print("null");
        else
            web_client.printf("%d", (int)v.rssi);
        web_client.print("}");
        shown++;
    }
    web_client.println("]}");
}
