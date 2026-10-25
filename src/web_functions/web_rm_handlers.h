/**
 * RM GUI W2a -- server side of the Remote page.
 *
 * All handlers run in the loop task like the rest of the web server (the key store and the sender
 * API of rm_runtime.h are loop-task only). Called by the route chain in web_functions.cpp (G2):
 *
 *   POST /rmpasswd  -> sub_rm_passwd(content_length)
 *   GET  /rmnodes   -> sub_rm_nodes_get()
 *   POST /rmnodes   -> sub_rm_nodes_post(content_length)
 *   GET  /rmheard   -> sub_rm_heard()
 *   POST /rmsend with slot=  -> rmParseSendBody() (web_rm_parse.h), then rmSendBySlot()
 *
 * Every state-changing request is a POST body, parsed in place (web_rm_parse.h), never echoed, and the
 * body buffer is wiped (rm_wipe) before the handler answers. Answers are JSON: 200 {"ok":true} or
 * 422 {"ok":false,"err":"<token>"}; tokens: size short form act slot call pw cmd dup store.
 */
#ifndef _WEB_RM_HANDLERS_H_
#define _WEB_RM_HANDLERS_H_

#include <stddef.h>
#include <stdint.h>

/** Reads a POST body of content_length bytes from web_client into buf (cap bytes incl. NUL) with an
 *  absolute deadline (a slow drip must not stall the loop task into the task WDT). Returns nullptr on
 *  success, else a static token: "size" (content_length <= 0 or does not fit) or "short" (the peer
 *  stopped early). The buffer is NUL-terminated in every case. */
const char *rmReadBody(char *buf, size_t cap, long content_length);

/** POST /rmpasswd   act=set&pw=<pw>  |  act=clear   -> nodePasswdApply() */
void sub_rm_passwd(long content_length);

/** GET /rmnodes   {"nodes":[{"slot":0,"used":1,"call":"DK5EN-1"},...3 entries]}   never a key */
void sub_rm_nodes_get(void);

/** POST /rmnodes   act=save&slot=0..2&call=<call>&pw=<pw>  |  act=del&slot=n  |  act=forget */
void sub_rm_nodes_post(long content_length);

/** GET /rmheard   {"heard":[{"call":"","hw":"HELTEC_V3","age_s":120,"rssi":-95},...]}  at most 12 */
void sub_rm_heard(void);

/** Sends cmd (wire form: lower case) and args (as typed) to the node saved in slot 0..2 with that slot's
 *  key. Loads the store into a local struct, hands the key to rmSendCommandKey(), scrubs the struct on
 *  every path. expectCall (optional, nullptr or "" = no guard): the send is refused unless the slot
 *  currently holds exactly that call (case-insensitive), before any key is derived or counter used.
 *  false + err token: "slot" (out of range, unused or guard mismatch), else the tokens of
 *  rmSendCommandKey() ("cmd", "busy", "limit", ...). */
bool rmSendBySlot(int slot, const char *cmd, const char *args, char *err, size_t errN, uint32_t *ctrOut, bool *viaSync,
                  bool force = false, const char *expectCall = nullptr);

/** Policy probe for a saved slot (for the /rmsend refusal reply): returns true when a send would pass now;
 *  retryS = seconds until it would, canForce = refused with LIMIT and the one-shot after a re-key is armed.
 *  Same decision as the send path (rmTargetMaySend -> policyFor). false + zeros for an empty/invalid slot. */
bool rmSlotTargetInfo(int slot, uint32_t *retryS, bool *canForce, char *callOut = nullptr, size_t callN = 0);

#endif // _WEB_RM_HANDLERS_H_
