/**
 * RM web GUI W2b: the Remote page (GET /?page=remote) -- HTML skeleton and the scaffold JS that drives it.
 * Contract: docs/rm-gui/impl-plan.md (C4), look and rules: docs/rm-gui/verdict-ux.md, verdict-webtests.md.
 *
 * Rules the code keeps:
 *  - Sub-pages are injected with innerHTML, scripts in them never run: sub_page_remote() prints HTML only. Its
 *    CSS is the tail of web/mc.css, its JS is web/mcrm.js; both are gzip blobs served by web_functions.cpp
 *    (tools/gen_web_assets.py makes web_assets.h from them) and loaded in the head of the scaffold.
 *  - The only thing printed here is the rmLen line of rmScaffoldJs(), because the limits come from rm_commands.h.
 *    Every web_client call is a print/println of ONE string literal of at most 512 bytes (RAK4631: one write is
 *    capped at 2048 B and the rest dropped; nRF52 printf over-reads above 255 B); the printf has two %u far below
 *    64 B, so Print::printf stays on the stack. mcrm.js reads rmLen while it loads, so rmLen is printed first.
 *  - The page owns no rule text of its own: error sentences come from the server (msg next to err), the
 *    text limits from rmLen. The JS only pre-checks lengths and the call-sign shape.
 *  - The JS writes all server data with textContent. Passwords go in POST bodies only, the input is cleared
 *    when the request is built, nothing is stored in the browser.
 *  - tools/webgui_rm_test.js runs web/mcrm.js in jsdom; the rmLen arguments must be (unsigned)RM_NAME_MAX /
 *    RM_ATXT_MAX / RM_MAX_ARGS, the harness resolves them from the headers.
 *  - Ids are unique per page: read-only values use rm_v_<card>_<key>, inputs and their Set/cnt/hint use
 *    rm_f_<card>_<key>. rmWUpd, rmWire, rmClrIn and rmIn look up rm_f_ only; a shared id makes rmEl() return
 *    the read-only <div> instead of the input (D5, docs/rm-gui/ux-tidy-plan-20261010.md 3.2).
 *  - The CSS/JS sources may carry full-line comments: the generator strips them before it compresses.
 */
#include <Arduino.h>
#include "web_rm_page.h"
#include "web_rm_util.h"
#include "web_functions.h"
#include "../rm_commands.h" // RM_NAME_MAX, RM_ATXT_MAX
#include "../remote_cmd.h"  // RM_MAX_ARGS

/** Remote page body: the sub-header, then the static skeleton. The elements are filled by the scaffold JS.
 *  Every card uses the shared collapsablecard markup (CSS and togglecard() live in web_functions.cpp): teaser span,
 *  toggle button and exactly one body div. The class names are short and unique to this page, so no id prefix. */
void sub_page_remote()
{
    _create_meshcom_subheader("Remote");
    web_client.println("<div id=\"content_inner\">\n"
                       "<div id=\"rm_page\">\n"
                       "<p id=\"rm_auth\" class=\"rmbad\" style=\"display:none\"></p>\n"
                       "<p id=\"rm_net\" class=\"font-small\"></p>\n"
                       "<p id=\"rm_msg\" class=\"rmmsg\"></p>\n"
                       "<p id=\"rm_force\" class=\"rmmsg\"></p>\n"
                       "<p id=\"rm_chain\" class=\"font-small\"></p>\n"
                       "<p id=\"rm_off\" class=\"font-small\" style=\"display:none\"></p>");
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Choose a node</label>\n"
                       "<span>Pick the node to manage.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div>");
    // One sprite for the padlock (closed = key missing, open = key saved), referenced by every tile.
    web_client.println("<svg id=\"rm_sprite\" width=\"0\" height=\"0\" style=\"position:absolute\" aria-hidden=\"true\"><symbol id=\"rmlk-c\" viewBox=\"0 0 12 12\"><path d=\"M2 5h8v6H2zM3 5V4a3 3 0 016 0v1H8V4a2 2 0 00-4 0v1z\"/></symbol>"
                       "<symbol id=\"rmlk-o\" viewBox=\"0 0 12 12\"><path d=\"M2 5h8v6H2zM3 4V3a3 3 0 016 0v1H8V3a2 2 0 00-4 0v1z\"/></symbol></svg>");
    web_client.println("<p class=\"font-small\">Nodes heard directly in the last 3 hours and nodes with a saved key. Remote management may be switched off on any of them.</p>\n"
                       "<div id=\"rm_nodes\" class=\"rmchips\"></div>\n"
                       "<div class=\"rmrow rmsplit\"><button type=\"button\" data-act=\"heard\">Refresh list</button><button type=\"button\" id=\"rm_forgetall\" data-act=\"forgetall\">Forget all</button></div>");
    web_client.println("<div class=\"rmrow\"><input type=\"text\" id=\"rm_call\" maxlength=\"9\" autocomplete=\"off\" autocapitalize=\"characters\" placeholder=\"Call sign, for example DK5EN-12\" aria-label=\"Call sign of the node\"></div>\n"
                       "<p id=\"rm_callchk\" class=\"font-small\"></p>");
    web_client.println("<div id=\"rm_pwrow\" class=\"rmrow\"><input type=\"password\" id=\"rm_pw\" maxlength=\"14\" autocomplete=\"new-password\" placeholder=\"Password of that node\" aria-label=\"Password of the node\"><button type=\"button\" data-act=\"remember\">Remember</button></div>\n"
                       "<p id=\"rm_savednote\" class=\"font-small\"></p>");
    web_client.println("</div></div>\n");
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Messages</label>\n"
                       "<span>Commands sent from this page.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div><table class=\"rmtab font-small\"><thead><tr><th>time</th><th>command</th><th>state</th><th>reply</th></tr></thead><tbody id=\"rm_msgs\"></tbody></table></div></div>\n");
    web_client.println("<div class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Actions</label>\n"
                       "<span>Status, switches, restart.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div><div id=\"rm_info\" class=\"rmtiles\"></div><div id=\"rm_sw\" class=\"grid rmg\"></div><p id=\"rm_swnote\" class=\"font-small\"></p></div></div>\n");
    web_client.println("<div id=\"rm_card_radio\" class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Radio</label>\n"
                       "<span>Radio settings and sensors.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_ident\" class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Identity</label>\n"
                       "<span>Name and APRS text.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_pos\" class=\"cardlayout collapsablecard cardopen\"><label class=\"cardlabel\">Position</label>\n"
                       "<span>Position of the node.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_queues\" class=\"cardlayout collapsablecard\"><label class=\"cardlabel\">Queues</label>\n"
                       "<span>Open this for TX queue, mailbox and max hop.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_card_mh\" class=\"cardlayout collapsablecard\"><label class=\"cardlabel\">Heard list</label>\n"
                       "<span>Open this for the heard list.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div></div></div>\n");
    web_client.println("<div id=\"rm_adv\" class=\"cardlayout collapsablecard\"><label class=\"cardlabel\">Advanced</label>\n"
                       "<span>Output pin, counters, command log.</span>\n"
                       "<button class=\"cardtoggle\" onclick=\"togglecard(this)\"><i></i></button>\n"
                       "<div>\n"
                       "<p class=\"font-small\">Switch an output pin of the node (only works if the pin is set as an output there).</p>");
    web_client.println("<div class=\"rmrow\"><select id=\"rm_pin\" aria-label=\"Output pin\"></select><label for=\"rm_pinsw\">Output</label><input type=\"checkbox\" role=\"switch\" class=\"rmsw\" id=\"rm_pinsw\" data-sw=\"pin\" disabled></div>\n"
                       "<div id=\"rm_card_pins\"><div></div></div>\n"
                       "<p id=\"rm_cnt\" class=\"font-small\"></p>");
    web_client.println("<p class=\"font-small\">Commands run on this node</p>\n"
                       "<table class=\"rmtab font-small\"><thead><tr><th>ago</th><th>from</th><th>ctr</th><th>command</th><th>result</th></tr></thead><tbody id=\"rm_log\"></tbody></table>\n"
                       "</div></div>\n"
                       "</div>\n"
                       "</div>");
}

/** Inline part of the Remote page scripts: the rmLen limits. The rest is web/mcrm.js (rmPageInit() / rmPageLeave() are
 *  called by loadPage(), all global names start with rm). Called once from deliver_scaffold(), inside its <script>. */
void rmScaffoldJs()
{
    web_client.printf("var rmLen={name:%u,atxt:%u,args:%u};\n", (unsigned)RM_NAME_MAX, (unsigned)RM_ATXT_MAX, (unsigned)RM_MAX_ARGS);
#if defined(MC_AU_DEV_CHANNEL)
    web_client.println("var rmUpdChan=1;");
#else
    web_client.println("var rmUpdChan=0;");
#endif
}
