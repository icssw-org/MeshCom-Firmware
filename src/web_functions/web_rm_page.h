/**
 * RM web GUI W2b: the Remote page. The orchestrator (G2) wires the route /?page=remote to sub_page_remote(),
 * calls rmScaffoldJs() once inside deliver_scaffold()'s inline <script>, and hooks rmPageInit()/rmPageLeave()
 * into the scaffold's loadPage().
 */
#ifndef _WEB_RM_PAGE_H_
#define _WEB_RM_PAGE_H_

/** prints the page skeleton (sub-header, static elements), HTML only, no script; the CSS is in web/mc.css */
void sub_page_remote();

/** prints the rmLen limits for web/mcrm.js (served as /mcrm.js); no <script> tags */
void rmScaffoldJs();

#endif // _WEB_RM_PAGE_H_
