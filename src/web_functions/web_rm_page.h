/**
 * RM web GUI W2b: the Remote page. The orchestrator (G2) wires the route /?page=remote to sub_page_remote(),
 * calls rmScaffoldJs() once inside deliver_scaffold()'s <script> block, and hooks rmPageInit()/rmPageLeave()
 * into the scaffold's loadPage().
 */
#ifndef _WEB_RM_PAGE_H_
#define _WEB_RM_PAGE_H_

/** prints the page skeleton (sub-header, inline style, static elements), HTML only, no script */
void sub_page_remote();

/** prints the scaffold JS of the Remote page (every call at most 512 bytes); no <script> tags */
void rmScaffoldJs();

#endif // _WEB_RM_PAGE_H_
