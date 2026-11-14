// TD-THEME-01: T-Deck LVGL theme (shared styles, live dark/light switch). T-Deck and T-Deck Plus only.
// The pure part (palette, resolve, parse) is in tdeck_palette.h and is host-tested.
#ifndef TDECK_THEME_H
#define TDECK_THEME_H

#if defined(BOARD_T_DECK) || defined(BOARD_T_DECK_PLUS)

#include <lvgl.h>
#include "tdeck_palette.h"

// Fills the shared styles from the resolved palette, installs the LVGL default theme on disp.
void tdeck_theme_init(lv_disp_t* disp);
// DAY<->NIGHT transition (TD-23 decision). Re-applies only when the resolved dark/light result changes.
void tdeck_theme_set_night(bool night);
// Resolve from meshcom_settings.node_tdeck_theme + the stored night state and apply live.
// Before tdeck_theme_init() it only records state, so the command may call it any time.
void tdeck_theme_reapply();
bool tdeck_theme_is_dark();
const TdeckPalette& tdeck_theme_pal();
// Up to 4 callbacks, all run (in registration order) after every apply, so runtime-coloured widgets can
// re-colour themselves. Returns true if cb is registered afterwards (a duplicate counts as registered),
// false for NULL or when all 4 slots are taken.
bool tdeck_theme_on_change(void (*cb)(void));

extern lv_style_t tdeck_style_header, tdeck_style_header_text, tdeck_style_tab, tdeck_style_tab_sel,
    tdeck_style_screen, tdeck_style_surface, tdeck_style_text, tdeck_style_muted, tdeck_style_btn_on,
    tdeck_style_btn_off, tdeck_style_bubble_in, tdeck_style_bubble_out;

#endif
#endif
