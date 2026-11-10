/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* SeqAdv milestone 0: hardware test page inside the stock Open firmware.
 *
 * SHIFT + top key (C, key 37) toggles the page. While it is open the stock
 * instrument receives no panel events and stays silent. The firmware update
 * path (SysEx transfer) is untouched and keeps running.
 *
 *   Pad 1 / 2 / 3   test note (ch 1, C4) on DIN / USB / BLE while held
 *   Any control     shown as the last raw panel event
 *   LOOP            control-loop interval, min/max over one second
 *
 * Included at the end of screen.c to reuse its 5x7 font and band buffer.
 * Control task only. */
#ifdef FM1_OPEN_WEB
/* The browser simulator has no hardware counter: derive it from milliseconds. */
static uint32_t m0_ticks(void) { return fm1_millis() * 24000U; }
#else
#include "fm1_device.h"
static uint32_t m0_ticks(void) { return fm1_device_ticks(); }
#endif

enum { M0_LINES = 12, M0_COLUMNS = 20, M0_SCALE = 2, M0_ROW = 20 };
static struct {
    uint8_t active, shift, dirty_all;
    uint8_t kind, id; int16_t value; uint32_t events;
    uint16_t sent[3], failed[3];
    uint32_t last_ticks, window_start;
    uint32_t min_us, max_us, shown_min_us, shown_max_us, loops, shown_loops;
    char shown[M0_LINES][M0_COLUMNS + 1];
} m0;

int seqadv_m0_active(void) { return m0.active; }

static void m0_note(unsigned port, unsigned on)
{
    uint8_t message[3] = {(uint8_t)(on ? 0x90 : 0x80), 60, (uint8_t)(on ? 100 : 0)};
    if (fm1_midi_send_port(port, message, 3) == 0) { if (on) ++m0.sent[port]; }
    else if (on) ++m0.failed[port];
}

/* Called first by smk37_control_event. Returns 1 when the event is consumed. */
int seqadv_m0_event(unsigned kind, unsigned id, int value)
{
    if (kind == SMK37_BUTTON && id == 0) m0.shift = value != 0;
    if (kind == SMK37_KEY && id == 36 && value && m0.shift) {
        if (!m0.active) {
            smk37_controls_release();       /* no hanging notes in the instrument */
            memset(m0.shown, 0, sizeof m0.shown);
            m0.dirty_all = 1; m0.min_us = ~0U; m0.max_us = 0; m0.loops = 0;
            m0.last_ticks = m0_ticks(); m0.window_start = fm1_millis();
            m0.active = 1;
        } else {
            for (unsigned port = 0; port < 3; ++port) m0_note(port, 0);
            m0.active = 0;
            saved_cache.shown_page = ~0U;   /* stock page repaints in full */
        }
        return 1;
    }
    if (!m0.active) return 0;
    m0.kind = (uint8_t)kind; m0.id = (uint8_t)id; m0.value = (int16_t)value; ++m0.events;
    if (kind == SMK37_PAD && id < 3) m0_note(id, value != 0);
    return 1;
}

static char *m0_put(char *out, const char *s) { while (*s) *out++ = *s++; *out = 0; return out; }
static char *m0_number(char *out, long value)
{
    char digits[12]; unsigned n = 0;
    unsigned long v = value < 0 ? (unsigned long)-value : (unsigned long)value;
    if (value < 0) *out++ = '-';
    do { digits[n++] = (char)('0' + v % 10U); v /= 10U; } while (v);
    while (n) *out++ = digits[--n];
    *out = 0; return out;
}

/* Blocking draw of one text row. Returns 0 when the LCD is still busy. */
static int m0_line(unsigned line, const char *s, uint16_t color)
{
    if (!m0.dirty_all && !strcmp(m0.shown[line], s)) return 1;
    if (fm1_screen_pending()) return 0;
    enum { W = FM1_SCREEN_WIDTH, H = 7 * M0_SCALE };
    for (unsigned i = 0; i < W * H; ++i) paint_pixels[i] = BACK;
    unsigned x = 0;
    for (const char *p = s; *p && x + 6U * M0_SCALE <= W; ++p, x += 6U * M0_SCALE) {
        char letter = *p >= 'a' && *p <= 'z' ? (char)(*p - 'a' + 'A') : *p;
        const uint8_t *shape = NULL;
        if (*p >= '0' && *p <= '9') shape = font[*p - '0'];
        else if (letter >= 'A' && letter <= 'Z') shape = font[10 + letter - 'A'];
        else if ((unsigned char)*p >= '!' && (unsigned char)*p <= '~') {
            unsigned c = (unsigned char)*p;
            shape = punctuation[c - (c < ':' ? 33U : c < '[' ? 43U : c < '{' ? 69U : 95U)];
        }
        if (!shape) continue;
        for (unsigned row = 0; row < H; ++row)
            for (unsigned col = 0; col < 5U * M0_SCALE; ++col)
                if (shape[col / M0_SCALE] & (1U << (row / M0_SCALE)))
                    paint_pixels[row * W + x + col] = color;
    }
    if (fm1_screen_blit(0, 4U + line * M0_ROW, W, H, paint_pixels, W * H)) return 0;
    strncpy(m0.shown[line], s, M0_COLUMNS); m0.shown[line][M0_COLUMNS] = 0;
    return 1;
}

/* Called from groove_screen_control after the panel has been read. */
static void m0_frame(void)
{
    uint32_t ticks = m0_ticks(), now = fm1_millis();
    uint32_t us = (ticks - m0.last_ticks) / 24U;       /* 24 MHz counter */
    m0.last_ticks = ticks;
    if (us < m0.min_us) m0.min_us = us;
    if (us > m0.max_us) m0.max_us = us;
    ++m0.loops;
    if (now - m0.window_start >= 1000U) {
        m0.shown_min_us = m0.min_us; m0.shown_max_us = m0.max_us; m0.shown_loops = m0.loops;
        m0.min_us = ~0U; m0.max_us = 0; m0.loops = 0; m0.window_start = now;
    }
    if (m0.dirty_all) {
        if (fm1_screen_pending()) return;
        fm1_screen_fill(0, 0, FM1_SCREEN_WIDTH, FM1_SCREEN_HEIGHT, BACK);
    }
    static const char *const kinds[8] = {"BTN", "KEY", "PRS", "PAD", "ENC", "CMB", "PED", "FDR"};
    static char text_line[M0_LINES][40]; char *p; unsigned ok = 1;
    m0_put(text_line[0], "SEQADV M0 TEST");
    p = m0_put(text_line[1], "LAST "); p = m0_put(p, m0.events ? kinds[m0.kind & 7U] : "---");
    *p++ = ' '; p = m0_number(p, m0.id); *p++ = ' '; m0_number(p, m0.value);
    p = m0_put(text_line[2], "EVENTS "); m0_number(p, (long)m0.events);
    p = m0_put(text_line[3], "FLASH "); m0_number(p, (long)fm1_flash_size());
    p = m0_put(text_line[4], "OUT "); p = m0_number(p, (long)fm1_midi_output_ports());
    p = m0_put(p, " RDY "); m0_number(p, (long)fm1_midi_ready_ports());
    p = m0_put(text_line[5], "LOOPS/S "); m0_number(p, (long)m0.shown_loops);
    p = m0_put(text_line[6], "MIN US "); m0_number(p, (long)m0.shown_min_us);
    p = m0_put(text_line[7], "MAX US "); m0_number(p, (long)m0.shown_max_us);
    static const char *const ports[3] = {"DIN ", "USB ", "BLE "};
    for (unsigned port = 0; port < 3; ++port) {
        p = m0_put(text_line[8 + port], "PAD"); *p++ = (char)('1' + port); *p++ = ' ';
        p = m0_put(p, ports[port]); p = m0_number(p, m0.sent[port]);
        p = m0_put(p, " ERR "); m0_number(p, m0.failed[port]);
    }
    m0_put(text_line[11], "SHIFT+TOP KEY EXIT");
    for (unsigned line = 0; line < M0_LINES && ok; ++line)
        ok = (unsigned)m0_line(line, text_line[line], line == 0 ? ACCENT : line == 11 ? DIM : TEXT);
    if (ok) m0.dirty_all = 0;
}
