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
static uint32_t m0_lock(void) { return 0; }
static void m0_unlock(uint32_t state) { (void)state; }
#else
#include "fm1_device.h"
static uint32_t m0_ticks(void) { return fm1_device_ticks(); }
static uint32_t m0_lock(void) { return fm1_device_irq_save(); }
static void m0_unlock(uint32_t state) { fm1_device_irq_restore(state); }
#endif

#include "seqadv_probe.h"
struct seqadv_probe seqadv_probes[SEQADV_PROBES];
volatile uint32_t seqadv_wake_time;
volatile uint8_t seqadv_wake_armed, seqadv_wake_pending;
struct seqadv_clock seqadv_clock;
volatile uint8_t seqadv_stock_clock_off;
volatile uint8_t seqadv_phase;

enum { M0_LINES = 12, M0_COLUMNS = 20, M0_SCALE = 2, M0_ROW = 20 };
static struct {
    uint8_t active, shift, dirty_all, page, clear, stress, note;
    uint16_t dirty_lines;               /* one bit per text row still to draw */
    /* Clock test: lateness = send time - due time, in 24 MHz ticks. */
    uint32_t consumed, late_min, late_max, late_sum, late_count, late_peak, late_over, overruns, clock_errors;
    uint32_t shown_late_min, shown_late_max, shown_late_avg;
    uint32_t over_phase[3], phase_max[3];   /* late ticks by activity; longest read / draw */
    uint32_t probe_count[SEQADV_PROBES], probe_min[SEQADV_PROBES], probe_max[SEQADV_PROBES];
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

static void m0_send(const uint8_t *message, size_t bytes)
{
    /* DIN and USB; a full queue is counted, never retried late. */
    for (unsigned port = 0; port < 2; ++port)
        if (fm1_midi_send_port(port, message, bytes) != 0 && port == 0) ++m0.clock_errors;
}
static void m0_clock_tempo(unsigned bpm)
{
    /* 96 PPQN: 24 MHz * 60 / (bpm * 96) = 15,000,000 / bpm counter ticks. */
    uint32_t state = m0_lock();
    seqadv_clock.bpm = bpm; seqadv_clock.period = 15000000U / bpm; seqadv_clock.rem = 15000000U % bpm;
    seqadv_clock.acc = 0;
    m0_unlock(state);
}
static void m0_clock_run(unsigned run)
{
    if (!seqadv_clock.bpm) m0_clock_tempo(120);
    if (run && !seqadv_clock.running) {
        const uint8_t start = 0xfa;
        m0.late_min = ~0U; m0.late_max = m0.late_sum = m0.late_count = 0;
        m0.late_peak = m0.late_over = m0.overruns = m0.clock_errors = 0;
        memset(m0.over_phase, 0, sizeof m0.over_phase); memset(m0.phase_max, 0, sizeof m0.phase_max);
        m0_send(&start, 1);
        uint32_t state = m0_lock();
        seqadv_clock.produced = 0; m0.consumed = 0; seqadv_clock.acc = 0;
        seqadv_clock.next = m0_ticks() + seqadv_clock.period;
        seqadv_clock.running = 1;
        m0_unlock(state);
    } else if (!run && seqadv_clock.running) {
        const uint8_t stop = 0xfc;
        seqadv_clock.running = 0;
        if (m0.note) { uint8_t off[3] = {0x80, m0.note, 0}; m0_send(off, 3); m0.note = 0; }
        m0_send(&stop, 1);
    }
}
/* Enter an activity phase; returns the time, and records how long the
 * previous phase (1 panel read, 2 draw) lasted. */
static uint32_t m0_phase(unsigned phase, uint32_t started)
{
    uint32_t now = m0_ticks(), previous = seqadv_phase;
    if (previous && seqadv_clock.running && now - started > m0.phase_max[previous]) m0.phase_max[previous] = now - started;
    seqadv_phase = (uint8_t)phase;
    return now;
}
/* Control task, first thing on every wake: send what is due. MIDI clock on
 * every 4th tick (24 PPQN), a metronome note on channel 1 every beat. */
static void m0_clock_service(void)
{
    while (seqadv_clock.running && m0.consumed != seqadv_clock.produced) {
        uint32_t behind = seqadv_clock.produced - m0.consumed;
        if (behind > 8U) { m0.overruns += behind - 8U; m0.consumed += behind - 8U; }
        uint32_t tick = m0.consumed++;
        uint32_t late = m0_ticks() - seqadv_clock.due[tick & 7U];
        if (late < m0.late_min) m0.late_min = late;
        if (late > m0.late_max) m0.late_max = late;
        if (late > m0.late_peak) m0.late_peak = late;
        if (late > 24000U) { ++m0.late_over; ++m0.over_phase[seqadv_clock.phase[tick & 7U] % 3U]; }
        m0.late_sum += late; ++m0.late_count;
        if (tick % 96U == 24U && m0.note) { uint8_t off[3] = {0x80, m0.note, 0}; m0_send(off, 3); m0.note = 0; }
        if (tick % 4U == 0) { const uint8_t clock = 0xf8; m0_send(&clock, 1); }
        if (tick % 96U == 0) {
            m0.note = (uint8_t)(tick % 384U == 0 ? 72 : 60);
            uint8_t on[3] = {0x90, m0.note, 100}; m0_send(on, 3);
        }
    }
}

/* Called first by smk37_control_event. Returns 1 when the event is consumed. */
int seqadv_m0_event(unsigned kind, unsigned id, int value)
{
    if (kind == SMK37_BUTTON && id == 0) m0.shift = value != 0;
    if (kind == SMK37_KEY && id == 36 && value && m0.shift) {
        if (!m0.active) {
            smk37_controls_release();       /* no hanging notes in the instrument */
            memset(m0.shown, 0, sizeof m0.shown);
            m0.dirty_all = m0.clear = 1; m0.min_us = ~0U; m0.max_us = 0; m0.loops = 0;
            m0.last_ticks = m0_ticks(); m0.window_start = fm1_millis();
            m0.active = 1; seqadv_stock_clock_off = 1;
        } else {
            for (unsigned port = 0; port < 3; ++port) m0_note(port, 0);
            m0_clock_run(0);
            m0.active = 0; seqadv_wake_armed = 0; m0.stress = 0; seqadv_stock_clock_off = 0;
            saved_cache.shown_page = ~0U;   /* stock page repaints in full */
        }
        return 1;
    }
    if (!m0.active) return 0;
    m0.kind = (uint8_t)kind; m0.id = (uint8_t)id; m0.value = (int16_t)value; ++m0.events;
    if (kind == SMK37_PAD && id < 3 && m0.page == 0) m0_note(id, value != 0);
    /* Pad 16 cycles the pages: ports, timing, clock. Holding pad 15 arms the
     * wake test. Pad 5 starts and stops the clock, holding pad 14 forces
     * continuous screen redraws, any encoder sets the tempo. */
    if (kind == SMK37_PAD && id == 15 && value) { m0.page = (uint8_t)((m0.page + 1U) % 3U); m0.dirty_all = 1; }
    if (kind == SMK37_PAD && id == 14) seqadv_wake_armed = value != 0;
    if (kind == SMK37_PAD && id == 13) m0.stress = value != 0;
    if (kind == SMK37_PAD && id == 4 && value) m0_clock_run(!seqadv_clock.running);
    if (kind == SMK37_ENCODER && value) {
        if (!seqadv_clock.bpm) m0_clock_tempo(120);
        int bpm = (int)seqadv_clock.bpm + value;
        m0_clock_tempo((unsigned)(bpm < 30 ? 30 : bpm > 300 ? 300 : bpm));
    }
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
    if (!(m0.dirty_lines & (1U << line)) && !strcmp(m0.shown[line], s)) return 1;
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
    /* Asynchronous DMA: returns at once and borrows the buffer until
     * fm1_screen_pending() reports completion, so one call stays short. */
    if (fm1_screen_submit(0, 4U + line * M0_ROW, W, H, paint_pixels, W * H)) return 0;
    strncpy(m0.shown[line], s, M0_COLUMNS); m0.shown[line][M0_COLUMNS] = 0;
    m0.dirty_lines &= (uint16_t)~(1U << line);
    return 1;
}

/* Called from groove_screen_control after the panel has been read. */
static void m0_frame(void)
{
    uint32_t ticks = m0_ticks(), now = fm1_millis();
    if (seqadv_wake_pending) {
        seqadv_probe_interval(SEQADV_WAKE, ticks - seqadv_wake_time);
        seqadv_wake_pending = 0;
    }
    uint32_t us = (ticks - m0.last_ticks) / 24U;       /* 24 MHz counter */
    m0.last_ticks = ticks;
    if (us < m0.min_us) m0.min_us = us;
    if (us > m0.max_us) m0.max_us = us;
    ++m0.loops;
    if (now - m0.window_start >= 1000U) {
        m0.shown_min_us = m0.min_us; m0.shown_max_us = m0.max_us; m0.shown_loops = m0.loops;
        m0.min_us = ~0U; m0.max_us = 0; m0.loops = 0; m0.window_start = now;
        for (unsigned i = 0; i < SEQADV_PROBES; ++i) {
            struct seqadv_probe *probe = &seqadv_probes[i];
            m0.probe_count[i] = probe->count; m0.probe_min[i] = probe->count ? probe->min / 24U : 0; m0.probe_max[i] = probe->max / 24U;
            probe->count = 0; probe->min = ~0U; probe->max = 0;
        }
        if (m0.late_count) {
            m0.shown_late_min = m0.late_min / 24U; m0.shown_late_max = m0.late_max / 24U;
            m0.shown_late_avg = m0.late_sum / m0.late_count / 24U;
            m0.late_min = ~0U; m0.late_max = m0.late_sum = m0.late_count = 0;
        }
    }
    if (m0.clear) {
        if (fm1_screen_pending()) return;
        fm1_screen_fill(0, 0, FM1_SCREEN_WIDTH, FM1_SCREEN_HEIGHT, BACK);
        m0.clear = 0;
    }
    /* Rows are sent one at a time by DMA, so a full redraw spans several
     * calls: remember which rows are still owed instead of restarting. */
    if (m0.dirty_all) { m0.dirty_lines = 0xfffU; m0.dirty_all = 0; }
    static const char *const kinds[8] = {"BTN", "KEY", "PRS", "PAD", "ENC", "CMB", "PED", "FDR"};
    static char text_line[M0_LINES][40]; char *p; unsigned ok = 1;
    if (m0.page == 2) {
        p = m0_put(text_line[0], "SEQADV M0 CLOCK");
        p = m0_put(text_line[1], seqadv_clock.running ? "RUN  BPM " : "STOP BPM "); m0_number(p, (long)seqadv_clock.bpm);
        p = m0_put(text_line[2], "TICKS "); m0_number(p, (long)m0.consumed);
        p = m0_put(text_line[3], "LATE MIN US "); m0_number(p, (long)m0.shown_late_min);
        p = m0_put(text_line[4], "LATE AVG US "); m0_number(p, (long)m0.shown_late_avg);
        p = m0_put(text_line[5], "LATE MAX US "); m0_number(p, (long)m0.shown_late_max);
        p = m0_put(text_line[6], "PEAK US "); m0_number(p, (long)(m0.late_peak / 24U));
        p = m0_put(text_line[7], "OVER 1MS "); m0_number(p, (long)m0.late_over);
        p = m0_put(text_line[8], "OUT "); p = m0_number(p, (long)m0.over_phase[0]);
        p = m0_put(p, " RD "); p = m0_number(p, (long)m0.over_phase[1]);
        p = m0_put(p, " DRW "); m0_number(p, (long)m0.over_phase[2]);
        p = m0_put(text_line[9], "MAX RD "); p = m0_number(p, (long)(m0.phase_max[1] / 24U));
        p = m0_put(p, " DR "); m0_number(p, (long)(m0.phase_max[2] / 24U));
        p = m0_put(text_line[10], "LOST "); p = m0_number(p, (long)m0.overruns);
        p = m0_put(p, " ERR "); p = m0_number(p, (long)m0.clock_errors);
        m0_put(p, m0.stress ? " STRESS" : "");
        m0_put(text_line[11], "P5 RUN P14 STR P16");
        for (unsigned line = 0; line < M0_LINES && ok; ++line) {
            m0_clock_service();
            ok = (unsigned)m0_line(line, text_line[line], line == 0 ? ACCENT : line >= 10 ? DIM : TEXT);
        }
        if (ok && m0.stress) m0.dirty_lines = 0xfffU;
        return;
    }
    if (m0.page) {
        static const char *const names[SEQADV_PROBES] = {"SCAN/S ", "SOFT/S ", "AUDIO/S ", "POLL/S ", "WAKE/S "};
        m0_put(text_line[0], "SEQADV M0 TIMING");
        for (unsigned i = 0; i < SEQADV_PROBES; ++i) {
            p = m0_put(text_line[1 + 2 * i], names[i]); m0_number(p, (long)m0.probe_count[i]);
            p = m0_put(text_line[2 + 2 * i], " US "); p = m0_number(p, (long)m0.probe_min[i]);
            *p++ = '-'; m0_number(p, (long)m0.probe_max[i]);
        }
        m0_put(text_line[11], "P15 WAKE  P16 PAGE");
        for (unsigned line = 0; line < M0_LINES && ok; ++line)
            ok = (unsigned)m0_line(line, text_line[line], line == 0 ? ACCENT : line == 11 ? DIM : (line & 1U) ? TEXT : DIM);
        if (ok && m0.stress) m0.dirty_lines = 0xfffU;
        return;
    }
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
    m0_put(text_line[11], "P16 TIMING PAGE");
    for (unsigned line = 0; line < M0_LINES && ok; ++line)
        ok = (unsigned)m0_line(line, text_line[line], line == 0 ? ACCENT : line == 11 ? DIM : TEXT);
    if (ok && m0.stress) m0.dirty_lines = 0xfffU;
}
