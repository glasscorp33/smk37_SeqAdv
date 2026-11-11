/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* SeqAdv M1: step sequencer mode inside the stock Open firmware.
 *
 * SHIFT (printed KNOB BANK, button 0) + top key (key 36) enters and leaves
 * the mode. While it is active the stock instrument receives no panel
 * events, its MIDI clock is silenced, and this file owns MIDI out on DIN and
 * USB through fm1_midi_send_port. The firmware update path is untouched.
 *
 * Timing (proven in M0): the scan interrupt checks the 24 MHz tick deadline
 * (seqadv_clock.h) and wakes the control task; sq_service() runs first in
 * groove_screen_control and again between screen rows.
 *
 * Included at the end of screen.c to reuse its 5x7 font and band buffer.
 * Control task only. No allocation, no Flash writes, no blocking draws
 * while the transport runs (rows go out with fm1_screen_submit). */
#ifdef FM1_OPEN_WEB
/* The browser simulator has no hardware counter: derive it from milliseconds. */
static uint32_t sq_ticks(void) { return fm1_millis() * 24000U; }
static uint32_t sq_lock(void) { return 0; }
static void sq_unlock(uint32_t state) { (void)state; }
#else
#include "fm1_device.h"
static uint32_t sq_ticks(void) { return fm1_device_ticks(); }
static uint32_t sq_lock(void) { return fm1_device_irq_save(); }
static void sq_unlock(uint32_t state) { fm1_device_irq_restore(state); }
#endif

#include "seqadv_clock.h"
#include "seqadv_seq.h"
struct seqadv_clock seqadv_clock;
volatile uint8_t seqadv_stock_clock_off;
volatile uint8_t seqadv_phase;

/* Physical control IDs from smk37_control_event (scanner order, see
 * button_map in smk37_controls.c and the labels in sdk/Simulator.js). */
enum { SQ_BTN_SHIFT = 0, SQ_BTN_OCT_DOWN = 4, SQ_BTN_OCT_UP = 5, SQ_BTN_PLAY = 16,
       SQ_KEY_MODE = 36, SQ_PAD_DIAG = 15,
       SQ_ENC_TEMPO = 0, SQ_ENC_LENGTH = 1, SQ_ENC_CHANNEL = 2, SQ_ENC_GATE = 3 };
enum { SQ_LINES = 12, SQ_COLUMNS = 20, SQ_SCALE = 2, SQ_ROW = 20, SQ_PAGES = SEQ_STEPS / 16,
       SQ_TEXT = 48 /* composed row, longer than the 20 shown columns */,
       SQ_NONE = 255, SQ_KEY_BASE = 48 /* key 0 = C3, key 12 = C4 */ };
static struct {
    uint8_t active, ready, shift, clear, diag, page, selected, gate;
    uint8_t pad_held, pad_keyed;            /* pad held for note entry; keys added */
    uint8_t last[SEQ_NOTES], last_count;    /* last notes played on the keys */
    uint8_t live_note[37], live_channel[37], live_ports[37], live_down;
    uint16_t dirty_lines;                   /* one bit per text row still to draw */
    /* Lateness = send time - due time, in 24 MHz ticks. */
    uint32_t late_min, late_max, late_sum, late_count, late_peak, late_over, overruns, errors;
    uint32_t shown_late_min, shown_late_max, shown_late_avg, window_start;
    uint32_t over_phase[3], phase_max[3];   /* late ticks by activity; longest read / draw */
    uint8_t kind, id; int16_t value; uint32_t events;
    char shown[SQ_LINES][SQ_COLUMNS + 1];
} sq;

int seqadv_active(void) { return sq.active; }

/* Engine output: one message to DIN and/or USB. A full queue is counted,
 * never retried late. USB may be unplugged, so only its full queue counts. */
static void sq_emit(unsigned ports, const uint8_t *bytes, unsigned n)
{
    for (unsigned port = 0; port < 2; ++port)
        if (ports & (1U << port)) {
            int r = fm1_midi_send_port(port, bytes, n);
            if (r == -2 || (r && port == FM1_MIDI_DIN)) ++sq.errors;
        }
}
static void sq_tempo(unsigned bpm)
{
    /* 96 PPQN: 24 MHz * 60 / (bpm * 96) = 15,000,000 / bpm counter ticks. */
    uint32_t state = sq_lock();
    seqadv_clock.bpm = bpm; seqadv_clock.period = 15000000U / bpm; seqadv_clock.rem = 15000000U % bpm;
    seqadv_clock.acc = 0;
    sq_unlock(state);
}
static void sq_run(unsigned run)
{
    if (!seqadv_clock.bpm) sq_tempo(120);
    if (run && !seqadv_clock.running) {
        sq.late_min = ~0U; sq.late_max = sq.late_sum = sq.late_count = 0;
        sq.late_peak = sq.late_over = sq.overruns = sq.errors = 0;
        memset(sq.over_phase, 0, sizeof sq.over_phase); memset(sq.phase_max, 0, sizeof sq.phase_max);
        seq_start(sq_emit);                 /* 0xFA; the first tick carries 0xF8 and step 0 */
        uint32_t state = sq_lock();
        seqadv_clock.produced = 0; seqadv_clock.consumed = 0; seqadv_clock.acc = 0; seqadv_clock.retries = 0;
        seqadv_clock.next = sq_ticks() + seqadv_clock.period;
        seqadv_clock.running = 1;
        sq_unlock(state);
    } else if (!run && seqadv_clock.running) {
        seqadv_clock.running = 0;
        seq_stop(sq_emit);                  /* releases every sounding note, 0xFC */
    }
}
/* Enter an activity phase; returns the time, and records how long the
 * previous phase (1 panel read, 2 draw) lasted. */
static uint32_t sq_phase(unsigned phase, uint32_t started)
{
    uint32_t now = sq_ticks(), previous = seqadv_phase;
    if (previous && seqadv_clock.running && now - started > sq.phase_max[previous]) sq.phase_max[previous] = now - started;
    seqadv_phase = (uint8_t)phase;
    return now;
}
/* Control task, first thing on every wake: send what is due. */
static void sq_service(void)
{
#ifdef FM1_OPEN_WEB
    /* No scan interrupt in the simulator: poll the clock from here. */
    while (seqadv_clock_poll(sq_ticks())) {}
#endif
    while (seqadv_clock.running && seqadv_clock.consumed != seqadv_clock.produced) {
        uint32_t behind = seqadv_clock.produced - seqadv_clock.consumed;
        if (behind > 8U) { sq.overruns += behind - 8U; seqadv_clock.consumed += behind - 8U; }
        uint32_t tick = seqadv_clock.consumed++;
        uint32_t late = sq_ticks() - seqadv_clock.due[tick & 7U];
        seq_tick(sq_emit);
        if (late < sq.late_min) sq.late_min = late;
        if (late > sq.late_max) sq.late_max = late;
        if (late > sq.late_peak) sq.late_peak = late;
        if (late > 24000U) { ++sq.late_over; ++sq.over_phase[seqadv_clock.phase[tick & 7U] % 3U]; }
        sq.late_sum += late; ++sq.late_count;
    }
}

/* Live notes from the keys, on the track's channel and ports at press time. */
static void sq_live(unsigned key, unsigned velocity)
{
    if (sq.live_note[key] != SQ_NONE) {
        uint8_t off[3] = {(uint8_t)(0x80 | sq.live_channel[key]), sq.live_note[key], 0};
        sq_emit(sq.live_ports[key], off, 3);
        sq.live_note[key] = SQ_NONE; --sq.live_down;
    }
    if (!velocity) return;
    const struct seq_track *t = seq_track();
    uint8_t on[3] = {(uint8_t)(0x90 | t->channel), (uint8_t)(SQ_KEY_BASE + key), (uint8_t)velocity};
    sq_emit(t->ports, on, 3);
    sq.live_note[key] = on[1]; sq.live_channel[key] = t->channel; sq.live_ports[key] = t->ports;
    ++sq.live_down;
}
static void sq_leave(void)
{
    sq_run(0);
    for (unsigned key = 0; key < 37; ++key) sq_live(key, 0);
    sq.active = 0; seqadv_stock_clock_off = 0;
    saved_cache.shown_page = ~0U;           /* stock page repaints in full */
}
static void sq_enter(void)
{
    smk37_controls_release();               /* no hanging notes in the instrument */
    if (!sq.ready) {
        seq_init();
        sq.gate = SEQ_DEFAULT_GATE; sq.last[0] = 60; sq.last_count = 1;   /* C4 */
        sq.ready = 1;
    }
    memset(sq.live_note, SQ_NONE, sizeof sq.live_note); sq.live_down = 0;
    sq.pad_held = SQ_NONE; sq.diag = 0;
    memset(sq.shown, 0, sizeof sq.shown);
    sq.dirty_lines = 0xfffU; sq.clear = 1; sq.window_start = fm1_millis();
    if (!seqadv_clock.bpm) sq_tempo(120);
    sq.active = 1; seqadv_stock_clock_off = 1;
}
static int sq_clamp(int value, int low, int high) { return value < low ? low : value > high ? high : value; }

/* Called first by smk37_control_event. Returns 1 when the event is consumed. */
int seqadv_event(unsigned kind, unsigned id, int value)
{
    if (kind == SMK37_BUTTON && id == SQ_BTN_SHIFT) sq.shift = value != 0;
    if (kind == SMK37_KEY && id == SQ_KEY_MODE && value && sq.shift) {
        if (!sq.active) sq_enter(); else sq_leave();
        return 1;
    }
    if (!sq.active) return 0;
    sq.kind = (uint8_t)kind; sq.id = (uint8_t)id; sq.value = (int16_t)value; ++sq.events;
    if (kind == SMK37_KEY && id < 37) {
        unsigned note = SQ_KEY_BASE + id;
        if (value) {
            /* A new chord starts when no other key is held. */
            if (!sq.live_down) sq.last_count = 0;
            unsigned known = 0;
            for (unsigned i = 0; i < sq.last_count; ++i) known |= sq.last[i] == note;
            if (!known && sq.last_count < SEQ_NOTES) sq.last[sq.last_count++] = (uint8_t)note;
            if (sq.pad_held != SQ_NONE) {
                unsigned step = sq.page * 16U + sq.pad_held;
                if (!sq.pad_keyed) {
                    uint8_t n = (uint8_t)note;
                    (void)seq_step_set(step, &n, 1, (unsigned)value, seq_step(step)->gate);
                    sq.pad_keyed = 1;
                } else (void)seq_step_add_note(step, note);
                sq.selected = (uint8_t)step;
            }
        }
        sq_live(id, (unsigned)value);
    } else if (kind == SMK37_PAD && id < 16) {
        if (!value) { if (sq.pad_held == id) sq.pad_held = SQ_NONE; return 1; }
        if (sq.shift) {
            if (id == SQ_PAD_DIAG) { sq.diag = !sq.diag; sq.dirty_lines = 0xfffU; }
            return 1;
        }
        unsigned step = sq.page * 16U + id;
        if (seq_step(step)->count) seq_step_clear(step);
        else (void)seq_step_set(step, sq.last, sq.last_count, SEQ_DEFAULT_VELOCITY, sq.gate);
        sq.selected = (uint8_t)step; sq.pad_held = (uint8_t)id; sq.pad_keyed = 0;
    } else if (kind == SMK37_BUTTON && value) {
        if (id == SQ_BTN_PLAY) sq_run(!seqadv_clock.running);
        else if (id == SQ_BTN_OCT_DOWN || id == SQ_BTN_OCT_UP) {
            int page = sq_clamp((int)sq.page + (id == SQ_BTN_OCT_UP ? 1 : -1), 0, SQ_PAGES - 1);
            if (page != sq.page) { sq.page = (uint8_t)page; sq.pad_held = SQ_NONE; sq.dirty_lines = 0xfffU; }
        }
    } else if (kind == SMK37_ENCODER && value) {
        const struct seq_track *t = seq_track();
        if (id == SQ_ENC_TEMPO) sq_tempo((unsigned)sq_clamp((int)seqadv_clock.bpm + value, 30, 300));
        else if (id == SQ_ENC_LENGTH) seq_set_length((unsigned)sq_clamp((int)t->length + value, 1, SEQ_STEPS));
        else if (id == SQ_ENC_CHANNEL) seq_set_channel((unsigned)sq_clamp((int)t->channel + value, 0, 15));
        else if (id == SQ_ENC_GATE) {
            sq.gate = (uint8_t)sq_clamp((int)seq_step(sq.selected)->gate + value, 1, SEQ_GATE_MAX);
            seq_step_gate(sq.selected, sq.gate);
        }
    }
    return 1;                               /* everything else is ignored in the mode */
}

/* Pad colours while the mode is active (see smk37_pad_color). */
uint32_t seqadv_pad_color(unsigned pad)
{
    unsigned step = sq.page * 16U + pad;
    const struct seq_track *t = seq_track();
    if (pad >= 16 || step >= t->length) return 0;
    if (seq_running() && step == seq_position()) return 0x303030;
    return t->step[step].count ? 0x002814 : 0x040404;
}

static char *sq_put(char *out, const char *s) { while (*s) *out++ = *s++; *out = 0; return out; }
static char *sq_number(char *out, long value, unsigned width)
{
    char digits[12]; unsigned n = 0;
    unsigned long v = value < 0 ? (unsigned long)-value : (unsigned long)value;
    if (value < 0) *out++ = '-';
    do { digits[n++] = (char)('0' + v % 10U); v /= 10U; } while (v);
    while (n < width) digits[n++] = '0';
    while (n) *out++ = digits[--n];
    *out = 0; return out;
}
static char *sq_note_name(char *out, unsigned note)
{
    static const char names[] = "C C#D D#E F F#G G#A A#B ";
    *out++ = names[note % 12U * 2U];
    if (names[note % 12U * 2U + 1U] == '#') *out++ = '#';
    return sq_number(out, (long)(note / 12U) - 1, 1);
}

/* Draw one text row with an asynchronous transfer. Returns 0 when the LCD
 * is still busy; the row stays owed. */
static int sq_line(unsigned line, const char *s, uint16_t color)
{
    if (!(sq.dirty_lines & (1U << line)) && !strcmp(sq.shown[line], s)) return 1;
    if (fm1_screen_pending()) return 0;
    enum { W = FM1_SCREEN_WIDTH, H = 7 * SQ_SCALE };
    for (unsigned i = 0; i < W * H; ++i) paint_pixels[i] = BACK;
    unsigned x = 0;
    for (const char *p = s; *p && x + 6U * SQ_SCALE <= W; ++p, x += 6U * SQ_SCALE) {
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
            for (unsigned col = 0; col < 5U * SQ_SCALE; ++col)
                if (shape[col / SQ_SCALE] & (1U << (row / SQ_SCALE)))
                    paint_pixels[row * W + x + col] = color;
    }
    /* Returns at once and borrows the buffer until fm1_screen_pending()
     * reports completion, so one call stays short. */
    if (fm1_screen_submit(0, 4U + line * SQ_ROW, W, H, paint_pixels, W * H)) return 0;
    strncpy(sq.shown[line], s, SQ_COLUMNS); sq.shown[line][SQ_COLUMNS] = 0;
    sq.dirty_lines &= (uint16_t)~(1U << line);
    return 1;
}

static void sq_text_main(char text[SQ_LINES][SQ_TEXT])
{
    const struct seq_track *t = seq_track();
    unsigned running = (unsigned)seq_running(), position = seq_position();
    char *p = sq_put(text[0], running ? "SEQADV PLAY " : "SEQADV STOP ");
    p = sq_number(p, (long)seqadv_clock.bpm, 1); sq_put(p, " BPM");
    p = sq_put(text[1], "T1 CH"); p = sq_number(p, t->channel + 1L, 2);
    p = sq_put(p, " LEN"); p = sq_number(p, t->length, 3);
    p = sq_put(p, " P"); p = sq_number(p, sq.page + 1L, 1); sq_put(p, "/8");
    const struct seq_step *s = seq_step(sq.selected);
    p = sq_put(text[2], "S"); p = sq_number(p, sq.selected + 1L, 3);
    if (!s->count) sq_put(p, " ---");
    for (unsigned n = 0; n < s->count; ++n) { *p++ = ' '; p = sq_note_name(p, s->note[n]); }
    for (unsigned row = 0; row < SQ_PAGES; ++row) {
        char *o = text[3 + row];
        for (unsigned i = 0; i < 16; ++i) {
            unsigned step = row * 16U + i;
            *o++ = step >= t->length ? ' ' : running && step == position ? '>' : t->step[step].count ? 'o' : '.';
        }
        *o = 0;
    }
    /* Velocity and gate of the selected step; row 3 has no room for them. */
    p = sq_put(text[11], "V"); p = sq_number(p, s->velocity, 3);
    p = sq_put(p, " G"); p = sq_number(p, s->gate, 2);
    sq_put(p, " S+P16 DIAG");
}
static void sq_text_diag(char text[SQ_LINES][SQ_TEXT])
{
    static const char *const kinds[8] = {"BTN", "KEY", "PRS", "PAD", "ENC", "CMB", "PED", "FDR"};
    char *p;
    sq_put(text[0], "SEQADV DIAG");
    p = sq_put(text[1], seqadv_clock.running ? "RUN  BPM " : "STOP BPM "); sq_number(p, (long)seqadv_clock.bpm, 1);
    p = sq_put(text[2], "TICKS "); sq_number(p, (long)seqadv_clock.consumed, 1);
    p = sq_put(text[3], "LATE MIN US "); sq_number(p, (long)sq.shown_late_min, 1);
    p = sq_put(text[4], "LATE AVG US "); sq_number(p, (long)sq.shown_late_avg, 1);
    p = sq_put(text[5], "LATE MAX US "); sq_number(p, (long)sq.shown_late_max, 1);
    p = sq_put(text[6], "PEAK US "); sq_number(p, (long)(sq.late_peak / 24U), 1);
    p = sq_put(text[7], "OVER 1MS "); sq_number(p, (long)sq.late_over, 1);
    p = sq_put(text[8], "OUT "); p = sq_number(p, (long)sq.over_phase[0], 1);
    p = sq_put(p, " RD "); p = sq_number(p, (long)sq.over_phase[1], 1);
    p = sq_put(p, " DRW "); sq_number(p, (long)sq.over_phase[2], 1);
    p = sq_put(text[9], "MAX RD "); p = sq_number(p, (long)(sq.phase_max[1] / 24U), 1);
    p = sq_put(p, " DR "); sq_number(p, (long)(sq.phase_max[2] / 24U), 1);
    p = sq_put(text[10], "LOST "); p = sq_number(p, (long)sq.overruns, 1);
    p = sq_put(p, " ERR "); p = sq_number(p, (long)sq.errors, 1);
    p = sq_put(p, " RT "); sq_number(p, (long)seqadv_clock.retries, 1);
    /* Last raw panel event, to check control IDs on the device. */
    p = sq_put(text[11], "LAST "); p = sq_put(p, sq.events ? kinds[sq.kind & 7U] : "---");
    *p++ = ' '; p = sq_number(p, sq.id, 1); *p++ = ' '; sq_number(p, sq.value, 1);
}

/* Called from groove_screen_control after the panel has been read. */
static void sq_frame(void)
{
    uint32_t now = fm1_millis();
    if (now - sq.window_start >= 1000U) {
        sq.window_start = now;
        if (sq.late_count) {
            sq.shown_late_min = sq.late_min / 24U; sq.shown_late_max = sq.late_max / 24U;
            sq.shown_late_avg = sq.late_sum / sq.late_count / 24U;
            sq.late_min = ~0U; sq.late_max = sq.late_sum = sq.late_count = 0;
        }
    }
    if (sq.clear) {
        /* Entry only, with the transport stopped: one blocking clear. */
        if (fm1_screen_pending()) return;
        if (!seqadv_clock.running) fm1_screen_fill(0, 0, FM1_SCREEN_WIDTH, FM1_SCREEN_HEIGHT, BACK);
        sq.clear = 0;
    }
    static char text[SQ_LINES][SQ_TEXT];
    if (sq.diag) sq_text_diag(text); else sq_text_main(text);
    /* Rows go out one at a time by DMA, so a redraw spans several calls:
     * the owed rows are remembered instead of restarting. */
    for (unsigned line = 0, ok = 1; line < SQ_LINES && ok; ++line) {
        sq_service();
        uint16_t color = line == 0 ? ACCENT : line == 11 ? DIM : TEXT;
        if (sq.diag && line == 10) color = DIM;
        if (!sq.diag && line == 3U + sq.page) color = ACCENT;
        ok = (unsigned)sq_line(line, text[line], color);
    }
}
