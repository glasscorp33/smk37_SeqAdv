/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* Host tests for the SeqAdv sequencer engine (app/seqadv_seq.c).
 * Build: cc -std=c11 -Wall -Wextra -Werror -Iapp tests/seqadv/test_seq.c app/seqadv_seq.c */
#include "seqadv_seq.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned failures, checks;
#define CHECK(cond) do { ++checks; if (!(cond)) { ++failures; \
    fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); } } while (0)

/* Every emitted message, stamped with the tick that produced it. */
struct event { unsigned tick, ports, n; uint8_t b[3]; };
enum { LOG_MAX = 1 << 20 };
static struct event *events;
static unsigned count, now;   /* now: index of the tick being produced */

/* Sounding notes as seen by a receiver, per port, channel and note. */
static uint8_t held[2][16][128];
static unsigned ons, offs, double_on, orphan_off, clocks;

static void emit(unsigned ports, const uint8_t *bytes, unsigned n)
{
    if (count < LOG_MAX) {
        struct event *e = &events[count++];
        e->tick = now; e->ports = ports; e->n = n; memcpy(e->b, bytes, n);
    }
    unsigned status = bytes[0] & 0xf0U, ch = bytes[0] & 15U;
    if (bytes[0] == 0xf8) ++clocks;
    for (unsigned p = 0; p < 2; ++p) {
        if (!(ports & (1U << p))) continue;
        if (status == 0x90 && n == 3 && bytes[2]) {
            if (held[p][ch][bytes[1]]) ++double_on;
            held[p][ch][bytes[1]] = 1; ++ons;
        } else if (status == 0x80 || (status == 0x90 && n == 3 && !bytes[2])) {
            if (!held[p][ch][bytes[1]]) ++orphan_off;
            held[p][ch][bytes[1]] = 0; ++offs;
        }
    }
}
static unsigned held_total(void)
{
    unsigned total = 0;
    for (unsigned p = 0; p < 2; ++p) for (unsigned c = 0; c < 16; ++c) for (unsigned k = 0; k < 128; ++k) total += held[p][c][k];
    return total;
}
static void reset(void)
{
    count = now = 0; memset(held, 0, sizeof held);
    ons = offs = double_on = orphan_off = clocks = 0;
    seq_init();
}
static void run(unsigned ticks) { for (unsigned i = 0; i < ticks; ++i) { seq_tick(emit); ++now; } }
static int is_on(const struct event *e) { return (e->b[0] & 0xf0) == 0x90 && e->n == 3 && e->b[2]; }
static int is_off(const struct event *e) { return (e->b[0] & 0xf0) == 0x80; }
static void put(unsigned step, unsigned note, unsigned gate)
{ uint8_t n = (uint8_t)note; CHECK(seq_step_set(step, &n, 1, 100, gate) == 0); }

/* 1. Clock on every 4th tick, ons only on step boundaries, order offs/clock/ons. */
static void test_clock_and_order(void)
{
    reset();
    for (unsigned s = 0; s < 16; ++s) put(s, 60 + s % 5, s % 2 ? 24 : 12);
    seq_start(emit);
    CHECK(count == 1 && events[0].b[0] == 0xfa && events[0].ports == SEQ_PORTS_ALL);
    unsigned first = count;
    run(16 * 24 * 4);
    CHECK(count > first + 1 && events[first].b[0] == 0xf8 && events[first].tick == 0);
    CHECK(is_on(&events[first + 1]) && events[first + 1].tick == 0 && events[first + 1].b[1] == 60);
    unsigned clock_ticks = 0;
    for (unsigned t = 0; t < 16 * 24 * 4; ++t) {
        unsigned phase = 0, clock_here = 0;     /* 0 offs, 1 clock, 2 ons */
        for (unsigned i = first; i < count; ++i) {
            const struct event *e = &events[i];
            if (e->tick != t) continue;
            if (e->b[0] == 0xf8) { CHECK(phase <= 1); phase = 1; ++clock_here; }
            else if (is_off(e)) CHECK(phase == 0);
            else if (is_on(e)) { CHECK(t % 24 == 0); phase = 2; }
            else CHECK(0);
        }
        CHECK(clock_here == (t % 4 == 0));
        clock_ticks += clock_here;
    }
    CHECK(clock_ticks == 16 * 24);
    CHECK(double_on == 0 && orphan_off == 0);
}

/* 2. The note-off arrives exactly gate ticks after its note-on. */
static void test_gate(void)
{
    static const unsigned gates[] = {1, 2, 11, 12, 23, 24, 25, 47, 95, 96};
    for (unsigned g = 0; g < sizeof gates / sizeof gates[0]; ++g) {
        reset();
        put(0, 64, gates[g]);
        seq_start(emit);
        run(16 * 24 * 3);
        unsigned pairs = 0;
        for (unsigned i = 0; i < count; ++i) {
            if (!is_on(&events[i])) continue;
            unsigned j = i + 1;
            while (j < count && !(is_off(&events[j]) && events[j].b[1] == 64)) ++j;
            if (j == count) continue;       /* still sounding at the end of the run */
            CHECK(events[j].tick - events[i].tick == gates[g]);
            ++pairs;
        }
        CHECK(pairs >= 2);
        CHECK(double_on == 0 && orphan_off == 0);
    }
}

/* 3. Lengths 1, 16, 17, 128 wrap to step 0. */
static void test_lengths(void)
{
    static const unsigned lengths[] = {1, 16, 17, 128};
    for (unsigned l = 0; l < 4; ++l) {
        reset();
        for (unsigned s = 0; s < SEQ_STEPS; ++s) put(s, s, 6);
        seq_set_length(lengths[l]);
        CHECK(seq_track()->length == lengths[l]);
        seq_start(emit);
        unsigned steps = lengths[l] * 3 + 5;
        for (unsigned k = 0; k < steps; ++k) {
            CHECK(seq_position() == k % lengths[l]);
            run(24);
        }
        unsigned k = 0;
        for (unsigned i = 0; i < count; ++i) {
            if (!is_on(&events[i])) continue;
            CHECK(events[i].b[1] == k % lengths[l]);
            CHECK(events[i].tick == k * 24);
            ++k;
        }
        CHECK(k == steps);
    }
    reset();
    seq_set_length(0); CHECK(seq_track()->length == 1);
    seq_set_length(500); CHECK(seq_track()->length == 128);
}

/* 4. A sounding note that starts again is released first. */
static void test_retrigger(void)
{
    reset();
    put(0, 60, 48); put(1, 60, 48);
    seq_start(emit);
    run(24 * 2 + 1);
    unsigned at = 0, seen = 0;
    for (unsigned i = 0; i < count; ++i)
        if (events[i].tick == 24 && (is_on(&events[i]) || is_off(&events[i]))) {
            if (!seen) CHECK(is_off(&events[i]) && events[i].b[1] == 60);
            else CHECK(is_on(&events[i]) && events[i].b[1] == 60);
            ++seen; at = i;
        }
    CHECK(seen == 2 && at > 0);
    CHECK(double_on == 0 && orphan_off == 0);
    seq_stop(emit);
    CHECK(held_total() == 0 && ons == offs);
}

/* 5. Stop releases everything; long randomised run with edits keeps one
 * note-off per note-on, on the note-on's own channel and ports. */
static unsigned rng = 12345;
static unsigned rnd(unsigned n) { rng = rng * 1103515245U + 12345U; return (rng >> 16) % n; }
static void test_stop_and_random(void)
{
    reset();
    uint8_t chord[4] = {60, 64, 67, 71};
    CHECK(seq_step_set(0, chord, 4, 90, 96) == 0);
    put(1, 50, 96); put(2, 52, 30);
    seq_start(emit);
    run(24 * 3 + 5);
    CHECK(held_total() > 0);
    seq_stop(emit);
    CHECK(held_total() == 0 && seq_sounding() == 0);
    CHECK(events[count - 1].b[0] == 0xfc);
    unsigned before = count;
    seq_stop(emit);                         /* stopping twice sends nothing */
    CHECK(count == before);

    for (unsigned round = 0; round < 20; ++round) {
        reset();
        rng = 1000U + round;
        for (unsigned s = 0; s < SEQ_STEPS; ++s)
            if (rnd(3)) {
                uint8_t notes[4]; unsigned n = 1 + rnd(4);
                for (unsigned i = 0; i < n; ++i) notes[i] = (uint8_t)(36 + rnd(24));
                CHECK(seq_step_set(s, notes, n, 1 + rnd(127), 1 + rnd(96)) == 0);
            }
        seq_set_length(1 + rnd(128));
        seq_start(emit);
        for (unsigned t = 0; t < 40000; ++t) {
            seq_tick(emit); ++now;
            switch (rnd(400)) {
            case 0: seq_set_length(1 + rnd(128)); break;
            case 1: seq_set_channel(rnd(16)); break;
            case 2: seq_step_clear(rnd(SEQ_STEPS)); break;
            case 3: { uint8_t n = (uint8_t)(rnd(128)); (void)seq_step_set(rnd(SEQ_STEPS), &n, 1, 100, 1 + rnd(96)); break; }
            case 4: (void)seq_step_add_note(rnd(SEQ_STEPS), rnd(128)); break;
            case 5: seq_step_gate(rnd(SEQ_STEPS), rnd(200)); break;
            case 6: seq_set_ports(rnd(4)); break;
            case 7: if (!rnd(10)) { seq_stop(emit); CHECK(held_total() == 0); seq_start(emit); } break;
            case 8: if (!rnd(10)) seq_start(emit); break;   /* restart while running */
            default: break;
            }
            CHECK(seq_sounding() <= SEQ_VOICES);
        }
        seq_stop(emit);
        CHECK(held_total() == 0 && seq_sounding() == 0);
        CHECK(double_on == 0 && orphan_off == 0);
        CHECK(ons == offs && ons > 1000);
    }
    /* Ticks while stopped send nothing. */
    before = count; run(100); CHECK(count == before);
}

/* 6. Four notes on one step start on the same tick. */
static void test_chord(void)
{
    reset();
    uint8_t chord[4] = {48, 55, 60, 64};
    CHECK(seq_step_set(3, chord, 4, 77, 20) == 0);
    CHECK(seq_step(3)->count == 4);
    uint8_t five[5] = {1, 2, 3, 4, 5};
    CHECK(seq_step_set(4, five, 5, 100, 12) == -1);
    CHECK(seq_step_add_note(3, 70) == -1);
    CHECK(seq_step_add_note(3, 55) == 0 && seq_step(3)->count == 4);  /* duplicate */
    uint8_t dup[3] = {50, 50, 51};
    CHECK(seq_step_set(5, dup, 3, 100, 12) == 0 && seq_step(5)->count == 2);
    seq_start(emit);
    run(16 * 24);
    unsigned seen = 0;
    for (unsigned i = 0; i < count; ++i)
        if (is_on(&events[i])) {
            if (events[i].b[1] == 50 || events[i].b[1] == 51) continue;
            CHECK(events[i].tick == 3 * 24 && events[i].b[1] == chord[seen] && events[i].b[2] == 77);
            ++seen;
        }
    CHECK(seen == 4);
    CHECK(double_on == 0 && orphan_off == 0);
}

/* Defaults from the design. */
static void test_defaults(void)
{
    reset();
    const struct seq_track *t = seq_track();
    CHECK(t->length == 16 && t->channel == 0 && t->ports == (SEQ_PORT_DIN | SEQ_PORT_USB));
    for (unsigned s = 0; s < SEQ_STEPS; ++s)
        CHECK(t->step[s].count == 0 && t->step[s].gate == 12 && t->step[s].velocity == 100);
    CHECK(!seq_running() && seq_position() == 0);
    seq_set_channel(9); put(0, 60, 12);
    seq_start(emit); run(1);
    CHECK(count == 3 && events[2].b[0] == 0x99 && events[2].b[1] == 60 && events[2].b[2] == 100);
    seq_set_channel(3); run(12);            /* off goes to the old channel */
    unsigned old_channel_off = 0;
    for (unsigned i = 0; i < count; ++i) old_channel_off += events[i].b[0] == 0x89 && events[i].tick == 12;
    CHECK(old_channel_off == 1 && held_total() == 0);
    seq_set_ports(SEQ_PORT_DIN); seq_stop(emit); seq_start(emit); run(1);
    CHECK(events[count - 1].ports == SEQ_PORT_DIN && events[count - 1].b[0] == 0x93);
    seq_stop(emit);
    CHECK(held_total() == 0);
}

int main(void)
{
    events = calloc(LOG_MAX, sizeof *events);
    if (!events) return 2;
    test_defaults();
    test_clock_and_order();
    test_gate();
    test_lengths();
    test_retrigger();
    test_stop_and_random();
    test_chord();
    free(events);
    printf("seqadv engine: %u checks, %u failures\n", checks, failures);
    return failures ? 1 : 0;
}
