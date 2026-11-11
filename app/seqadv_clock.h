/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* SeqAdv interrupt-side clock: 96 PPQN tick deadlines in ticks of the
 * 24 MHz counter. The check is a few instructions and always inlined, so
 * the scan interrupt stays inside internal SRAM. Verified on hardware in M0. */
#ifndef SEQADV_CLOCK_H
#define SEQADV_CLOCK_H
#include <stdint.h>
/* The scan interrupt (about 240 kHz) compares the counter with `next`,
 * records the due time and requests the control task, which sends the MIDI. */
struct seqadv_clock {
    volatile uint32_t next, period, rem, acc, bpm;
    volatile uint32_t produced;         /* ticks made by the interrupt */
    volatile uint32_t consumed;         /* ticks sent by the control task */
    volatile uint32_t requested, retries; /* last wake request; repeats */
    volatile uint32_t due[8];           /* due time of tick n at [n & 7] */
    volatile uint8_t phase[8];          /* what the control task was doing then */
    volatile uint8_t running;
};
extern struct seqadv_clock seqadv_clock;
/* Control task activity, sampled by the interrupt when a tick falls due:
 * 0 outside the screen callback, 1 reading the panel, 2 drawing. */
extern volatile uint8_t seqadv_phase;
/* Nonzero while SeqAdv owns MIDI out: the stock engine otherwise sends
 * its own MIDI clock on every port, all the time, at its own tempo. */
extern volatile uint8_t seqadv_stock_clock_off;
static inline __attribute__((always_inline)) int seqadv_clock_poll(uint32_t now)
{
    struct seqadv_clock *c = &seqadv_clock;
    if (!c->running) return 0;
    if ((int32_t)(now - c->next) < 0) {
        /* A wake request can be missed. While ticks are waiting, ask again
         * every 300 us instead of leaving them to the next tick or the
         * 10 ms control period. */
        if (c->produced == c->consumed || now - c->requested < 7200U) return 0;
        c->requested = now; ++c->retries;
        return 1;
    }
    c->requested = now;
    c->due[c->produced & 7U] = c->next;
    c->phase[c->produced & 7U] = seqadv_phase;
    ++c->produced;
    c->next += c->period;
    c->acc += c->rem;
    if (c->acc >= c->bpm) { c->acc -= c->bpm; ++c->next; }
    return 1;
}
#endif
