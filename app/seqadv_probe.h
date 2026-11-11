/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* SeqAdv M0 timing probes: interval statistics of existing interrupts and
 * task hooks, in ticks of the 24 MHz counter. Marks are a few instructions
 * and are always inlined, so interrupt handlers stay inside internal SRAM. */
#ifndef SEQADV_PROBE_H
#define SEQADV_PROBE_H
#include <stdint.h>
struct seqadv_probe { uint32_t last, min, max, count; };
enum { SEQADV_SCAN, SEQADV_SOFT, SEQADV_AUDIO, SEQADV_POLL, SEQADV_WAKE, SEQADV_PROBES };
extern struct seqadv_probe seqadv_probes[SEQADV_PROBES];
/* Wake test: an interrupt stamps the time and requests the control task. */
extern volatile uint32_t seqadv_wake_time;
extern volatile uint8_t seqadv_wake_armed, seqadv_wake_pending;
/* M0 clock: 96 PPQN tick deadlines in 24 MHz counter ticks. The scan
 * interrupt (about 240 kHz) compares the counter with `next`, records the
 * due time and requests the control task, which sends the MIDI. */
struct seqadv_clock {
    volatile uint32_t next, period, rem, acc, bpm;
    volatile uint32_t produced;         /* ticks made by the interrupt */
    volatile uint32_t due[8];           /* due time of tick n at [n & 7] */
    volatile uint8_t running;
};
extern struct seqadv_clock seqadv_clock;
/* Nonzero while the test page owns MIDI out: the stock engine otherwise
 * sends its own MIDI clock on every port, all the time, at its own tempo. */
extern volatile uint8_t seqadv_stock_clock_off;
static inline __attribute__((always_inline)) int seqadv_clock_poll(uint32_t now)
{
    struct seqadv_clock *c = &seqadv_clock;
    if (!c->running || (int32_t)(now - c->next) < 0) return 0;
    c->due[c->produced & 7U] = c->next;
    ++c->produced;
    c->next += c->period;
    c->acc += c->rem;
    if (c->acc >= c->bpm) { c->acc -= c->bpm; ++c->next; }
    return 1;
}
static inline __attribute__((always_inline)) void seqadv_probe_interval(unsigned index, uint32_t interval)
{
    struct seqadv_probe *p = &seqadv_probes[index];
    if (interval < p->min) p->min = interval;
    if (interval > p->max) p->max = interval;
    ++p->count;
}
static inline __attribute__((always_inline)) void seqadv_probe_mark(unsigned index, uint32_t now)
{
    uint32_t interval = now - seqadv_probes[index].last;
    seqadv_probes[index].last = now;
    seqadv_probe_interval(index, interval);
}
#endif
