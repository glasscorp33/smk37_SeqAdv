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
