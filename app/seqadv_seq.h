/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* SeqAdv sequencer engine: one track of 1..128 steps at 96 PPQN.
 * Pure C without hardware or SDK headers, so it also runs in host tests.
 * The caller supplies the time base (one seq_tick per 96 PPQN tick) and the
 * output (emit). Control task only on the device. */
#ifndef SEQADV_SEQ_H
#define SEQADV_SEQ_H
#include <stdint.h>

enum { SEQ_STEPS = 128, SEQ_NOTES = 4, SEQ_TRACKS = 1 };
enum { SEQ_PPQN = 96, SEQ_STEP_TICKS = 24, SEQ_GATE_MAX = 96 };
/* Sounding-note table, per track. A gate longer than a step can overlap up
 * to 16 notes; when the table is full the note closest to its end is
 * released early, so every note-on still gets exactly one note-off. */
enum { SEQ_VOICES = SEQ_NOTES * 2 };
enum { SEQ_PORT_DIN = 1, SEQ_PORT_USB = 2, SEQ_PORTS_ALL = 3 };
enum { SEQ_DEFAULT_GATE = 12, SEQ_DEFAULT_VELOCITY = 100, SEQ_DEFAULT_LENGTH = 16 };

struct seq_step  { uint8_t note[SEQ_NOTES]; uint8_t count; uint8_t velocity; uint8_t gate; };
                   /* count 0 = empty step; gate in ticks 1..96 (24 = one step) */
struct seq_track { struct seq_step step[SEQ_STEPS]; uint8_t length;      /* 1..128 */
                   uint8_t channel;                                       /* 0..15  */
                   uint8_t ports;                                         /* bit 0 DIN, bit 1 USB */ };

/* Output: one complete MIDI message for the given port mask. */
typedef void (*seq_emit_fn)(unsigned ports, const uint8_t *bytes, unsigned n);

void seq_init(void);
void seq_start(seq_emit_fn emit);       /* 0xFA; the next tick is tick 0 of step 0 */
void seq_stop(seq_emit_fn emit);        /* releases every sounding note, then 0xFC */
void seq_tick(seq_emit_fn emit);        /* offs, clock (every 4th tick), step ons */
int seq_running(void);
unsigned seq_position(void);            /* step under the playhead */
unsigned seq_sounding(void);            /* notes currently held by the engine */

const struct seq_track *seq_track(void);
const struct seq_step *seq_step(unsigned step);
/* Step editing. Notes are 0..127, duplicates are dropped, at most SEQ_NOTES.
 * count 0 clears the step. Returns 0 on success, -1 for invalid input. */
int seq_step_set(unsigned step, const uint8_t *notes, unsigned count, unsigned velocity, unsigned gate);
int seq_step_add_note(unsigned step, unsigned note); /* -1 when full or invalid */
void seq_step_clear(unsigned step);
void seq_step_velocity(unsigned step, unsigned velocity); /* clamped 1..127 */
void seq_step_gate(unsigned step, unsigned gate);         /* clamped 1..96 */
void seq_set_length(unsigned length);                      /* clamped 1..128 */
void seq_set_channel(unsigned channel);                    /* 0..15 */
void seq_set_ports(unsigned ports);
#endif
