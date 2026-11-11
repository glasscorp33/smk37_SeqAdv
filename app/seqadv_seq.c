/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* SeqAdv sequencer engine. See seqadv_seq.h. No allocation, no I/O. */
#include "seqadv_seq.h"
#include <string.h>

/* Build.ps1 compiles new units at -O2; ask the firmware compiler for size
 * here. The engine is a few hundred instructions per tick at most. */
#if defined(__clang__) && !defined(FM1_OPEN_WEB)
#define SEQ_SMALL __attribute__((minsize))
#else
#define SEQ_SMALL
#endif

struct seq_voice { uint8_t note, channel, ports, remaining; };  /* remaining 0: free */

static struct seq_track track;
static struct seq_voice voice[SEQ_VOICES];
static uint8_t running, step_tick, position;

static unsigned clamp(unsigned value, unsigned low, unsigned high)
{ return value < low ? low : value > high ? high : value; }

static SEQ_SMALL void note_message(seq_emit_fn emit, unsigned status, const struct seq_voice *v, unsigned velocity)
{
    uint8_t m[3] = {(uint8_t)(status | v->channel), v->note, (uint8_t)velocity};
    emit(v->ports, m, 3);
}
static SEQ_SMALL void voice_off(seq_emit_fn emit, struct seq_voice *v)
{
    note_message(emit, 0x80, v, 0);
    v->remaining = 0;
}
static SEQ_SMALL void release_all(seq_emit_fn emit)
{
    for (unsigned i = 0; i < SEQ_VOICES; ++i)
        if (voice[i].remaining) voice_off(emit, &voice[i]);
}
static SEQ_SMALL void voice_on(seq_emit_fn emit, unsigned note, unsigned velocity, unsigned gate)
{
    struct seq_voice *slot = NULL;
    /* A note still sounding on this channel ends before it starts again. */
    for (unsigned i = 0; i < SEQ_VOICES; ++i)
        if (voice[i].remaining && voice[i].note == note && voice[i].channel == track.channel) {
            voice_off(emit, &voice[i]); slot = &voice[i]; break;
        }
    if (!slot)
        for (unsigned i = 0; i < SEQ_VOICES && !slot; ++i)
            if (!voice[i].remaining) slot = &voice[i];
    if (!slot) {
        /* Table full: release the note nearest to its own note-off. */
        slot = &voice[0];
        for (unsigned i = 1; i < SEQ_VOICES; ++i)
            if (voice[i].remaining < slot->remaining) slot = &voice[i];
        voice_off(emit, slot);
    }
    slot->note = (uint8_t)note; slot->channel = track.channel; slot->ports = track.ports;
    slot->remaining = (uint8_t)gate;
    note_message(emit, 0x90, slot, velocity);
}

SEQ_SMALL void seq_init(void)
{
    memset(&track, 0, sizeof track);
    memset(voice, 0, sizeof voice);
    track.length = SEQ_DEFAULT_LENGTH; track.channel = 0; track.ports = SEQ_PORTS_ALL;
    for (unsigned i = 0; i < SEQ_STEPS; ++i) {
        track.step[i].velocity = SEQ_DEFAULT_VELOCITY; track.step[i].gate = SEQ_DEFAULT_GATE;
    }
    running = 0; step_tick = 0; position = 0;
}

SEQ_SMALL void seq_start(seq_emit_fn emit)
{
    const uint8_t start = 0xfa;
    release_all(emit);
    step_tick = 0; position = 0; running = 1;
    emit(SEQ_PORTS_ALL, &start, 1);
}

SEQ_SMALL void seq_stop(seq_emit_fn emit)
{
    const uint8_t stop = 0xfc;
    int was = running;
    running = 0;
    release_all(emit);
    if (was) emit(SEQ_PORTS_ALL, &stop, 1);
    step_tick = 0; position = 0;
}

SEQ_SMALL void seq_tick(seq_emit_fn emit)
{
    if (!running) return;
    for (unsigned i = 0; i < SEQ_VOICES; ++i)
        if (voice[i].remaining && !--voice[i].remaining) note_message(emit, 0x80, &voice[i], 0);
    if (step_tick % 4U == 0) { const uint8_t clock = 0xf8; emit(SEQ_PORTS_ALL, &clock, 1); }
    if (step_tick == 0) {
        const struct seq_step *s = &track.step[position];
        for (unsigned n = 0; n < s->count; ++n) voice_on(emit, s->note[n], s->velocity, s->gate);
    }
    if (++step_tick >= SEQ_STEP_TICKS) {
        step_tick = 0;
        position = (uint8_t)(position + 1U >= track.length ? 0 : position + 1U);
    }
}

int seq_running(void) { return running; }
unsigned seq_position(void) { return position; }
unsigned seq_sounding(void)
{
    unsigned count = 0;
    for (unsigned i = 0; i < SEQ_VOICES; ++i) count += voice[i].remaining != 0;
    return count;
}
const struct seq_track *seq_track(void) { return &track; }
const struct seq_step *seq_step(unsigned step) { return step < SEQ_STEPS ? &track.step[step] : NULL; }

SEQ_SMALL int seq_step_add_note(unsigned step, unsigned note)
{
    if (step >= SEQ_STEPS || note > 127) return -1;
    struct seq_step *s = &track.step[step];
    for (unsigned n = 0; n < s->count; ++n) if (s->note[n] == note) return 0;
    if (s->count >= SEQ_NOTES) return -1;
    s->note[s->count++] = (uint8_t)note;
    return 0;
}
SEQ_SMALL int seq_step_set(unsigned step, const uint8_t *notes, unsigned count, unsigned velocity, unsigned gate)
{
    if (step >= SEQ_STEPS || count > SEQ_NOTES || (count && !notes)) return -1;
    for (unsigned n = 0; n < count; ++n) if (notes[n] > 127) return -1;
    struct seq_step *s = &track.step[step];
    s->count = 0;
    for (unsigned n = 0; n < count; ++n) (void)seq_step_add_note(step, notes[n]);
    s->velocity = (uint8_t)clamp(velocity, 1, 127);
    s->gate = (uint8_t)clamp(gate, 1, SEQ_GATE_MAX);
    return 0;
}
void seq_step_clear(unsigned step) { if (step < SEQ_STEPS) track.step[step].count = 0; }
void seq_step_velocity(unsigned step, unsigned velocity)
{ if (step < SEQ_STEPS) track.step[step].velocity = (uint8_t)clamp(velocity, 1, 127); }
void seq_step_gate(unsigned step, unsigned gate)
{ if (step < SEQ_STEPS) track.step[step].gate = (uint8_t)clamp(gate, 1, SEQ_GATE_MAX); }
void seq_set_length(unsigned length) { track.length = (uint8_t)clamp(length, 1, SEQ_STEPS); }
void seq_set_channel(unsigned channel) { track.channel = (uint8_t)(channel & 15U); }
void seq_set_ports(unsigned ports) { track.ports = (uint8_t)(ports & SEQ_PORTS_ALL); }
