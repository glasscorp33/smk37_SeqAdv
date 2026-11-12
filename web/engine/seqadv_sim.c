/* SPDX-License-Identifier: GPL-3.0-only WITH GPL-3.0-linking-exception */
/* Browser simulator only: make the SeqAdv sequencer audible.
 *
 * On the device SeqAdv (app/seqadv_ui.inc) sends its MIDI to DIN and USB
 * through fm1_midi_send_port and makes no sound itself. Here that output is
 * also given to
 *   - the simulator's own instrument, as if it were MIDI input: the engine
 *     routes each channel to the tracks listening on it (default project:
 *     channel 1 = track 1 DX7, channel 10 = track 8 808 kit), and
 *   - a small queue the page drains to a Web MIDI output port.
 * Only while the SeqAdv mode is active, and never for messages sent by the
 * stock MIDI router, so stock behaviour outside the mode is unchanged.
 *
 * Nothing here is part of the firmware: web/ is compiled only by
 * tools/Build-Web.ps1. */
#include "internal.h"
#include "seqadv_clock.h"
#include <string.h>

#ifdef __wasm__
#define EXPORT(name) __attribute__((export_name(name)))
#else
#define EXPORT(name)
#endif

int seqadv_active(void);
extern void groove_screen_snapshot(uint8_t state[8]);

enum { SOUND_INTERNAL = 1, SOUND_FORWARD = 2, OUT_QUEUE = 256 };
static unsigned flags = SOUND_INTERNAL;
static unsigned router_depth;
static uint32_t out_queue[OUT_QUEUE];
static unsigned out_read, out_write, out_dropped;
/* SeqAdv sends one message to DIN and then the same buffer to USB. */
static const uint8_t *last_data;
static uint8_t last_copy[3], last_bytes;

void fm1_web_stock_midi(int enter) { if (enter) ++router_depth; else if (router_depth) --router_depth; }

/* Called by fm1_midi_send_port for every message it accepted. */
void fm1_web_seqadv_sent(unsigned port, const uint8_t *data, size_t bytes)
{
    unsigned duplicate = port == 1 && last_data == data && last_bytes == bytes &&
                         bytes <= 3 && !memcmp(last_copy, data, bytes);
    last_data = NULL;
    if (port == 0 && bytes <= 3) { last_data = data; last_bytes = (uint8_t)bytes; memcpy(last_copy, data, bytes); }
    if (duplicate || router_depth || !bytes || bytes > 3 || !seqadv_active()) return;
    unsigned status = data[0], note = (status & 0xe0U) == 0x80U && bytes == 3;
    unsigned realtime = bytes == 1 && (status == 0xf8 || status == 0xfa || status == 0xfb || status == 0xfc);
    if (note && (flags & SOUND_INTERNAL)) {
        /* Stock REC records MIDI input: never write SeqAdv notes into it. */
        uint8_t view[8]; groove_screen_snapshot(view);
        if (!view[4]) (void)fm1_midi_post(data, bytes);
    }
    if ((note || realtime) && (flags & SOUND_FORWARD)) {
        if (out_write - out_read >= OUT_QUEUE) { ++out_dropped; return; }
        uint32_t packed = (uint32_t)bytes << 24;
        for (size_t i = 0; i < bytes; ++i) packed |= (uint32_t)data[i] << (8U * i);
        out_queue[out_write++ % OUT_QUEUE] = packed;
    }
}

/* The scan interrupt wakes the control task when a SeqAdv tick is due. Do the
 * same once per audio block instead of waiting for the next 10 ms poll. */
int fm1_web_seqadv_due(uint32_t now_ms)
{
    return seqadv_clock.running && (int32_t)(now_ms * 24000U - seqadv_clock.next) >= 0;
}

/* Bit 0: play SeqAdv notes on the simulator's instrument (default on).
 * Bit 1: queue SeqAdv MIDI for sim_seqadv_out (Web MIDI output). */
EXPORT("sim_seqadv_sound") void sim_seqadv_sound(unsigned value)
{
    flags = value & (SOUND_INTERNAL | SOUND_FORWARD);
    if (!(flags & SOUND_FORWARD)) out_read = out_write;
}
/* One queued message: bytes in bits 0..23 (first byte lowest), count in
 * bits 24..25. 0 when empty. */
EXPORT("sim_seqadv_out") uint32_t sim_seqadv_out(void)
{
    return out_read == out_write ? 0 : out_queue[out_read++ % OUT_QUEUE];
}
/* Bits 0..1 flags, bit 2 mode active, bits 8.. messages dropped (queue full). */
EXPORT("sim_seqadv_state") uint32_t sim_seqadv_state(void)
{
    return flags | (seqadv_active() ? 4U : 0U) | out_dropped << 8;
}
