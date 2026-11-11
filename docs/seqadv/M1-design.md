# SeqAdv M1 design: sequencer core, one track, step entry

Branch: `m1-sequencer` (from `m0-test-app`). Target: M-VAVE SMK-37 Elite.
Read `docs/seqadv/M1-design.md` (this file) fully before changing code.

## 1. Goal

A first usable sequencer inside the firmware: transport, tempo, MIDI clock out,
one track of 1 to 128 steps with step entry from pads and keys, notes out on
DIN and USB with the timing path already proven on hardware.

M1 is a **mode inside the stock Open firmware**, entered the same way as the
M0 test page (SHIFT + highest key; SHIFT is the button printed KNOB BANK).
The device still boots into the stock firmware. Reasons: the stock update
path stays untouched (brick safety), and removing stock code needs a size and
dependency study that M1 produces as a report (section 9). Stripping the stock
app is a later milestone.

Out of scope for M1: multiple tracks in the UI, drum lanes, real-time
recording, swing, parameter locks, chaining, saving to flash, BLE output,
the editor.

## 2. Hard rules (do not break)

1. Do not change `app/transfer.c`, anything under `sdk/` (it is hash-pinned by
   `sdk/manifest.json`), or `app/synth/` (hash-pinned). Do not change the
   update or recovery behaviour.
2. Outside SeqAdv mode the firmware must behave exactly like stock. SeqAdv
   code may run from boot only as: the inlined clock check in `scan_isr`
   (does nothing while the clock is stopped) and the `m0_clock_service`
   style call at the top of `groove_screen_control` (returns at once when
   stopped).
3. Interrupt code (`app/board/scan.c`, `app/board/audio.c`): only always-inline
   helpers and plain memory access. No new function calls from interrupts
   except `fm1_control_request()`. Everything an interrupt touches must stay
   in SRAM; the build checks this.
4. Timing path, verified on hardware, must be kept exactly:
   - tick deadlines as 24 MHz counter values, checked in `scan_isr`;
   - on a due tick: store due time, `fm1_control_request()`;
   - repeat the request every 300 us while ticks are waiting;
   - the control task sends due MIDI first thing in `groove_screen_control`,
     and again between screen rows while drawing;
   - stock MIDI clock silenced while SeqAdv mode is active
     (`seqadv_stock_clock_off`);
   - screen drawing only with `fm1_screen_submit` (one row per call), never
     `fm1_screen_blit` or `fm1_screen_fill` while the transport runs.
5. No flash writes, no `malloc`, no blocking waits in SeqAdv code.
6. MIDI output only through `fm1_midi_send_port` from the control task. Never
   through the stock router (`groove_midi_*`), whose poll path is up to 50 ms late.
7. The build must pass all three CI jobs (firmware, installer, simulator).
   The app image must fit: the limit is not known exactly; stock was 453,892
   bytes and 456,292 failed. Stay at or below 453,800. The runtime heap must
   stay at or above 49,152 bytes (build check).
8. Work only on branch `m1-sequencer`. Never push to `main`, never force-push,
   never open a pull request (the default PR target would be the upstream
   M-VAVE repository).

## 3. Code layout

| File | Role |
|---|---|
| `app/seqadv_clock.h` | Rename of `app/seqadv_probe.h`. Interrupt-side clock (`struct seqadv_clock`, `seqadv_clock_poll`) and the timing probes. |
| `app/seqadv_seq.h`, `app/seqadv_seq.c` | Sequencer engine. Pure C, no hardware or SDK includes, so it compiles and runs on a PC for tests. |
| `app/seqadv_ui.inc` | Replaces `app/seqadv_m0.inc`. Included at the end of `app/screen.c` (it reuses that file's font and row buffer). Mode entry/exit, control handling, screen, pad colours, MIDI sending glue, diagnostics page. |
| `tests/seqadv/test_seq.c` | Host tests for the engine (plain `cc`, no framework). |
| `.github/workflows/build.yml` | Add a Linux job that builds and runs the host tests. Add a step printing `build/elite/firmware-report.json` in the firmware job. |

`app/seqadv_seq.c` is compiled into the firmware and the simulator by the
existing build (all `.c` under `app/` except `app/synth/`). Check which
optimisation flags it gets in `sdk/Build.ps1` and add it to the `-Oz` list if
that is how size is controlled there (Build.ps1 is under `sdk/` but is not
listed in the manifest if editing it is needed; verify before touching it, and
prefer not to).

## 4. Engine (`seqadv_seq`)

Resolution: 96 PPQN. A step is a 1/16 note = 24 ticks.

```c
enum { SEQ_STEPS = 128, SEQ_NOTES = 4, SEQ_TRACKS = 1 };   /* tracks: see section 9 */
struct seq_step  { uint8_t note[SEQ_NOTES]; uint8_t count; uint8_t velocity; uint8_t gate; };
                   /* count 0 = empty step; gate in ticks 1..96 (24 = one step) */
struct seq_track { struct seq_step step[SEQ_STEPS]; uint8_t length;      /* 1..128 */
                   uint8_t channel;                                       /* 0..15  */
                   uint8_t ports;                                         /* bit 0 DIN, bit 1 USB */ };
```

API (names can be adjusted, behaviour may not):

- `seq_init()`: one track, length 16, channel 0, ports DIN+USB, all steps empty, default gate 12, default velocity 100.
- `seq_start()`, `seq_stop()`: start always from step 0, tick 0. Stop releases every sounding note.
- `seq_tick(emit)`: advance one 96 PPQN tick. In this order: note-offs that are due, MIDI clock (`0xF8`) on every 4th tick, then on a step boundary the note-ons of that step. `emit(ports, bytes, n)` is supplied by the caller.
- A note that is still sounding when the same note starts again gets its note-off first. No stuck notes on stop, on length change, on step edits, or on channel change (release on the old channel).
- `seq_position()`: current step, for the playhead.
- Step editing: set/clear a step, set its notes (1 to 4), velocity, gate; set length; set channel.
- Start sends `0xFA` before the first clock; stop sends `0xFC`. The first tick (tick 0) carries the first `0xF8` and the notes of step 0.

The engine keeps a small table of sounding notes (note, channel, ports,
remaining ticks). Size it for SEQ_NOTES x 2 per track.

Tempo and the tick deadline stay in the clock (interrupt side); the engine
only counts ticks.

## 5. Mode and controls

Physical IDs come from `smk37_control_event(kind, id, value)`. The Elite's
printed labels differ from the Open firmware's names; verify every ID you use
against `app/smk37_controls.c` (`button_map`, encoder map) and the simulator
panel (`sdk/Simulator.js`), and list the final assignment in the report.

| Control | Function in SeqAdv mode |
|---|---|
| SHIFT (printed KNOB BANK) + highest key | Enter / leave SeqAdv mode. Leaving stops the transport and releases notes. |
| Button printed SEQ PLAY | Play / stop. |
| Pads 1-16 | Steps of the current page. Tap: toggle the step. A step switched on takes the last notes played on the keys (default C4), default velocity and gate. |
| Hold a pad + press keys | Set that step's notes (up to 4; the first key replaces, further keys while the pad is held add). Velocity from the key. |
| OCTAVE- / OCTAVE+ (printed) | Previous / next step page (8 pages of 16). |
| Keys | Play live on the track's channel and ports (note on/off sent at once from the control task). Also remembered as "last notes". |
| Knob K1 | Tempo 30-300 BPM. |
| Knob K2 | Sequence length 1-128. |
| Knob K3 | MIDI channel 1-16. |
| Knob K4 | Gate of the last touched step (and the default for new steps), 1-96 ticks. |
| SHIFT + pad 16 | Diagnostics page on/off (section 7). |

Pad colours (through `smk37_pad_color` in `app/smk37_controls.c`, only while
the mode is active): empty step dim, step with notes lit in one colour,
playhead step white, steps beyond the sequence length off.

All other controls are consumed and ignored in the mode, as M0 does.

## 6. Screen

240 x 240, text rows from the existing 5x7 font at scale 2 (12 rows of 20
characters), drawn one row per call with `fm1_screen_submit`.

Rows:
1. `SEQADV` + `PLAY`/`STOP` + BPM
2. `T1 CH 01 LEN 016 P1/8` (track, channel, length, page)
3. Selected step: number, notes as names (`C4 E4 G4`), velocity, gate
4. to 11. Sequence overview, 8 rows of 16 characters, one per step: `.` empty, `o` has notes, `>` playhead, space beyond the length. The row of the current page is highlighted.
12. Key hints.

Redraw only rows whose text changed (the M0 `dirty_lines` mechanism). The
playhead moves every step: only the affected rows change.

## 7. Diagnostics page

Keep the clock lateness statistics from M0 (LATE MIN/AVG/MAX, PEAK, OVER 1MS,
OUT/RD/DRW, MAX RD/DR, LOST, ERR, RT) as one page, now measured on the real
sequencer ticks. Remove the other M0 pages (port test, interval probes, wake
test, redraw stress) and their interrupt probes to save space, but keep the
phase attribution.

## 8. Simulator

The browser build (`tools/Build-Web.ps1`, define `FM1_OPEN_WEB`) has no scan
interrupt and no 24 MHz counter. In that build, drive `seqadv_clock_poll`
from the control loop with a time derived from `fm1_millis()` so the
sequencer runs (coarse timing is fine there). The mode, controls and screen
must work in the simulator.

## 9. Size and RAM report (part of the deliverable)

Flash and RAM headroom are not known. Find out and report:

- Make the firmware CI job print `build/elite/firmware-report.json` and the
  30 largest symbols or object files from `build/elite/sdk.map` (a few lines
  of PowerShell in the workflow). Artifacts cannot be downloaded from this
  environment; read the job log with `gh run view --log`.
- Report: app image size before and after M1, runtime heap bytes before and
  after, how much flash the factory DX7 bank trim freed, and the largest
  removable candidates in the stock app (by symbol or file) for the later
  strip-down. Do not strip stock features in M1 beyond what is needed to fit;
  if more room is needed, cut the factory DX7 bank further first.
- Keep `SEQ_TRACKS = 1` unless the heap report shows room; say how many
  tracks would fit.

## 10. Tests and verification

Host tests (`tests/seqadv/test_seq.c`, run in CI and locally with `cc`):

1. A 16-step pattern emits clock on every 4th tick and note-ons exactly on
   step boundaries (tick % 24 == 0), in the order offs, clock, ons.
2. Gate: note-off arrives exactly `gate` ticks after its note-on.
3. Length 1, 16, 17 and 128 wrap correctly.
4. Retrigger of a sounding note sends off before on.
5. Stop releases all sounding notes; every note-on has exactly one note-off
   over a long randomised run with edits (length, channel, step changes) in
   the middle.
6. Chords: 4 notes on one step all start on the same tick.

CI: all jobs green on the final commit. Print the firmware Build ID in the
log if practical (it is the SHA-256 described in `sdk/Build.ps1` around the
`identityInputs` code).

## 11. Report back

In the final message: what was built and where; the control assignment with
verified IDs; test results; the section 9 numbers; the final commit hash and
the run ID of the green CI run; anything not done or not verified, stated
plainly. Hardware cannot be tested from this environment: say so rather than
implying it was.
