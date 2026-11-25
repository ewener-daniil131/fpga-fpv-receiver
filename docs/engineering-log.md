# Engineering log — FPGA FPV Receiver

> A working log kept during development: what was built, what broke, how each
> defect was found, and what to do next. It is deliberately specific about dead
> ends — those cost the most time and are the most useful thing to write down.
> See [README.md](../README.md) for the overview.

---

Analog FPV video receiver: **HackRF One** as RF front end, **Sipeed Tang Primer
20K + Primer 20K Dock** (Gowin GW2A-LV18PG256C8/I7) running the whole digital
pipeline, output to **HDMI**.

> **Defining constraint: everything runs in the FPGA. No PC and no bridge MCU in
> the signal chain.** The Dock carries a Microchip USB3317 ULPI PHY on FPGA pins;
> the FPGA is the USB 2.0 Hi-Speed host.

Last updated 2026-10-05.

---

## 0. Board status: WORKING (BL702 bridge was the fault)

The long "board is dead" episode was **not** the FPGA and not the design. The
Dock's USB bridge is a **BL702** emulating an FT2232D, and its firmware was
lost. Symptoms, all of which look like a dead board:

- `lsusb` shows `ffff:ffff BLIOT / CDC Virtual ComPort` instead of
  `0403:6010 SIPEED / JTAG Debugger`
- `/dev/ttyUSB*` gone (only `/dev/ttyACM0`, the ROM bootloader)
- UART silent, JTAG `TDO is stuck at 0` or `unable to open ftdi device`
- kernel log: `error -71`, re-enumeration every minute or two

**First check when the board seems dead:** `lsusb | grep 0403:6010`.

Fixed by building the bridge firmware from source with the board config whose
pinmap matches this Dock (`BOARD=bl702_debugger_4k`: TDI=0, TDO=1, TMS=2,
TCK=15). The working image and the full procedure — including three images that
do *not* work and why — are in **`tools/bl702/README.md`**.

Verified after the fix: JTAG detects `GW2A(R)-18(C)`, and the UART delivers
clean telemetry at 115200:

```
V=1D50 AD=1 ST=09FA SI=0E N=467C     HackRF enumerated, Si5351 set, bulk flowing
M0:A=02 E=00 C=00008000              HackRF in RX, no errors
IQ: WR=5640 RD=5640 OCC=0000 OV=0    IQ FIFO healthy, no loss
FPV: CH=1B F=16A8 RS=0076 LK=0       parked on 5800 MHz, no carrier (TX off)
```

**UART is back at 115200** (it had been raised to 460800; the rebuilt bridge is
reliable at 115200, so `tools/grab.py` and the telemetry snippets below use it).
`iq_dump` is no longer instantiated: `field_dump.v` took its place, because a
32768-sample IQ dump only ever covers about 25 lines, while a whole demodulated
field fits in BSRAM at 8 bits per pixel (240 x 256 = 60 kB). Both are armed by
`step_ok[11] & dump_arm`, about 3 s after the radio reaches RX.

## 1. Where the project actually stands

### Working, and measured on hardware
- FPGA-native USB 2.0 Hi-Speed host: enumeration, full control script, and
  **bulk IQ streaming** (~16 MB/s at 8 Msps; currently configured for 20 Msps).
- 40-channel 5.8 GHz scanner: with the camera off every channel read RSSI ~150;
  with it on, channel 27 / 5800 MHz (band F4) rose to ~12000 and the scanner
  parked there.
- FM demodulation is correct. Measured inside one line: sync tip **-2.30 MHz**,
  active video **+0.73 MHz mean**, sync-to-video separation **3.03 MHz**,
  sample-to-sample noise 0.19 MHz against 0.85 MHz video swing.
- Line and field sync: period **1269..1272 samples** against the NTSC 1271,
  `LK=1`, 15 of 15 lines exactly 525 pixels, horizontal drift **0.0 px/line**.
- HDMI output is alive: the dim blue "no lock" field renders, which proves PLL,
  TMDS, raster and the unlocked branch all work.

### Not working yet
**The picture on the monitor is still mush**, and `captures/iq_session.txt` has
now been measured rather than argued about. Two separate causes, in this order:

**1. The USB transport delivers islands, not a stream.** The recording is not
continuous. Inside any 8192-sample block the line period is exactly 1269..1274
and the autocorrelation is 0.7..0.84 - clean, contiguous video. At every
multiple of 8192 samples there is a splice: a phase discontinuity, aligned to
the 8192 grid with vector strength **1.000**. 8192 samples = 16384 bytes = one
half of the HackRF's USB bulk buffer, so whole buffer halves are being lost. Each
dump of 32768 samples is four islands of about 6.4 lines each, with an unknown
but large amount of real time missing between them.

That is why the offline decode in `captures/fpv_full_frame.png` is four narrow
bands of real, recognisable picture separated by black: the picture content is
genuine and the RF, USB and demodulation are all proven end to end. **What is
missing is time, and no sync separator can reconstruct time that was never
delivered.** Fixing this is now the top item - see section 7.

**2. The sync separator was slicing inside the picture** - found and fixed, see
defect 12 in section 3. It is replaced by `ntsc_sync_dpll.v`.

Still open from before: the signal was clipping (49.7% and 100% of samples at the
ADC rail, `|z|` 134 and 152 against a limit of 127) at VGA 38 dB. VGA was dropped
38 -> 24 dB in control script step 9 and the AGC loop of section 2 is now
implemented, so this should be self-correcting; it has not yet been confirmed on
hardware.

---

## 2. Automatic gain — implemented, not yet confirmed on hardware

Measured carrier levels across the session, same camera, same channel:

| Condition | `\|z\|` | VGA |
|---|---|---|
| camera far / weak | 17 | 36 dB |
| camera mid | 90 | 44 dB |
| camera close | 152 (clipped) | 38 dB |

That is a **19 dB spread**, which no fixed gain covers: set it for the weak case
and the close case clips; set it for the close case and the weak case sinks into
noise. **The fix is an AGC loop**, and the design already has everything needed:

- `rf_rssi` from `fm_demod` is the level measurement.
- `H_RTUNE` already exists as a path for issuing a control request from the
  running host FSM (it is used for `SET_FREQ` retuning by the scanner).

This is now built, in `ulpi_enum_test.v` (search `AGC_HI`): every 200 ms, if
`rf_rssi` is above 8500 and the gain is above its minimum, VGA drops 2 dB; below
5000 and under the maximum, it rises 2 dB. The request goes out as vendor
request 20 through `H_AGC`/`H_RTUNE`, the same path the scanner retunes with, and
the gain in dB is reported on the `VO:` telemetry line as `G=`. It targets `|z|`
of about 60–80, i.e. roughly half the ADC range. **It has never run against a
live transmitter** — that is step 3 of section 7.

**Important and counter-intuitive:** the gain requests *do* take effect even
though `step_ok` bits 9 and 10 always read 0. Those are IN requests and their
status stage never completes, so the flag lies. Proven by sweeping VGA 36 -> 56
dB and watching the noise floor rise 258 -> 2425 (×9.4 for +20 dB). **Do not
trust `step_ok` for any IN request** — bits 0, 2, 4, 9, 10, 12, 13 read 0 while
the transfers actually work (the descriptor and `SI=0E` are read successfully).

---

## 3. Defects found and fixed this session

Each was real, each blocked the picture, and none were visible from the final
output alone.

1. **De-emphasis filter, signedness.** A concatenation is unsigned in Verilog,
   and one unsigned operand made the whole expression unsigned, degrading `>>>`
   to a logical shift. The filter ran away into saturation and destroyed the
   sync tip specifically. Diagnostic: `std(out)/std(in)` was **2.80** for a
   filter that can only attenuate; now 0.73. Fixed with `$signed(...)`.
2. **De-emphasis corner far too low.** `alpha = 1/8` at 20 Msps puts the corner
   at **400 kHz** while NTSC video runs to 4.2 MHz — horizontal detail was cut
   roughly tenfold, which looked like "lines fine, content smeared". Changed to
   `alpha = 1/2` (1.6 MHz). Measured effect: horizontal detail **7.23 -> 23.89**.
3. **Field detection by serration width.** Noise fragments a 27 us serration, so
   `broad` counted 0, `line_cnt` never reset and free-ran to 495, and almost no
   pixels passed the active-window test. Replaced by counting **missed lines**
   (4 in a row = vertical interval). Effect: lines/field **15–23 -> 231–252**
   against the true 262.
4. **Two-line ping-pong buffer.** USB delivers IQ in bursts and the sample FIFO
   is drained whenever non-empty, so the demodulator runs in spurts and line
   strobes jitter by tens of microseconds while the raster is regular. The
   reader landed mid-swap and stitched halves of two lines. Replaced with
   `video_line_fifo.v`: elastic 16-line ring, Gray-coded pointers, repeats a
   line when starved, re-centres if lapped. Simulated bursty: 1167 lines, 0 torn.
5. **Sync accepted at any moment.** Noise injected extra pulses that cut lines
   short; line lengths varied (median 504, range 42..1028) and every following
   line started at a different offset — diagonal mush. Added an acceptance
   window (`LINE_MIN..LINE_MAX`) once locked.
6. **Flywheel ran at the wrong period, and self-latched.** It advanced a line
   every `LINE_TIMEOUT` = 1300 samples while NTSC is 1271, walking the picture
   14.5 px/line. Worse: after it fired the counter was zeroed, so the next real
   pulse arrived at 1242 — outside the 1250..1295 window — and was rejected
   forever. Fixed by carrying the remainder (`samples_since_hsync -
   LINE_NOMINAL`) instead of zeroing. Effect: drift **42–55 px -> 0.0 px/line**,
   line correlation 0.63 -> 0.86, and in simulation at 6.9 dB SNR the design now
   *holds* lock (it never did before) and delivers 99.7% of expected pixels.
7. **Lock latched high on signal loss.** `ntsc_locked` is only updated when a
   sync pulse is found, so it held `1` forever with no signal. Added a watchdog
   (`LOSS_SAMPLES`, 3.3 ms).
8. **Screen showed demodulated noise as if it were a picture.** With no carrier
   the demodulator emits random phase and the flywheel still runs the raster.
   Display is now gated on `rf_rssi > 0x0600` as well as lock, so "no signal"
   renders as the blue field. This matters for diagnosis: several times a noise
   capture was mistaken for a broken picture.
9. **UART telemetry muted indefinitely by `iq_dump.v`.** In `iq_dump.v`, `busy <= 1'b1;`
   was asserted upon arm in `S_IDLE` and held throughout `S_CAP`. When waiting for
   32,768 samples from a paused or stalled radio, `dump_busy` locked high permanently.
   `ulpi_enum_test.v` suppressed normal periodic telemetry whenever `dump_busy` was
   high, making the board appear completely silent / dead on `/dev/ttyUSB1`. Fixed:
   `busy` is now 0 during `S_IDLE`, `S_CAP`, and `S_PAUSE`, asserting only while hex
   bytes are actively transmitted on UART (`S_HDR` -> `S_SEP`).
10. **FPGA reset unasserted on SRAM load.** Gowin devices do not cycle the external
    T10 button pin on SRAM programming. Without a manual press, registers and CDC
    synchronizers (`rst_sync_c`, `rst_sync_u`) could initialize to undefined states or
    miss the clean PHY reset window. Added internal 255-cycle synchronous Power-On Reset
    (`boot_por_cnt` / `boot_por_n`) to drive `sys_rst_n = boot_por_n & rst_n;`.
12. **The sync slicer sliced inside the picture.** `ntsc_sync_separator.v` tracked
    the sync tip with "fast attack down, slow leak up", but the leak was **+1 per
    sample** with a ceiling of 80. From a real tip of 40 it reached that ceiling
    in 2 us and stayed there **87% of the time**, so the slicing level, `tip+25`,
    sat at **105** - against a measured sync tip of 37, blanking of 85 and picture
    median of 135. Dark picture content was therefore sliced as sync. Measured on
    the recording with the module's own logic ported to Python: line-period
    jitter **154.5 samples**, and `ntsc_locked` never asserted at all on clean or
    24 dB input. Since the display is gated on lock, that alone produces the blue
    "no signal" field, which is the reported symptom.

    Replaced by `ntsc_sync_dpll.v`: the slice is halfway between two *measured*
    levels (sync tip, and back porch sampled in a window gated by the pulse's own
    trailing edge, not by the loop phase), the line timing comes from an NCO
    whose period is learned by an integrator, and the vertical interval is found
    by integrating sync over a line rather than by measuring one serration.
    Measured on the same input, through the RTL: jitter **0.20 samples**, lock in
    2-3 lines, and **100% of the expected pixels** at the recording's real SNR.
    Two bugs were found while building it, both worth remembering:
    the signedness trap again (an unsigned operand in the blanking-level IIR
    turned `>>>` into a logical shift and walked the level up to 222), and a
    dropped NCO increment on the cycle that applied a phase correction, which
    biased the loop by exactly KP samples and dropped lock every LOSS_N lines.
13. **USB connection instability (`error -71`).** Physical cable / port looseness on
    the USB hub caused the FT2232 chip to repeatedly reset, drop address, and re-enumerate
    up to device index 70+, wiping FPGA SRAM in the process. Stabilized with cable re-seat
    as persistent device 081.

---

## 4. Hardware rules and facts

- **Never power the HackRF from the Dock's J8 +5V/VBUS rail.** Use a powered hub
  or splitter: data to the Dock, power from an external supply.
- **Never remove the flash interlock** in `ulpi_enum_test.v`.

### The flash-erase incidents — root cause, confirmed
The HackRF firmware was destroyed four times. The cause was **not** power. The
control script's step 12 was labelled "SI5351C_READ" but sent vendor request
**10**, which per `hackrf-2026.01.3/host/libhackrf/src/hackrf.c:57` is
`SPIFLASH_ERASE`. Every successful run erased the flash.

    3 MAX283X_READ   4 SI5351C_WRITE   5 SI5351C_READ
    10 SPIFLASH_ERASE  11 SPIFLASH_WRITE  12 SPIFLASH_READ
    14 BOARD_ID_READ   16 SET_FREQ  17 AMP_ENABLE  19 LNA_GAIN
    20 VGA_GAIN  41 GET_M0_STATE

Fixed: request 5 for the Si5351C readback, and
`req_is_flash = (ctl_req >= 10) && (ctl_req <= 12)` remaps 10..12 to 14.
An earlier interlock covered 3/4/5 — all harmless — because the numbering was
never checked against the device's own source.

### The transmitter
It is switched off by hand between tests because it gets hot. That is *not* a
fault, and not thermal foldback — a lot of time was lost waiting for a signal
that was simply switched off. Work in short bursts: tell the user to switch on,
capture for 14 s, done.

### Pin map
| Signal | Pin | Notes |
|---|---|---|
| Board clock | H11 | 27 MHz |
| Reset | T10 | **button**; needs `set_option -use_sspi_as_gpio 1` |
| Debug UART TX | M11 | FT2232 ch. B, `/dev/ttyUSB1`, **115200** 8N1 |
| JTAG | FT2232 ch. A | `openFPGALoader` |
| ULPI data[7:0] | R12 P13 R13 T14 H13 J12 H12 G11 | D7..D0 |
| ULPI dir/nxt/stp | K12 / K13 / K11 | |
| ULPI clk | T15 | 60 MHz from PHY, sampled on falling edge |
| ULPI reset | F10 | active low |
| LEDs | L16 L14 N14 N16 | LED0 = heartbeat |
| TMDS clk +/- | G16, H15 | |
| TMDS data0/1/2 | H14,H16 / J15,K16 / K14,K15 | |

---

## 5. Build, flash, measure

```bash
cd fpga_ulpi_test
gw_sh build.tcl                                   # synth + P&R
openFPGALoader -b tangprimer20k impl/pnr/enum.fs  # SRAM load

python3 ../tools/grab.py 14        # one 14 s capture + full analysis
```

`tools/grab.py` prints, in one shot: RSSI, lock, per-dump `|z|` / clipping /
line period / periodicity, and for the best dump the reconstructed picture —
line correlation, horizontal detail, interval consistency, ASCII preview. It
saves the frame to `last_img.npy`. **This is the main instrument; use it.**

**Gowin licence** is node-locked to the MAC of a USB-Ethernet adapter
(`00:e8:00:33:e3:54`, ICS Advent `0fe6:9702`). When it is unplugged the build
fails with `License hostid not match` — and note that `openFPGALoader` will then
happily flash the *previous* bitstream, so changes silently do not apply. Check
`ip link | grep 00:e8:00:33:e3:54` when a change seems to have no effect.

Simulation (no board needed). The sync separator has its own generator, its own
reference model and a test bench that prints the same numbers as the model, so a
change can be judged without hardware:

```bash
# 1. make a test signal: three NTSC fields, FM modulated, at a chosen SNR.
#    24 dB matches the noise measured in captures/iq_session.txt.
python3 tools/ntsc_gen.py --fields 3 --snr 24 --out /tmp/iq.hex
#    --gap-every/--gap-len punch out blocks of samples, the way the USB
#    transport loses buffer halves.

# 2. demodulate it to baseband video (bit-exact against fm_demod.v: mean
#    difference 0.068 levels, 93.3% of samples identical), then:
iverilog -g2012 -o /tmp/tb -DVIDFILE='"/tmp/vid.hex"' -DNSAMP=999613 \
    sim/tb_sync_dpll.v src/ntsc_sync_dpll.v && vvp /tmp/tb
#    -DOLD runs the same measurement against the superseded separator.
#    -DGAPEVERY=8192 pulses stream_gap, as the transport would.

# 3. the Python reference model, for trying a change in seconds:
python3 tools/sync_model.py <baseband video, one level per line>

iverilog -o /tmp/tb sim/tb_vout2.v src/fpv_video_out.v src/video_line_fifo.v   # display path
```

Measured through the RTL on three fields, same input to both modules:

| | old separator | `ntsc_sync_dpll` |
|---|---|---|
| clean | never locks, 190546 px, jitter 32.3, mean period 1185 | locks, **378000 px**, jitter 0.99, period 1271.00 |
| 24 dB | never locks, 189449 px, jitter 31.4 | locks, **378000 px**, jitter 1.00 |
| 16 dB | locks late, 4 vsyncs for 3 fields, jitter 88.0 | locks, **378000 px**, 3 vsyncs, jitter 1.03 |
| 12 dB | never locks | locks, 277725 px, jitter 1.22 |

On the 16 recorded islands of real video, steady-state line-period jitter is
**0.20 samples** against **154.5** for the old design.
`tools/gowin_stubs.v` stubs rPLL / CLKDIV / DVI_TX_Top so the whole top level
elaborates under iverilog (the vendor DVI IP is encrypted and cannot be read).

### Telemetry (UART, 115200)
```
V=..  AD=1 ST=.... SI=0E N=....     USB status, N = bulk packets
M0:A=02 E=00 C=...                 HackRF M0 state
IQ: WR=.. RD=.. OCC=.. OV=0        IQ FIFO health
FPV: CH=.. F=.. RS=.. LK=. SC=.    channel, freq, RSSI, lock, scanning
VD: MIN=.. MAX=.. H=<8 bins>       video level distribution
SY: T=.. H=.. M=.. B=.. P=..       sync census: tiny/H/mid/broad, line period
    ... SL=.. TP=.. BK=.. LP=.....  slicer: slice / tip / blanking, and the
                                   NCO line period in 1/256 sample (4F700 = 1271.00)
VO: PX=.. LN=.. FD=.. AV=.. ...    display path counters
```
`SY: H=` near 103 per window and `P=` near 1271 means the video path is healthy.
**`VO: PX` overflows its 16 bits ~125 times per interval — its delta is
meaningless.** `AV` (mean over 8192 pixels) is the usable indicator.

---

## 6. Source map (`fpga_ulpi_test/src/`)

| File | Role |
|---|---|
| `ulpi_enum_test.v` | Top: ULPI engine, USB host FSM, control script, bulk polling, telemetry, integration |
| `usb_iq_depacketizer.v`, `usb_iq_fifo.v`, `async_fifo.v` | Bulk packets -> IQ samples, 60->27 MHz CDC |
| `cordic_atan2.v` | 10-stage vectoring CORDIC, **pi = 32768**. Verified bit-exact against a reference atan2 (correlation 1.0000) — do not re-suspect it |
| `fm_demod.v` | Conjugate product, CORDIC, de-emphasis, RSSI |
| `ntsc_sync_dpll.v` | **In use.** Sync slicer on measured levels, line-locked NCO with a learned period, vertical by sync integration, raster coords. Parameterised by sample rate |
| `ntsc_sync_separator.v` | Superseded, kept so the older test benches still build. Do not instantiate: its slicer sits inside the picture (defect 12) |
| `fpv_auto_scanner.v` | 40-channel sweep, scores by RSSI + lock |
| `video_line_fifo.v` | Elastic 16-line FIFO, capture domain -> pixel domain |
| `fpv_video_out.v` | 640x480@60 raster, 525->640 DDA upscale, line doubling, genlock |
| `dvi/DVI_TX_Top.v`, `dvi/TMDS_rPLL.v` | Vendor DVI IP (encrypted) + PLL retuned to 126 MHz |
| `video_diag.v`, `iq_dump.v`, `field_dump.v` | Diagnostics: level histogram / pulse census, raw IQ over UART, one whole demodulated field over UART |

**Display clocking (corrected 2026-11-25):** the text here used to claim
640x480@60 off a 126 MHz serial / 25.2 MHz pixel clock. That was stale — it
described an earlier revision and no longer matched the code. What is actually
built:

- `TMDS_rPLL.v`: 27 MHz, `IDIV_SEL=0` (/1), `FBDIV_SEL=8` (x9) → **243 MHz**
  serial, VCO 972 MHz; `CLKDIV DIV_MODE=5` → **48.6 MHz** pixel.
- `fpv_video_out` defaults, which `ulpi_enum_test.v` does not override:
  `H_ACT=1024, H_FP=40, H_SYNC=104, H_BP=144` and
  `V_ACT=600, V_FP=3, V_SYNC=4, V_BP=10` → **1024x600@60**,
  H_TOTAL 1312, V_TOTAL 617. 1312 x 617 x 60 = 48.57 MHz, which is the pixel
  clock above — so the two are consistent.
- Scaling is therefore not 2x line doubling: `STEP = 525*1024/1024` horizontally
  and `VSTEP = 240*1024/600` vertically, i.e. ~1.95x across and 2.5x down. The
  output line rate is 1312 x 60 = 37.02 kHz against the source's 15.734 kHz, so
  the 16-line elastic FIFO plus genlock is what replaces a frame buffer — a full
  field at 8 bpp is ~1 Mbit and the device has 828 kbit of BSRAM.

Current usage 32/46 BSRAM.

**Sample rate:** currently 20 Msps / 20 MHz baseband filter / 12-packet bulk
burst. Measured deviation is only about -1.6..+2.3 MHz, so **8 Msps would be
enough** and is the configuration proven at 16 MB/s. To revert: step 5 payload
`00 12 7A 00`, step 6 filter `wValue=0x8D80, wIndex=91`, `bburst != 4'd3`, and
instantiate `ntsc_sync_separator` with `HSYNC_MIN=25, HSYNC_MAX=55,
BROAD_MIN=150, LINE_MIN=500, LINE_MAX=516, LINE_NOMINAL=508, ACT_START=76,
ACT_END=496, PIX_SHIFT=0` plus `fpv_video_out` with `SRC_PIX=420`. All are
module parameters.

---

## 7. What to do next, in order

1. **Flash and verify telemetry.** `impl/pnr/enum.fs` was rebuilt 2026-10-05 with
   `ntsc_sync_dpll`, the AGC loop, `field_dump`, the POR generator, and an SDC
   that finally declares the pixel clock. Timing is clean: worst setup **+5.42
   ns**, worst hold **+0.21 ns**, no violated paths. Usage: logic 37%, registers
   24%, **BSRAM 44/46 (96%)**, DSP 2.
   - `openFPGALoader -b tangprimer20k impl/pnr/enum.fs`
   - Confirm telemetry on `/dev/ttyUSB1` at 115200 with the camera off.
   - The `SY:` line now ends with `SL=.. TP=.. BK=.. LP=.....`. **Check these
     first.** Healthy with a carrier: `TP` near 25..45, `BK` near 80..110, `SL`
     between them (50..80), `LP` near `4F700`. `SL` up at 105 with `TP` at 80 was
     the old defect, and it was invisible until these counters existed.

2. **Fix the transport - this is the one that is actually blocking the picture.**
   The IQ stream arrives as islands of 8192 samples with real time missing in
   between (section 1). Two independent levers, and they combine:
   - **Raise the bulk duty cycle.** `H_BRXW` takes 12 packets back to back
     (`bburst` 0..11) and then sits in `H_OBS` for `twait >= 5000` cycles at
     60 MHz = **83 us** before issuing a SOF. Twelve packets are roughly 110 us of
     wire time, so about 40% of every cycle is spent idle while the radio keeps
     producing. `H_SOFI` already exists to inject a microframe from inside
     another transfer - use the same trick here: keep issuing INs and insert a
     SOF only when the 125 us microframe timer demands one.
   - **Drop the sample rate.** 20 Msps needs 40 MB/s, which is 75% of the
     theoretical Hi-Speed bulk ceiling and leaves no margin for FSM overhead.
     8 Msps needs 16 MB/s and is the configuration already proven at that rate;
     section 6 lists every parameter to change. Measured deviation is only
     -1.6..+2.3 MHz, so 8 Msps carries the signal.
   - Confirm with telemetry rather than by eye: `IQ: OV` must stay 0 **and** the
     `N=` packet count must rise by the full expected number per interval.
     A capture whose islands are longer than 8192 samples is the proof.

3. **Verify RF reception with the camera on (14 s burst).** Switch the
   transmitter on for about 15 s only - it overheats, and a lot of time has been
   lost to a transmitter that was simply off.
   - `python3 tools/grab.py 14`
   - With the AGC running, expect clipping < 1% and `|z|` of 60..80.

4. **Then, and only then, judge the picture.** With a continuous stream the sync
   separator delivers 100% of the expected pixels at the recording's own SNR in
   simulation, so if the picture is still wrong after step 2 the fault is
   downstream - raster geometry (`ACT_START` / `VACT_START`) or the display path.
   `field_dump` puts a whole demodulated field on the UART, which settles
   "the demodulator is wrong" against "the raster is wrong" in one capture.

## 8. How to debug this thing

The technique that broke the deadlock twice, and the one to reach for first:
**dump an intermediate stream over UART and analyse it in Python.**

- Raw IQ proved the RF and USB were fine and the demodulator was lying.
- The pixel stream proved the picture was real and cleared the line buffer —
  after two iterations had already been spent on the buffer and field anchoring,
  neither of which was the cause.

Corollary learned the hard way: an aggregate statistic can look like success
without being one. Adjacent-line correlation of 0.81 was read as "there is a
picture" when the frame was in fact bands of noise. Reconstruct and *look*, and
prefer a measurement with a known expected value (line period 1271, pixels per
line 525, clipping 0%) over a similarity score.

