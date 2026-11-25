# BL702 debugger firmware (Tang Primer 20K Dock)

The Dock's USB bridge is a **BL702** running firmware that emulates an FT2232D
(JTAG + UART). When that firmware is lost the chip enumerates as
`ffff:ffff  BLIOT / CDC Virtual ComPort`, `/dev/ttyUSB*` disappear, JTAG dies,
and the kernel logs `error -71` with re-enumeration every minute or two.
**The board then looks completely dead and it is easy to blame the FPGA — it is
not the FPGA.** Check `lsusb` for `0403:6010 SIPEED / JTAG Debugger` first.

## Use this image

**`usb2uartjtag_primer20k.bin`** — built from source in this project.
SHA256 `82bc41315844bf9f10f1f8c687821a7126e2b1f94e06079dcae1839aa7bb726b`.
Verified working: JTAG detects `GW2A(R)-18(C)` and the UART delivers clean
telemetry at 115200.

It is built with `BOARD=bl702_debugger_4k`, which is the config whose pinmap
matches the Primer 20K Dock: **TDI=GPIO0, TDO=GPIO1, TMS=GPIO2, TCK=GPIO15**,
UART TX=GPIO26 / RX=GPIO27.

### How it was built

```bash
sudo pacman -S --needed riscv64-elf-gcc riscv64-elf-newlib riscv64-elf-binutils
# the SDK wants the riscv64-unknown-elf- prefix:
mkdir -p ~/.local/bin
for t in gcc g++ ar objcopy objdump size nm ld strip readelf; do
  ln -sf "$(which riscv64-elf-$t)" ~/.local/bin/riscv64-unknown-elf-$t
done

git clone --recursive https://github.com/sipeed/RV-Debugger-BL702 ~/Work/RV-Debugger-BL702
cd ~/Work/RV-Debugger-BL702/firmware/bl_mcu_sdk
# this SDK predates GCC 14/15; tools/cmake/tools.cmake was patched with
#   -std=gnu17 -Wno-implicit-function-declaration -Wno-implicit-int
#   -Wno-int-conversion -Wno-incompatible-pointer-types
export PATH="$HOME/.local/bin:$PATH"
make BOARD=bl702_debugger_4k APP_DIR=../app APP=usb2uartjtag
# -> out/app/usb2uartjtag/usb2uartjtag_bl702.bin
```

## Images that do NOT work here — kept as warnings

> The failing images below are **Sipeed's and Bouffalo's own binaries**, not
> ours, so they are not redistributed in this repository — only the findings
> are. Get them from
> [sipeed/RV-Debugger-BL702](https://github.com/sipeed/RV-Debugger-BL702) and
> Sipeed's wiki if you want to reproduce the failures. The one image that *is*
> kept here, `usb2uartjtag_primer20k.bin`, was built from that source by the
> procedure above.

| File | What happens |
|---|---|
| `usb2uartjtag_bl702_v1.1.1.bin` | Built for **Tang Nano 9K** (TMS=14, TCK=23, TDI=17, TDO=15). Bridge comes up as FT2232 and a UART node appears, but JTAG fails: `TDO is stuck at 0`. |
| `fpga_partner_bl702_20260630.bin`, `fpga_partner_20260626.bin` | Sipeed's newest, but in `BFNP` full-image format. `bflb_*_tool` refuses them (`BFLB FLASH CFG NOT FIT WITH BOOTHEADER` — bootheader flash config vs this flash, JEDEC `c2201380`). `blisp write -c bl70x` writes and verifies them, but the chip then **does not enumerate at all**. Recoverable, but do not use. |
| `dbger_gowin.bin` | Sipeed's `general/default`. JTAG works, but the UART is unreliable — garbled output, valid-looking lines appearing at two different baud rates, then silence. |

Sipeed's `general` folder covers every Tang board with a BL702 **except nano 9K**
(`dl.sipeed.com/fileList/TANG/Debugger/onboard/BL702`).

## Flashing

1. Enter the ROM bootloader: hold **BOOT** while plugging the USB cable. The
   chip appears as `ffff:ffff BLIOT / CDC Virtual ComPort` on `/dev/ttyACM0`.
   **It must be entered freshly for every attempt** — after a failed run the
   bootloader answers the handshake and then stalls (`BFLB LOAD HELP BIN FAIL`);
   only re-plugging with BOOT held clears it.
2. Flash (tool ships inside the SDK clone, nothing to install):

```bash
cd ~/Work/RV-Debugger-BL702/firmware/bl_mcu_sdk/tools/bflb_flash_tool
./bflb_mcu_tool --chipname=bl702 --interface=uart --port=/dev/ttyACM0 \
    --baudrate=2000000 \
    --firmware=~/Work/fpv-hackrf-hdmi/tools/bl702/usb2uartjtag_primer20k.bin
```

3. Unplug and replug **without** BOOT. Healthy result:
   `0403:6010 SIPEED / JTAG Debugger`, both `ttyUSB0` (JTAG) and `ttyUSB1` (UART),
   and `openFPGALoader --detect` reports `idcode 0x81b ... GW2A(R)-18(C)`.

`blisp` (statically linked) is an alternative writer that also
handles `BFNP` images — it was downloaded from pine64/blisp releases and needs
no installation.
