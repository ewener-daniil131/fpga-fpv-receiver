# Vendor DVI IP — not included

`DVI_TX_Top.v` is Gowin's DVI_TX core. It ships as Synplify-encrypted
(IEEE P1735) Verilog and is **not redistributable**, so it is excluded from
this repository via `.gitignore`. The build will fail until you regenerate it.

## Regenerating it

In the Gowin IDE (V1.9.7 or newer), **Tools -> IP Core Generator -> Multimedia
-> DVI/HDMI -> DVI_TX**:

| Setting | Value |
|---|---|
| Module name | `DVI_TX_Top` |
| Device | GW2A-18C (`GW2A-LV18PG256C8/I7`) |
| Video format | RGB888, 8-bit per channel |
| Interface | `I_rst_n`, `I_serial_clk`, `I_rgb_clk`, `I_rgb_vs/hs/de`, `I_rgb_r/g/b` |
| Output | `O_tmds_clk_p/n`, `O_tmds_data_p/n` |

Generate it into this directory as `DVI_TX_Top.v`. The port list above is what
`ulpi_enum_test.v` instantiates, so keep the default Gowin port names.

## Clocking

`TMDS_rPLL.v` (included — it is plain, unencrypted Verilog) is the serial clock
source, and the pixel clock is derived from it:

```
27 MHz  --rPLL--> 243 MHz serial  --CLKDIV /5--> 48.6 MHz pixel
        IDIV_SEL = 0   (/1)
        FBDIV_SEL = 8  (x9)
        ODIV_SEL = 4   (VCO 972 MHz)
```

48.6 MHz is the pixel clock for the 1024x600@60 raster that `fpv_video_out`
generates by default (1312 x 617 total x 60 Hz = 48.57 MHz).

If you retarget the raster, retune the rPLL to match and update the SDC — the
pixel clock is declared there, and leaving it undeclared hides real timing
violations.
