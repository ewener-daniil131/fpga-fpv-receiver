create_clock -name ulpi_clk -period 16.667 -waveform {0 8.333} [get_ports {ulpi_clk}]
create_clock -name clk27 -period 37.037 -waveform {0 18.5} [get_ports {clk27}]

# The display clock: rPLL 27 MHz x14/3 = 126 MHz serial, CLKDIV /5 = 25.2 MHz
# pixel. Left undeclared, the tool invents a clock for the CLKDIV output ("was
# determined to be a clock but was not created") and then analyses every
# capture-to-pixel crossing against the worst alignment of two unrelated clocks,
# which is where a reported -2.3 ns of setup violation came from. Those
# crossings are handled in RTL - Gray-coded pointers in video_line_fifo, a
# three-stage synchroniser on cap_locked - so they are asynchronous by
# construction and belong in their own clock group.
create_clock -name pix_clk -period 39.683 [get_pins {u_clkdiv/CLKOUT}]
set_clock_groups -asynchronous -group [get_clocks {ulpi_clk}] -group [get_clocks {clk27}] -group [get_clocks {pix_clk}]
