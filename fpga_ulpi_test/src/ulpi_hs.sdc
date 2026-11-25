create_clock -name ulpi_clk -period 16.667 -waveform {0 8.333} [get_ports {ulpi_clk}]
create_clock -name clk27 -period 37.037 -waveform {0 18.5} [get_ports {clk27}]
