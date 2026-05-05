#set_property -dict { PACKAGE_PIN H16   IOSTANDARD LVCMOS33 } [get_ports { clk }]
#create_clock -period 4.0 -waveform {0 2} -name clk -add [get_ports clk]
#create_clock -period 3.96 -waveform {0 1.93} -name clk -add [get_ports clk]
create_clock -period 4.5 -name clk -add [get_ports clk]

# 0 ns input/output delays w.r.t. the clock
set_input_delay  0 [get_ports {start[*] rst}]  -clock clk

# Ignore timing analysis for LED outputs
set_false_path -to [get_ports {done bad_sig}]

## Switches
#set_property -dict {PACKAGE_PIN M20 IOSTANDARD LVCMOS33} [get_ports {start[0]}]
#set_property -dict {PACKAGE_PIN M19 IOSTANDARD LVCMOS33} [get_ports {start[1]}]

## Buttons
#set_property -dict {PACKAGE_PIN D19 IOSTANDARD LVCMOS33} [get_ports {rst}]

## LEDs
#set_property -dict {PACKAGE_PIN R14 IOSTANDARD LVCMOS33} [get_ports {done}]
#set_property -dict {PACKAGE_PIN P14 IOSTANDARD LVCMOS33} [get_ports {bad_sig}]