vlib work

vcom ../src/mavg.vhd
vcom ../sim/mavg_tb.vhd

vsim -novopt mavg_tb

view structure
view signals
onerror resume

do wave_mavg.do

run 5 ms
wave zoomfull
