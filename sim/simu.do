vlib work

vcom ../src/a4988.vhd
vcom ../sim/a4988_tb.vhd

vsim -novopt a4988_tb

view structure
view signals
onerror resume

do wave.do

run 1000 ms
wave zoomfull
