onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /mavg_tb/UUT/CLK
add wave -noupdate /mavg_tb/UUT/RESET
add wave -noupdate -format Analog-Step -height 84 -max 507.0 -min -505.0 -radix decimal /mavg_tb/UUT/Din
add wave -noupdate /mavg_tb/UUT/DAVin
add wave -noupdate -format Analog-Step -height 84 -max 511.0 -min -512.0 -radix decimal /mavg_tb/UUT/Dout
add wave -noupdate /mavg_tb/UUT/DAVout
add wave -noupdate -radix decimal /mavg_tb/UUT/Mem
add wave -noupdate -radix decimal /mavg_tb/UUT/Accum
add wave -noupdate /mavg_tb/UUT/Done
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 3} {23426921 ns} 0} {{Cursor 2} {57237 ns} 0}
quietly wave cursor active 2
configure wave -namecolwidth 160
configure wave -valuecolwidth 158
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits us
update
WaveRestoreZoom {0 ns} {98309 ns}
