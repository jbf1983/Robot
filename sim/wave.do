onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /a4988_tb/UUT_TB/CLK
add wave -noupdate /a4988_tb/UUT_TB/RESETn
add wave -noupdate /a4988_tb/UUT_TB/SW0
add wave -noupdate /a4988_tb/UUT_TB/A4988_STEP
add wave -noupdate /a4988_tb/UUT_TB/A4988_DIR
add wave -noupdate /a4988_tb/UUT_TB/State
add wave -noupdate -radix decimal /a4988_tb/UUT_TB/half_step_cnt
add wave -noupdate /a4988_tb/UUT_TB/RESET
add wave -noupdate -radix decimal /a4988_tb/UUT_TB/half_step_clk
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 3} {32673210 ns} 0} {{Cursor 2} {14056210 ns} 0}
quietly wave cursor active 1
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
WaveRestoreZoom {0 ns} {58617395 ns}
