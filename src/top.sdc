# Robot balance V70 top-level timing constraints.
# All internal control logic runs from the DE10-Lite 50 MHz clock.
create_clock -name CLK -period 20.000 [get_ports {CLK}]
derive_clock_uncertainty

# Asynchronous board inputs. ROT_EN is synchronized by two flip-flops before
# entering control logic; RESETn is an asynchronous reset source. HM10 RX and
# MPU digital inputs are handled by their receiving RTL/protocol logic.
set_false_path -from [get_ports {RESETn}]
set_false_path -from [get_ports {ROT_EN}]
set_false_path -from [get_ports {HM10_RX_SYS}]
set_false_path -from [get_ports {MPU9250_MISO}]
set_false_path -from [get_ports {MPU9250_INT}]
set_false_path -from [get_ports {MPU9250_FSYNC}]

# Board-level outputs are not timed against an external synchronous interface.
# Internal register-to-register paths feeding them remain timed by CLK.
set_false_path -to [get_ports {LED*}]
set_false_path -to [get_ports {HM10_TX_SYS}]
set_false_path -to [get_ports {MPU9250_CSn}]
set_false_path -to [get_ports {MPU9250_SCLK}]
set_false_path -to [get_ports {MPU9250_MOSI}]
set_false_path -to [get_ports {MR_A4988_*}]
set_false_path -to [get_ports {ML_A4988_*}]
set_false_path -to [get_ports {HEX*}]
set_false_path -to [get_ports {DP*}]
