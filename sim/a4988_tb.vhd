library ieee;
  use ieee.std_logic_1164.all;

entity a4988_tb is end;

architecture bench of a4988_tb is

  signal CLK          : std_logic := '0';
  signal RESETn       : std_logic;
  
  signal SW0          : std_logic;  
  signal LEDR0        : std_logic;  

  signal A4988_STEP   : std_logic;
  signal A4988_DIR    : std_logic := '0';

  constant period     : time := 20 ns; -- -> 50 MHz System Clock
  signal done         : boolean := false;

begin

  UUT_TB: entity work.a4988
    port map (
      CLK           => CLK,
      RESETn        => RESETn,
      SW0           => SW0,
      LEDR0         => LEDR0,
      A4988_STEP    => A4988_STEP,
      A4988_DIR     => A4988_DIR
    );

  CLK <= '0' when done else not CLK after period / 2;
  done <= true after 300 ms;

  -- RESET process
  process
  begin
    RESETn <= '0';
    wait for 5 * period;
    RESETn <= '1';
    wait;
  end process;

end architecture bench;
