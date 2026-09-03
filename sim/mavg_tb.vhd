library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

entity mavg_tb is end;

architecture bench of mavg_tb is

  signal CLK            : std_logic := '0';
  signal RESET          : std_logic;
  
  signal gravY_v        : std_logic_vector(9 downto 0);
  signal gravY_v_F      : std_logic_vector(9 downto 0);
  signal grav_new_data  : std_logic;
  signal gravY_valid    : std_logic;

  constant period       : time := 20 ns; -- -> 50 MHz System Clock
  signal done           : boolean := false;

begin


UUT : entity work.mavg
  generic map (
    DataWidth => 10,
    NbTaps    => 5  -- 2**5=32 taps -> temps retard = 32 cycles data
  )
  port map (
    CLK         => CLK,
    RESET       => RESET,
    Din         => gravY_v,
    DAVin       => grav_new_data,
    Dout        => gravY_v_F,
    DAVout      => gravY_valid
    );

  CLK <= '0' when done else not CLK after period / 2;
  done <= true after 300 ms;

-- RESET process
process
  begin
    RESET <= '1';
    wait for 5 * period;
    RESET <= '0';
    wait;
  end process;
  
-- process data
process
  begin
    grav_new_data <= '0'; 
    gravY_v <= (others => '0');
    for i in 1 to 100 loop
      wait for 13 us;
      grav_new_data <= '1'; 
      gravY_v <= std_logic_vector(to_signed(100,10));
      wait for period;
      grav_new_data <= '0';
    end loop;
    
    for i in 0 to 10 loop
      wait for 13 us;
      grav_new_data <= '1'; 
      gravY_v <= std_logic_vector(to_signed(100-7**i,10));
      wait for period;
      grav_new_data <= '0';
    end loop;
    
    for i in 1 to 100 loop
      wait for 13 us;
      grav_new_data <= '1'; 
      gravY_v <= std_logic_vector(to_signed(100,10));
      wait for period;
      grav_new_data <= '0';
    end loop;
    
    
    
    
    wait;
  end process;

end architecture bench;
