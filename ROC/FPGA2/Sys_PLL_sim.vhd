-- Behavioral simulation model for Sys_PLL (ISim/VHDL-93)
-- Replace the Xilinx coregen black-box for simulation only.
library ieee;
use ieee.std_logic_1164.all;

entity Sys_PLL is
  port (
    CLK_IN1_P : in  std_logic;
    CLK_IN1_N : in  std_logic;
    CLK_OUT1  : out std_logic;  -- 500 MHz BitClk
    CLK_OUT2  : out std_logic;  -- 100 MHz SysClk
    CLK_OUT3  : out std_logic;  -- 200 MHz RxFMClk
    CLK_OUT4  : out std_logic;  --  50 MHz i50MHz
    RESET     : in  std_logic;
    LOCKED    : out std_logic
  );
end entity Sys_PLL;

architecture sim of Sys_PLL is
  signal clk_in : std_logic := '0';
  signal out1, out2, out3, out4 : std_logic := '0';
  signal locked_int : std_logic := '0';
  signal rst_r : std_logic_vector(3 downto 0) := "0000";
begin

  clk_in <= CLK_IN1_P;

  -- 500 MHz  (period = 2 ns)
  out1_gen : process
  begin
    while true loop
      out1 <= '0'; wait for 1 ns;
      out1 <= '1'; wait for 1 ns;
    end loop;
  end process;

  -- 100 MHz  (period = 10 ns)
  out2_gen : process
  begin
    while true loop
      out2 <= '0'; wait for 5 ns;
      out2 <= '1'; wait for 5 ns;
    end loop;
  end process;

  -- 200 MHz  (period = 5 ns)
  out3_gen : process
  begin
    while true loop
      out3 <= '0'; wait for 2.5 ns;
      out3 <= '1'; wait for 2.5 ns;
    end loop;
  end process;

  -- 50 MHz  (period = 20 ns)
  out4_gen : process
  begin
    while true loop
      out4 <= '0'; wait for 10 ns;
      out4 <= '1'; wait for 10 ns;
    end loop;
  end process;

  -- Assert LOCKED after 100 ns (simulates PLL lock time)
  lock_gen : process(RESET, out2)
  begin
    if RESET = '1' then
      rst_r      <= "0000";
      locked_int <= '0';
    elsif rising_edge(out2) then
      rst_r(0) <= '1';
      rst_r(1) <= rst_r(0);
      rst_r(2) <= rst_r(1);
      rst_r(3) <= rst_r(2);
      if rst_r(3) = '1' then
        locked_int <= '1';
      end if;
    end if;
  end process;

  CLK_OUT1 <= out1;
  CLK_OUT2 <= out2;
  CLK_OUT3 <= out3;
  CLK_OUT4 <= out4;
  LOCKED   <= locked_int;

end architecture sim;