-- Behavioral stub for LinkTx serializer (output-only, for simulation)
library ieee;
use ieee.std_logic_1164.all;

entity LinkTx is
  generic (
    sys_w : integer := 4;
    dev_w : integer := 20
  );
  port (
    DATA_OUT_FROM_DEVICE : in  std_logic_vector(dev_w-1 downto 0);
    DATA_OUT_TO_PINS_P   : out std_logic_vector(sys_w-1 downto 0);
    DATA_OUT_TO_PINS_N   : out std_logic_vector(sys_w-1 downto 0);
    CLK_IN               : in  std_logic;
    CLK_DIV_IN           : in  std_logic;
    LOCKED_IN            : in  std_logic;
    LOCKED_OUT           : out std_logic;
    IO_RESET             : in  std_logic
  );
end entity LinkTx;

architecture sim of LinkTx is
begin
  -- In simulation, pass LOCKED through and drive outputs to '0'
  LOCKED_OUT         <= LOCKED_IN;
  DATA_OUT_TO_PINS_P <= (others => '0');
  DATA_OUT_TO_PINS_N <= (others => '0');
end architecture sim;