-- IBUF_sim.vhd
-- Simulation-only model of the Xilinx IBUF primitive.
-- Use this in place of the Unisim IBUF when simulating with
-- Xilinx ISim / ISE without the full Unisim library, or when
-- the Unisim IBUF causes elaboration errors.
--
-- The real IBUF has no logic: it is purely a pin buffer.
-- This model passes the input directly to the output with
-- zero propagation delay, which is correct for simulation.
--
-- Usage: compile this file BEFORE Controller_FPGA2.vhd and
-- the testbench. It will satisfy the component instantiation
-- in Controller_FPGA2 without requiring Unisim.
--
-- DO NOT include this file in the synthesis file list --
-- synthesis must use the real Unisim IBUF primitive.

library ieee;
use ieee.std_logic_1164.all;

-- ============================================================
-- IBUF: single-ended input buffer
-- ============================================================
entity IBUF is
  generic (
    -- These generics mirror the real Unisim IBUF so that any
    -- instantiation that passes generic values still compiles.
    CAPACITANCE : string  := "DONT_CARE"; -- "LOW", "NORMAL", "DONT_CARE"
    IBUF_DELAY_VALUE : string := "0";     -- "0".."16"
    IBUF_LOW_PWR : boolean := true;       -- Low power mode
    IFD_DELAY_VALUE : string := "AUTO";   -- "AUTO", "0".."8"
    IOSTANDARD  : string  := "DEFAULT"
  );
  port (
    I : in  std_logic;   -- Input: connect to top-level port
    O : out std_logic    -- Output: drives internal logic
  );
end entity IBUF;

architecture sim of IBUF is
begin
  -- Pure combinational pass-through, zero delay.
  -- A real IBUF adds ~1-2 ns in hardware; for simulation
  -- the zero-delay model is correct and avoids timing loops.
  O <= I;
end architecture sim;


-- ============================================================
-- IBUFG: global clock input buffer
-- (also commonly needed alongside IBUF)
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity IBUFG is
  generic (
    CAPACITANCE  : string  := "DONT_CARE";
    IBUF_LOW_PWR : boolean := true;
    IOSTANDARD   : string  := "DEFAULT"
  );
  port (
    I : in  std_logic;
    O : out std_logic
  );
end entity IBUFG;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of IBUFG is
begin
  O <= I;
end architecture sim;


-- ============================================================
-- IBUFDS: differential input buffer
-- (needed if any differential pairs use IBUFDS explicitly)
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity IBUFDS is
  generic (
    CAPACITANCE      : string  := "DONT_CARE";
    DIFF_TERM        : boolean := false;
    IBUF_DELAY_VALUE : string  := "0";
    IBUF_LOW_PWR     : boolean := true;
    IFD_DELAY_VALUE  : string  := "AUTO";
    IOSTANDARD       : string  := "DEFAULT"
  );
  port (
    I  : in  std_logic;   -- Positive input
    IB : in  std_logic;   -- Negative input
    O  : out std_logic    -- Output
  );
end entity IBUFDS;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of IBUFDS is
begin
  -- Model the differential pair: output follows the positive input.
  -- A more accurate model would be: O <= '1' when I='1' and IB='0' else
  --                                       '0' when I='0' and IB='1' else 'X';
  -- but the simple version below is sufficient for functional simulation.
  O <= I;
end architecture sim;


-- ============================================================
-- IBUFDS_GTXE1: GTX transceiver reference clock input buffer
-- (Spartan-6 / Virtex-6; include if the design uses GTX)
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity IBUFDS_GTXE1 is
  generic (
    CLKRCV_TRST      : boolean := true;
    CLKCM_CFG        : boolean := true;
    CLKSWING_CFG     : std_logic_vector(1 downto 0) := "11"
  );
  port (
    I       : in  std_logic;
    IB      : in  std_logic;
    CEB     : in  std_logic;
    O       : out std_logic;
    ODIV2   : out std_logic
  );
end entity IBUFDS_GTXE1;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of IBUFDS_GTXE1 is
begin
  O     <= I;
  ODIV2 <= I;
end architecture sim;


-- ============================================================
-- BUFG: global clock buffer
-- (frequently instantiated alongside IBUFs)
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity BUFG is
  port (
    I : in  std_logic;
    O : out std_logic
  );
end entity BUFG;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of BUFG is
begin
  O <= I;
end architecture sim;


-- ============================================================
-- BUFIO: I/O clock buffer (Spartan-6)
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity BUFIO is
  port (
    I : in  std_logic;
    O : out std_logic
  );
end entity BUFIO;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of BUFIO is
begin
  O <= I;
end architecture sim;


-- ============================================================
-- BUFR: regional clock buffer (Spartan-6)
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity BUFR is
  generic (
    BUFR_DIVIDE : string  := "BYPASS";
    SIM_DEVICE  : string  := "SPARTAN6"
  );
  port (
    I   : in  std_logic;
    CE  : in  std_logic;
    CLR : in  std_logic;
    O   : out std_logic
  );
end entity BUFR;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of BUFR is
begin
  -- Simple pass-through; divide functionality not modelled
  O <= I and CE and (not CLR);
end architecture sim;


-- ============================================================
-- OBUF: single-ended output buffer
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity OBUF is
  generic (
    CAPACITANCE : string  := "DONT_CARE";
    DRIVE       : integer := 12;
    IOSTANDARD  : string  := "DEFAULT";
    SLEW        : string  := "SLOW"
  );
  port (
    I : in  std_logic;
    O : out std_logic
  );
end entity OBUF;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of OBUF is
begin
  O <= I;
end architecture sim;


-- ============================================================
-- IOBUF: bidirectional buffer
-- ============================================================
library ieee;
use ieee.std_logic_1164.all;

entity IOBUF is
  generic (
    CAPACITANCE : string  := "DONT_CARE";
    DRIVE       : integer := 12;
    IBUF_DELAY_VALUE : string := "0";
    IFD_DELAY_VALUE  : string := "AUTO";
    IOSTANDARD  : string  := "DEFAULT";
    SLEW        : string  := "SLOW"
  );
  port (
    IO : inout std_logic;  -- Connect to top-level bidirectional port
    I  : in    std_logic;  -- Data input  (from logic to pin)
    O  : out   std_logic;  -- Data output (from pin to logic)
    T  : in    std_logic   -- Tristate enable: '1' = high-Z (input mode)
  );
end entity IOBUF;

library ieee;
use ieee.std_logic_1164.all;
architecture sim of IOBUF is
begin
  IO <= I when T = '0' else 'Z';
  O  <= IO;
end architecture sim;