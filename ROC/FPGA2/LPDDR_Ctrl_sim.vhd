-- Minimal behavioral stub for LPDDR_Ctrl (MIG macro) for simulation
library ieee;
use ieee.std_logic_1164.all;

entity LPDDR_Ctrl is
  generic (
    C3_P0_MASK_SIZE       : integer := 4;
    C3_P0_DATA_PORT_SIZE  : integer := 32;
    C3_P1_MASK_SIZE       : integer := 4;
    C3_P1_DATA_PORT_SIZE  : integer := 32;
    C3_MEMCLK_PERIOD      : integer := 9400;
    C3_RST_ACT_LOW        : integer := 0;
    C3_INPUT_CLK_TYPE     : string  := "DIFFERENTIAL";
    C3_CALIB_SOFT_IP      : string  := "TRUE";
    C3_SIMULATION         : string  := "FALSE";
    DEBUG_EN              : integer := 0;
    C3_MEM_ADDR_ORDER     : string  := "BANK_ROW_COLUMN";
    C3_NUM_DQ_PINS        : integer := 16;
    C3_MEM_ADDR_WIDTH     : integer := 14;
    C3_MEM_BANKADDR_WIDTH : integer := 2
  );
  port (
    mcb3_dram_dq    : inout std_logic_vector(C3_NUM_DQ_PINS-1 downto 0);
    mcb3_dram_a     : out   std_logic_vector(C3_MEM_ADDR_WIDTH-1 downto 0);
    mcb3_dram_ba    : out   std_logic_vector(C3_MEM_BANKADDR_WIDTH-1 downto 0);
    mcb3_dram_cke   : out   std_logic;
    mcb3_dram_ras_n : out   std_logic;
    mcb3_dram_cas_n : out   std_logic;
    mcb3_dram_we_n  : out   std_logic;
    mcb3_dram_dm    : out   std_logic;
    mcb3_dram_udqs  : inout std_logic;
    mcb3_rzq        : inout std_logic;
    mcb3_dram_udm   : out   std_logic;
    c3_sys_clk_p    : in    std_logic;
    c3_sys_clk_n    : in    std_logic;
    c3_sys_rst_i    : in    std_logic;
    c3_calib_done   : out   std_logic;
    c3_clk0         : out   std_logic;
    c3_rst0         : out   std_logic;
    mcb3_dram_dqs   : inout std_logic;
    mcb3_dram_ck    : out   std_logic;
    mcb3_dram_ck_n  : out   std_logic;
    c3_p2_cmd_clk        : in  std_logic;
    c3_p2_cmd_en         : in  std_logic;
    c3_p2_cmd_instr      : in  std_logic_vector(2 downto 0);
    c3_p2_cmd_bl         : in  std_logic_vector(5 downto 0);
    c3_p2_cmd_byte_addr  : in  std_logic_vector(29 downto 0);
    c3_p2_cmd_empty      : out std_logic;
    c3_p2_cmd_full       : out std_logic;
    c3_p2_wr_clk         : in  std_logic;
    c3_p2_wr_en          : in  std_logic;
    c3_p2_wr_mask        : in  std_logic_vector(3 downto 0);
    c3_p2_wr_data        : in  std_logic_vector(31 downto 0);
    c3_p2_wr_full        : out std_logic;
    c3_p2_wr_empty       : out std_logic;
    c3_p2_wr_count       : out std_logic_vector(6 downto 0);
    c3_p2_wr_underrun    : out std_logic;
    c3_p2_wr_error       : out std_logic;
    c3_p3_cmd_clk        : in  std_logic;
    c3_p3_cmd_en         : in  std_logic;
    c3_p3_cmd_instr      : in  std_logic_vector(2 downto 0);
    c3_p3_cmd_bl         : in  std_logic_vector(5 downto 0);
    c3_p3_cmd_byte_addr  : in  std_logic_vector(29 downto 0);
    c3_p3_cmd_empty      : out std_logic;
    c3_p3_cmd_full       : out std_logic;
    c3_p3_rd_clk         : in  std_logic;
    c3_p3_rd_en          : in  std_logic;
    c3_p3_rd_data        : out std_logic_vector(31 downto 0);
    c3_p3_rd_full        : out std_logic;
    c3_p3_rd_empty       : out std_logic;
    c3_p3_rd_count       : out std_logic_vector(6 downto 0);
    c3_p3_rd_overflow    : out std_logic;
    c3_p3_rd_error       : out std_logic
  );
end entity LPDDR_Ctrl;

architecture sim of LPDDR_Ctrl is
begin
  -- Calibration done immediately
  c3_calib_done   <= '1';
  c3_clk0         <= c3_sys_clk_p;
  c3_rst0         <= c3_sys_rst_i;

  -- DDR outputs idle
  mcb3_dram_a     <= (others => '0');
  mcb3_dram_ba    <= (others => '0');
  mcb3_dram_cke   <= '0';
  mcb3_dram_ras_n <= '1';
  mcb3_dram_cas_n <= '1';
  mcb3_dram_we_n  <= '1';
  mcb3_dram_dm    <= '0';
  mcb3_dram_udm   <= '0';
  mcb3_dram_ck    <= '0';
  mcb3_dram_ck_n  <= '1';
  mcb3_dram_dq    <= (others => 'Z');
  mcb3_dram_udqs  <= 'Z';
  mcb3_dram_dqs   <= 'Z';
  mcb3_rzq        <= 'Z';

  -- Command FIFOs always empty / not full
  c3_p2_cmd_empty  <= '1';
  c3_p2_cmd_full   <= '0';
  c3_p3_cmd_empty  <= '1';
  c3_p3_cmd_full   <= '0';

  -- Write FIFO empty
  c3_p2_wr_full     <= '0';
  c3_p2_wr_empty    <= '1';
  c3_p2_wr_count    <= (others => '0');
  c3_p2_wr_underrun <= '0';
  c3_p2_wr_error    <= '0';

  -- Read FIFO empty
  c3_p3_rd_data     <= (others => '0');
  c3_p3_rd_full     <= '0';
  c3_p3_rd_empty    <= '1';
  c3_p3_rd_count    <= (others => '0');
  c3_p3_rd_overflow <= '0';
  c3_p3_rd_error    <= '0';

end architecture sim;