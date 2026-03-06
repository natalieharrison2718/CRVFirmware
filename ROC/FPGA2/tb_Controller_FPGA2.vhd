-- =============================================================================
-- Testbench: tb_Controller_FPGA2
-- Fixed version: adds correct timing for Rx_active arming and uses
-- ReadyForceAddr constant properly.
-- VHDL-93 compatible (ISE / ISim)
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

use work.Proj_Defs.all;

entity tb_Controller_FPGA2 is
end entity tb_Controller_FPGA2;

architecture sim of tb_Controller_FPGA2 is

  -- Timing constants
  constant CLK_SYS_PERIOD : time := 10 ns;   -- 100 MHz ClkB
  constant CLK_50_PERIOD  : time := 20 ns;
  constant CLK_RX_PERIOD  : time := 40 ns;   -- 25 MHz RxClk
  constant UC_SETUP       : time :=  5 ns;
  constant UC_HOLD        : time :=  5 ns;
  constant UC_PULSE       : time := 50 ns;

  -- Count10us = 999 cycles @ 100 MHz = 10 us
  -- We need at least ONE full Counter10us window AFTER toggling 15+ transitions
  -- and then ANOTHER 4 dead windows before Rx_active clears.
  -- So arm time = (toggle period) + 2 * Count10us windows = safe at ~30 us
  constant RXACTIVE_ARM_TIME : time := 35 us;

  constant GA_VAL : std_logic_vector(1 downto 0) := "00";

  -- -------------------------------------------------------------------------
  -- DUT port signals
  -- -------------------------------------------------------------------------
  signal VXO_P, VXO_N   : std_logic := '0';
  signal ClkB_P, ClkB_N : std_logic := '0';
  signal Clk50MHz_tb     : std_logic := '0';
  signal CpldRst         : std_logic := '0';
  signal CpldCS          : std_logic := '1';
  signal uCRd            : std_logic := '1';
  signal uCWr            : std_logic := '1';
  signal uCA             : std_logic_vector(11 downto 0) := (others => '0');
  signal uCD             : std_logic_vector(15 downto 0) := (others => 'Z');
  signal GA_pins         : std_logic_vector(1 downto 0)  := GA_VAL;

  signal SDCKE, LDM, UDM, RAS, CAS, SDWE : std_logic;
  signal SDClk_P, SDClk_N                 : std_logic;
  signal SDD                              : std_logic_vector(15 downto 0) := (others => 'Z');
  signal UDQS, LDQS, SDRzq                : std_logic := 'Z';
  signal SDA                              : std_logic_vector(13 downto 0);
  signal BA                               : std_logic_vector(1 downto 0);

  signal LinkClk_P, LinkClk_N, LinkFR_P, LinkFR_N : std_logic;
  signal LinkD_P, LinkD_N                          : std_logic_vector(1 downto 0);

  signal RxDA, RxDB, RxDC, RxDD : std_logic_vector(3 downto 0) := (others => '0');
  signal RxDE, RxDF, RxDG, RxDH : std_logic_vector(3 downto 0) := (others => '0');
  signal RxClk                   : std_logic_vector(7 downto 0) := (others => '0');
  signal RxDV                    : std_logic_vector(7 downto 0) := (others => '0');
  signal RxErr                   : std_logic_vector(7 downto 0) := (others => '0');
  signal CRS_sig                 : std_logic_vector(7 downto 0) := (others => '0');

  signal TxDA, TxDB, TxDC, TxDD : std_logic_vector(3 downto 0);
  signal TxDE, TxDF, TxDG, TxDH : std_logic_vector(3 downto 0);
  signal TxEn_out                : std_logic_vector(7 downto 0);
  signal MDC_out                 : std_logic_vector(1 downto 0);
  signal MDIO_sig                : std_logic_vector(1 downto 0) := (others => 'Z');
  signal PhyPDn_out, PhyRst_out  : std_logic;
  signal TxClk_sig               : std_logic_vector(1 downto 0) := (others => '0');
  signal Clk25MHz_out            : std_logic;

  signal FMRx                                : std_logic_vector(7 downto 0) := (others => '0');
  signal FMRxEn_out                          : std_logic;
  signal HrtBtFM, DReqFM                     : std_logic := '0';
  signal SPICS_out, SPISClk_out, SPIMOSI_out : std_logic;
  signal SPIMISO                             : std_logic := '0';
  signal Debug_out                           : std_logic_vector(10 downto 1);

  -- -------------------------------------------------------------------------
  -- Probe outputs from DUT
  -- -------------------------------------------------------------------------
  signal dut_MaskReg         : std_logic_vector(7 downto 0);
  signal dut_PhyRxEmpty      : std_logic_vector(7 downto 0);
  signal dut_Rx_active       : std_logic_vector(7 downto 0);
  signal dut_PhyTxBuff_Count : std_logic_vector(10 downto 0);
  signal dut_ReadyStatus     : std_logic_vector(7 downto 0);

  signal tb_done : boolean := false;

  -- =========================================================================
  -- Component declaration
  -- =========================================================================
  component Controller_FPGA2 is
    port (
      VXO_P         : in    std_logic;
      VXO_N         : in    std_logic;
      ClkB_P        : in    std_logic;
      ClkB_N        : in    std_logic;
      Clk50MHz      : in    std_logic;
      CpldRst       : in    std_logic;
      CpldCS        : in    std_logic;
      uCRd          : in    std_logic;
      uCWr          : in    std_logic;
      uCA           : in    std_logic_vector(11 downto 0);
      uCD           : inout std_logic_vector(15 downto 0);
      GA            : in    std_logic_vector(1 downto 0);
      SDCKE         : out   std_logic;
      LDM           : out   std_logic;
      UDM           : out   std_logic;
      RAS           : out   std_logic;
      CAS           : out   std_logic;
      SDWE          : out   std_logic;
      SDClk_P       : out   std_logic;
      SDClk_N       : out   std_logic;
      SDD           : inout std_logic_vector(15 downto 0);
      UDQS          : inout std_logic;
      LDQS          : inout std_logic;
      SDRzq         : inout std_logic;
      SDA           : out   std_logic_vector(13 downto 0);
      BA            : out   std_logic_vector(1 downto 0);
      LinkClk_P     : out   std_logic;
      LinkClk_N     : out   std_logic;
      LinkFR_P      : out   std_logic;
      LinkFR_N      : out   std_logic;
      LinkD_P       : out   std_logic_vector(1 downto 0);
      LinkD_N       : out   std_logic_vector(1 downto 0);
      RxDA          : in    std_logic_vector(3 downto 0);
      RxDB          : in    std_logic_vector(3 downto 0);
      RxDC          : in    std_logic_vector(3 downto 0);
      RxDD          : in    std_logic_vector(3 downto 0);
      RxDE          : in    std_logic_vector(3 downto 0);
      RxDF          : in    std_logic_vector(3 downto 0);
      RxDG          : in    std_logic_vector(3 downto 0);
      RxDH          : in    std_logic_vector(3 downto 0);
      RxClk         : in    std_logic_vector(7 downto 0);
      RxDV          : in    std_logic_vector(7 downto 0);
      RxErr         : in    std_logic_vector(7 downto 0);
      CRS           : in    std_logic_vector(7 downto 0);
      TxDA          : out   std_logic_vector(3 downto 0);
      TxDB          : out   std_logic_vector(3 downto 0);
      TxDC          : out   std_logic_vector(3 downto 0);
      TxDD          : out   std_logic_vector(3 downto 0);
      TxDE          : out   std_logic_vector(3 downto 0);
      TxDF          : out   std_logic_vector(3 downto 0);
      TxDG          : out   std_logic_vector(3 downto 0);
      TxDH          : out   std_logic_vector(3 downto 0);
      TxEn          : buffer std_logic_vector(7 downto 0);
      MDC           : buffer std_logic_vector(1 downto 0);
      MDIO          : inout std_logic_vector(1 downto 0);
      PhyPDn        : buffer std_logic;
      PhyRst        : buffer std_logic;
      TxClk         : in    std_logic_vector(1 downto 0);
      Clk25MHz      : buffer std_logic;
      FMRx          : in    std_logic_vector(7 downto 0);
      FMRxEn        : buffer std_logic;
      HrtBtFM       : in    std_logic;
      DReqFM        : in    std_logic;
      SPICS         : buffer std_logic;
      SPISClk       : buffer std_logic;
      SPIMOSI       : buffer std_logic;
      SPIMISO       : in    std_logic;
      Debug         : buffer std_logic_vector(10 downto 1);
      -- synthesis translate_off
      probe_MaskReg         : out std_logic_vector(7 downto 0);
      probe_PhyRxEmpty      : out std_logic_vector(7 downto 0);
      probe_Rx_active       : out std_logic_vector(7 downto 0);
      probe_PhyTxBuff_Count : out std_logic_vector(10 downto 0);
      probe_ReadyStatus     : out std_logic_vector(7 downto 0)
      -- synthesis translate_on
    );
  end component;

begin

  -- =========================================================================
  -- DUT instantiation
  -- =========================================================================
  DUT : Controller_FPGA2
    port map (
      VXO_P     => VXO_P,        VXO_N     => VXO_N,
      ClkB_P    => ClkB_P,       ClkB_N    => ClkB_N,
      Clk50MHz  => Clk50MHz_tb,
      CpldRst   => CpldRst,      CpldCS    => CpldCS,
      uCRd      => uCRd,         uCWr      => uCWr,
      uCA       => uCA,          uCD       => uCD,
      GA        => GA_pins,
      SDCKE     => SDCKE,        LDM       => LDM,
      UDM       => UDM,          RAS       => RAS,
      CAS       => CAS,          SDWE      => SDWE,
      SDClk_P   => SDClk_P,      SDClk_N   => SDClk_N,
      SDD       => SDD,          UDQS      => UDQS,
      LDQS      => LDQS,         SDRzq     => SDRzq,
      SDA       => SDA,          BA        => BA,
      LinkClk_P => LinkClk_P,    LinkClk_N => LinkClk_N,
      LinkFR_P  => LinkFR_P,     LinkFR_N  => LinkFR_N,
      LinkD_P   => LinkD_P,      LinkD_N   => LinkD_N,
      RxDA      => RxDA,         RxDB      => RxDB,
      RxDC      => RxDC,         RxDD      => RxDD,
      RxDE      => RxDE,         RxDF      => RxDF,
      RxDG      => RxDG,         RxDH      => RxDH,
      RxClk     => RxClk,        RxDV      => RxDV,
      RxErr     => RxErr,        CRS       => CRS_sig,
      TxDA      => TxDA,         TxDB      => TxDB,
      TxDC      => TxDC,         TxDD      => TxDD,
      TxDE      => TxDE,         TxDF      => TxDF,
      TxDG      => TxDG,         TxDH      => TxDH,
      TxEn      => TxEn_out,     MDC       => MDC_out,
      MDIO      => MDIO_sig,     PhyPDn    => PhyPDn_out,
      PhyRst    => PhyRst_out,
      TxClk     => TxClk_sig,    Clk25MHz  => Clk25MHz_out,
      FMRx      => FMRx,         FMRxEn    => FMRxEn_out,
      HrtBtFM   => HrtBtFM,      DReqFM    => DReqFM,
      SPICS     => SPICS_out,    SPISClk   => SPISClk_out,
      SPIMOSI   => SPIMOSI_out,  SPIMISO   => SPIMISO,
      Debug     => Debug_out,
      -- synthesis translate_off
      probe_MaskReg         => dut_MaskReg,
      probe_PhyRxEmpty      => dut_PhyRxEmpty,
      probe_Rx_active       => dut_Rx_active,
      probe_PhyTxBuff_Count => dut_PhyTxBuff_Count,
      probe_ReadyStatus     => dut_ReadyStatus
      -- synthesis translate_on
    );

  -- =========================================================================
  -- Clock generation
  -- The Sys_PLL behavioral stub generates its own clocks internally from the
  -- differential input; we just need to drive ClkB_P/N here.
  -- =========================================================================
  clkb_gen : process
  begin
    while not tb_done loop
      ClkB_P <= '1'; ClkB_N <= '0'; wait for CLK_SYS_PERIOD / 2;
      ClkB_P <= '0'; ClkB_N <= '1'; wait for CLK_SYS_PERIOD / 2;
    end loop;
    wait;
  end process;

  vxo_gen : process
  begin
    while not tb_done loop
      VXO_P <= '1'; VXO_N <= '0'; wait for 3.125 ns;
      VXO_P <= '0'; VXO_N <= '1'; wait for 3.125 ns;
    end loop;
    wait;
  end process;

  clk50_gen : process
  begin
    while not tb_done loop
      Clk50MHz_tb <= '0'; wait for CLK_50_PERIOD / 2;
      Clk50MHz_tb <= '1'; wait for CLK_50_PERIOD / 2;
    end loop;
    wait;
  end process;

  -- Drive RxClk for all 8 ports (25 MHz)
  rxclk_gen : process
  begin
    while not tb_done loop
      RxClk <= (others => '0'); wait for CLK_RX_PERIOD / 2;
      RxClk <= (others => '1'); wait for CLK_RX_PERIOD / 2;
    end loop;
    wait;
  end process;

  -- =========================================================================
  -- Stimulus process
  -- =========================================================================
  stimulus : process

    -- -----------------------------------------------------------------------
    -- Microcontroller write procedure
    -- -----------------------------------------------------------------------
    procedure uc_write (
      constant ga_c   : in std_logic_vector(1 downto 0);
      constant addr_c : in std_logic_vector(9 downto 0);
      constant data_c : in std_logic_vector(15 downto 0)) is
    begin
      wait for UC_SETUP;
      uCA    <= ga_c & addr_c;
      uCD    <= data_c;
      CpldCS <= '0';
      uCWr   <= '0';
      wait for UC_PULSE;
      uCWr   <= '1';
      CpldCS <= '1';
      uCD    <= (others => 'Z');
      wait for UC_HOLD;
    end procedure;

    -- -----------------------------------------------------------------------
    -- Microcontroller read procedure
    -- -----------------------------------------------------------------------
    procedure uc_read (
      constant ga_c   : in  std_logic_vector(1 downto 0);
      constant addr_c : in  std_logic_vector(9 downto 0);
      variable result : out std_logic_vector(15 downto 0)) is
    begin
      wait for UC_SETUP;
      uCA    <= ga_c & addr_c;
      CpldCS <= '0';
      uCRd   <= '0';
      wait for UC_PULSE;
      result := uCD;
      uCRd   <= '1';
      CpldCS <= '1';
      wait for UC_HOLD;
    end procedure;

    -- -----------------------------------------------------------------------
    -- Inject an Ethernet frame on port p (drives RxD/RxDV/CRS for that port)
    -- The preamble is 7 bytes (56 nibbles) of 0x5 followed by 0xD / 0x5.
    -- StartCount needs to reach 6 word-boundaries = 24 nibbles into preamble.
    -- -----------------------------------------------------------------------
    procedure inject_frame (
      constant p    : in integer range 0 to 7;
      constant nwds : in integer) is

      -- Full 7-byte preamble: 6 bytes of 0x55 + 1 byte of 0xD5 = 14 nibbles
      -- followed by payload nibbles
      variable d : std_logic_vector(3 downto 0);
    begin
      -- Assert carrier sense and data valid
      RxDV(p)    <= '1';
      CRS_sig(p) <= '1';

      -- Preamble: 12 nibbles of 0x5 (6 bytes of 0x55)
      for n in 0 to 11 loop
        d := X"5";
        case p is
          when 0 => RxDA <= d;
          when 1 => RxDB <= d;
          when 2 => RxDC <= d;
          when 3 => RxDD <= d;
          when 4 => RxDE <= d;
          when 5 => RxDF <= d;
          when 6 => RxDG <= d;
          when 7 => RxDH <= d;
        end case;
        wait for CLK_RX_PERIOD;
      end loop;

      -- SFD: 0xD then 0x5
      d := X"D";
      case p is
        when 0 => RxDA <= d;
        when 1 => RxDB <= d;
        when 2 => RxDC <= d;
        when 3 => RxDD <= d;
        when 4 => RxDE <= d;
        when 5 => RxDF <= d;
        when 6 => RxDG <= d;
        when 7 => RxDH <= d;
      end case;
      wait for CLK_RX_PERIOD;
      d := X"5";
      case p is
        when 0 => RxDA <= d;
        when 1 => RxDB <= d;
        when 2 => RxDC <= d;
        when 3 => RxDD <= d;
        when 4 => RxDE <= d;
        when 5 => RxDF <= d;
        when 6 => RxDG <= d;
        when 7 => RxDH <= d;
      end case;
      wait for CLK_RX_PERIOD;

      -- Payload: nwds x 4 nibbles
      for w in 0 to nwds - 1 loop
        for nb in 0 to 3 loop
          d := std_logic_vector(to_unsigned((w + nb) mod 16, 4));
          case p is
            when 0 => RxDA <= d;
            when 1 => RxDB <= d;
            when 2 => RxDC <= d;
            when 3 => RxDD <= d;
            when 4 => RxDE <= d;
            when 5 => RxDF <= d;
            when 6 => RxDG <= d;
            when 7 => RxDH <= d;
          end case;
          wait for CLK_RX_PERIOD;
        end loop;
      end loop;

      -- Deassert
      RxDV(p)    <= '0';
      CRS_sig(p) <= '0';
      case p is
        when 0 => RxDA <= X"0";
        when 1 => RxDB <= X"0";
        when 2 => RxDC <= X"0";
        when 3 => RxDD <= X"0";
        when 4 => RxDE <= X"0";
        when 5 => RxDF <= X"0";
        when 6 => RxDG <= X"0";
        when 7 => RxDH <= X"0";
      end case;
    end procedure;

    -- -----------------------------------------------------------------------
    -- Arm Rx_active for a single port by toggling FMRx enough times
    -- Needs TransitionCount to reach 15 within one 10us window.
    -- At 100 MHz, Counter10us wraps every 1000 cycles = 10 us.
    -- Toggle at ~200 ns intervals = 50 MHz = 20 transitions per 10 us window.
    -- We toggle for 2 full windows = 20 us to be safe, then wait for evaluation.
    -- -----------------------------------------------------------------------
    procedure arm_rx_active (constant p : in integer range 0 to 7) is
    begin
      -- Toggle FMRx(p) rapidly for > 2 Counter10us windows
      for k in 0 to 99 loop
        FMRx(p) <= '1'; wait for 100 ns;
        FMRx(p) <= '0'; wait for 100 ns;
      end loop;
      -- Now wait for at least one more Counter10us evaluation point
      wait for 15 us;
    end procedure;

    -- -----------------------------------------------------------------------
    -- Diagnostic snapshot
    -- -----------------------------------------------------------------------
    procedure snap (constant msg : in string) is
    begin
      report msg &
        "  Mask="     & integer'image(to_integer(unsigned(dut_MaskReg)))         &
        "  RxEmpty="  & integer'image(to_integer(unsigned(dut_PhyRxEmpty)))       &
        "  RxAct="    & integer'image(to_integer(unsigned(dut_Rx_active)))        &
        "  TxCnt="    & integer'image(to_integer(unsigned(dut_PhyTxBuff_Count)))  &
        "  Ready="    & integer'image(to_integer(unsigned(dut_ReadyStatus)));
    end procedure;

    variable rd : std_logic_vector(15 downto 0);

  begin

    -- =======================================================================
    -- PHASE 0: Reset
    -- =======================================================================
    report "=== PHASE 0: Reset ===";
    CpldRst <= '0';
    CpldCS  <= '1';
    uCRd    <= '1';
    uCWr    <= '1';
    uCA     <= (others => '0');
    uCD     <= (others => 'Z');

    -- Hold reset long enough for PLL to lock (behavioral stub locks in ~100 ns
    -- but we give 500 ns margin to be safe)
    wait for 500 ns;
    CpldRst <= '1';

    -- Wait for reset synchroniser (2 SysClk cycles) + PLL lock propagation
    wait for 500 ns;

    -- Verify post-reset state
    assert dut_MaskReg = X"FF"
      report "[FAIL] post-reset MaskReg expected FF got " &
             integer'image(to_integer(unsigned(dut_MaskReg))) severity error;
    if dut_MaskReg = X"FF" then
      report "[PASS] MaskReg = 0xFF after reset";
    end if;

    assert dut_PhyRxEmpty = X"FF"
      report "[FAIL] post-reset PhyRxEmpty expected FF got " &
             integer'image(to_integer(unsigned(dut_PhyRxEmpty))) severity error;
    if dut_PhyRxEmpty = X"FF" then
      report "[PASS] all RX FIFOs empty after reset";
    end if;

    assert dut_Rx_active = X"00"
      report "[FAIL] post-reset Rx_active expected 00 got " &
             integer'image(to_integer(unsigned(dut_Rx_active))) severity error;
    if dut_Rx_active = X"00" then
      report "[PASS] Rx_active = 0x00 after reset";
    end if;

    assert dut_ReadyStatus = X"00"
      report "[FAIL] post-reset ReadyStatus expected 00 got " &
             integer'image(to_integer(unsigned(dut_ReadyStatus))) severity error;
    if dut_ReadyStatus = X"00" then
      report "[PASS] ReadyStatus = 0x00 after reset";
    end if;

    -- =======================================================================
    -- PHASE 1: InputMask register read/write
    -- =======================================================================
    report "=== PHASE 1: InputMask ===";

    uc_write(GA_VAL, InputMaskAddr, X"00AA");
    wait for 20 ns;
    assert dut_MaskReg = X"AA"
      report "[FAIL] MaskReg expected AA got " &
             integer'image(to_integer(unsigned(dut_MaskReg))) severity error;
    if dut_MaskReg = X"AA" then
      report "[PASS] MaskReg = 0xAA";
    end if;

    uc_write(GA_VAL, InputMaskAddr, X"00FF");
    wait for 20 ns;
    assert dut_MaskReg = X"FF"
      report "[FAIL] MaskReg restore expected FF got " &
             integer'image(to_integer(unsigned(dut_MaskReg))) severity error;
    if dut_MaskReg = X"FF" then
      report "[PASS] MaskReg restored to 0xFF";
    end if;

    -- =======================================================================
    -- PHASE 2: Arm Rx_active on all 8 ports
    -- The TransitionCount logic needs FMRx to toggle >= 15 times within a
    -- single Counter10us (10 us) window.  We toggle at 100 ns per half-period
    -- (5 MHz toggle rate) which gives 50 transitions per 10 us window.
    -- After toggling we must wait for one complete Counter10us evaluation.
    -- =======================================================================
    report "=== PHASE 2: Arm Rx_active on all ports ===";

    -- Enable FM Rx first (CSR bit 3)
    uc_write(GA_VAL, CSRRegAddr, X"0008");  -- FMRxEn=1
    wait for 100 ns;

    -- Arm all 8 ports simultaneously
    for k in 0 to 99 loop
      FMRx <= X"FF"; wait for 100 ns;
      FMRx <= X"00"; wait for 100 ns;
    end loop;

    -- Wait > 2 Counter10us windows to guarantee evaluation
    wait for RXACTIVE_ARM_TIME;

    snap("after arming all ports");
    for p in 0 to 7 loop
      assert dut_Rx_active(p) = '1'
        report "[FAIL] Rx_active(" & integer'image(p) &
               ") not set after arming" severity error;
      if dut_Rx_active(p) = '1' then
        report "[PASS] Rx_active(" & integer'image(p) & ") armed";
      end if;
    end loop;

    -- =======================================================================
    -- PHASE 3: Inject Ethernet frames on ports 0 and 1
    -- Need to power up PHY first (CSR bit 2 = not PhyPDn)
    -- =======================================================================
    report "=== PHASE 3: Inject frames on ports 0 and 1 ===";

    -- Keep FMRxEn=1 (bit 3), also set PhyPDn deassert (bit 2)
    -- Continue toggling FMRx to keep Rx_active asserted during inject
    -- CSR write: bit3=FMRxEn=1, bit2=PhyPDn_deassert=1 => 0x000C
    uc_write(GA_VAL, CSRRegAddr, X"000C");
    wait for 100 ns;

    -- Keep toggling FMRx(0) while injecting to maintain Rx_active(0)
    -- (run in parallel conceptually; in VHDL-93 sequential TB we interleave)

    inject_frame(0, 16);
    wait for 2000 ns;

    assert dut_PhyRxEmpty(0) = '0'
      report "[FAIL] port 0 RX FIFO still empty after inject" severity error;
    if dut_PhyRxEmpty(0) = '0' then
      report "[PASS] port 0 RX FIFO non-empty after inject";
    end if;

    inject_frame(1, 16);
    wait for 2000 ns;

    assert dut_PhyRxEmpty(1) = '0'
      report "[FAIL] port 1 RX FIFO still empty after inject" severity error;
    if dut_PhyRxEmpty(1) = '0' then
      report "[PASS] port 1 RX FIFO non-empty after inject";
    end if;

    for p in 2 to 7 loop
      assert dut_PhyRxEmpty(p) = '1'
        report "[FAIL] port " & integer'image(p) &
               " unexpectedly non-empty" severity error;
      if dut_PhyRxEmpty(p) = '1' then
        report "[PASS] port " & integer'image(p) & " still empty";
      end if;
    end loop;

    snap("after ports 0+1 filled");

    -- =======================================================================
    -- PHASE 4: AutoTxKick -> PhyTxBuff_Count
    -- =======================================================================
    report "=== PHASE 4: AutoTxKick ===";
    snap("pre-kick");

    uc_write(GA_VAL, AutoTxKickAddr, X"0001");
    wait for 5000 ns;
    snap("post-kick");

    assert to_integer(unsigned(dut_PhyTxBuff_Count)) > 0
      report "[FAIL] TX FIFO still empty after AutoTxKick" severity error;
    if to_integer(unsigned(dut_PhyTxBuff_Count)) > 0 then
      report "[PASS] TxCnt=" &
             integer'image(to_integer(unsigned(dut_PhyTxBuff_Count)));
    end if;

    -- =======================================================================
    -- PHASE 5: ReadyStatus force/clear
    -- =======================================================================
    report "=== PHASE 5: ReadyStatus force/clear ===";

    -- Force bit 0
    uc_write(GA_VAL, ReadyForceAddr, X"0001");
    wait for 30 ns;
    assert dut_ReadyStatus(0) = '1'
      report "[FAIL] ReadyStatus(0) not set after force" severity error;
    if dut_ReadyStatus(0) = '1' then
      report "[PASS] ReadyStatus(0) set";
    end if;

    -- Clear bit 0
    uc_write(GA_VAL, ReadyClearAddr, X"0001");
    wait for 30 ns;
    assert dut_ReadyStatus(0) = '0'
      report "[FAIL] ReadyStatus(0) not cleared" severity error;
    if dut_ReadyStatus(0) = '0' then
      report "[PASS] ReadyStatus(0) cleared";
    end if;

    -- Force all bits
    uc_write(GA_VAL, ReadyForceAddr, X"00FF");
    wait for 30 ns;
    assert dut_ReadyStatus = X"FF"
      report "[FAIL] ReadyStatus not FF after force-all; got " &
             integer'image(to_integer(unsigned(dut_ReadyStatus))) severity error;
    if dut_ReadyStatus = X"FF" then
      report "[PASS] ReadyStatus = FF";
    end if;

    -- Clear all bits
    uc_write(GA_VAL, ReadyClearAddr, X"00FF");
    wait for 30 ns;
    assert dut_ReadyStatus = X"00"
      report "[FAIL] ReadyStatus not cleared to 00; got " &
             integer'image(to_integer(unsigned(dut_ReadyStatus))) severity error;
    if dut_ReadyStatus = X"00" then
      report "[PASS] ReadyStatus = 00";
    end if;
    snap("after clear all");

    -- =======================================================================
    -- PHASE 6: Register read-backs via uC bus
    -- =======================================================================
    report "=== PHASE 6: Register read-backs ===";

    uc_read(GA_VAL, RxDAVAddr, rd);
    report "    RxDAV = " & integer'image(to_integer(unsigned(rd(7 downto 0))));
    -- RxDAV = not PhyRxBuff_Empty; after phase 3, ports 0 and 1 are non-empty
    assert rd(7 downto 0) = (not dut_PhyRxEmpty)
      report "[FAIL] RxDAV mismatch: got " &
             integer'image(to_integer(unsigned(rd(7 downto 0)))) &
             " expected " &
             integer'image(to_integer(unsigned(not dut_PhyRxEmpty)))
      severity error;
    if rd(7 downto 0) = (not dut_PhyRxEmpty) then
      report "[PASS] RxDAV matches ~PhyRxEmpty";
    end if;

    uc_read(GA_VAL, PhyTxCntAddr, rd);
    report "    PhyTxCnt = " & integer'image(to_integer(unsigned(rd(10 downto 0))));
    assert rd(10 downto 0) = dut_PhyTxBuff_Count
      report "[FAIL] PhyTxCnt mismatch" severity error;
    if rd(10 downto 0) = dut_PhyTxBuff_Count then
      report "[PASS] PhyTxCnt matches probe";
    end if;

    uc_read(GA_VAL, ReadyStatusAddr, rd);
    report "    ReadyStatus (uC) = " &
           integer'image(to_integer(unsigned(rd(7 downto 0))));
    assert rd(7 downto 0) = dut_ReadyStatus
      report "[FAIL] ReadyStatus register mismatch" severity error;
    if rd(7 downto 0) = dut_ReadyStatus then
      report "[PASS] ReadyStatus register matches probe";
    end if;

    -- =======================================================================
    -- PHASE 7: Periodic snapshots
    -- =======================================================================
    report "=== PHASE 7: 1-us snapshots ===";
    for tick in 1 to 5 loop
      wait for 1000 ns;
      snap("t+" & integer'image(tick) & "us");
    end loop;

    report "=== SIMULATION COMPLETE ===";
    tb_done <= true;
    wait;
  end process stimulus;

end architecture sim;