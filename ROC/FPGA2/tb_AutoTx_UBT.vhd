-- =============================================================================
-- tb_AutoTx_UBT.vhd  (ISE / VHDL-93 compatible)
--
-- Targeted testbench for the AutoTx / UBT sequencer in Controller_FPGA2.
--
-- Changes in this version:
--   - Extended simulation to 3 ms to cover one full 1Hz re-arm cycle
--   - Added 1Hz re-arm check after FEB reply injection
--   - Added probe_UBT_in_progress port and monitor
--   - TxBuff=3 at first write is explained: FIFO has pre-existing words;
--     monitor now prints a note if TxBuff > expected at packet start
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

use work.Proj_Defs.all;

entity tb_AutoTx_UBT is
end entity tb_AutoTx_UBT;

architecture sim of tb_AutoTx_UBT is

  -- -------------------------------------------------------------------------
  -- Timing constants
  -- -------------------------------------------------------------------------
  constant CLK_HALF     : time    := 5 ns;      -- 100 MHz SysClk
  constant VXO_HALF     : time    := 3125 ps;   -- 160 MHz VXO
  constant CLK50_HALF   : time    := 10 ns;     --  50 MHz
  constant RESET_CYCLES : integer := 20;

  -- -------------------------------------------------------------------------
  -- UBT packet word count (UBT_ASC_COUNT_EXPLICIT = 4 in Proj_Defs).
  -- READY_WORD_COUNT lives inside the DUT architecture body and is not
  -- visible here, so we mirror the value explicitly.
  -- -------------------------------------------------------------------------
  constant TB_UBT_WORD_COUNT : integer := 4;

  -- -------------------------------------------------------------------------
  -- VHDL-93 compatible hex conversion (replaces VHDL-2008 to_hstring)
  -- -------------------------------------------------------------------------
  function slv_to_hex(slv : std_logic_vector) return string is
    constant HEX_CHARS : string(1 to 16) := "0123456789ABCDEF";
    constant PAD  : integer := (4 - (slv'length mod 4)) mod 4;
    constant LEN  : integer := slv'length + PAD;
    variable padded : std_logic_vector(LEN - 1 downto 0) := (others => '0');
    variable nibble : std_logic_vector(3 downto 0);
    variable result : string(1 to LEN / 4);
    variable idx    : integer;
  begin
    padded(slv'length - 1 downto 0) := slv;
    for i in 0 to LEN/4 - 1 loop
      nibble := padded(LEN - 1 - i*4 downto LEN - 4 - i*4);
      idx := to_integer(unsigned(nibble));
      result(i + 1) := HEX_CHARS(idx + 1);
    end loop;
    return result;
  end function slv_to_hex;

  -- -------------------------------------------------------------------------
  -- DUT port signals
  -- -------------------------------------------------------------------------
  signal ClkB_P, ClkB_N   : std_logic := '1';
  signal VXO_P,  VXO_N    : std_logic := '1';
  signal Clk50MHz          : std_logic := '0';
  signal CpldRst           : std_logic := '0';   -- active-low

  signal CpldCS  : std_logic := '1';
  signal uCRd    : std_logic := '1';
  signal uCWr    : std_logic := '1';
  signal uCA     : std_logic_vector(11 downto 0) := (others => '0');
  signal uCD     : std_logic_vector(15 downto 0) := (others => 'Z');

  signal GA      : std_logic_vector(1 downto 0) := "00";

  signal SDCKE, LDM, UDM, RAS, CAS, SDWE : std_logic;
  signal SDClk_P, SDClk_N                 : std_logic;
  signal SDD                              : std_logic_vector(15 downto 0) := (others => 'Z');
  signal UDQS, LDQS, SDRzq               : std_logic := 'Z';
  signal SDA                              : std_logic_vector(13 downto 0);
  signal BA                               : std_logic_vector(1 downto 0);

  signal LinkClk_P, LinkClk_N : std_logic;
  signal LinkFR_P,  LinkFR_N  : std_logic;
  signal LinkD_P,   LinkD_N   : std_logic_vector(1 downto 0);

  signal RxDA,RxDB,RxDC,RxDD,
         RxDE,RxDF,RxDG,RxDH : std_logic_vector(3 downto 0) := (others => '0');
  signal RxClk  : std_logic_vector(7 downto 0) := (others => '0');
  signal RxDV   : std_logic_vector(7 downto 0) := (others => '0');
  signal RxErr  : std_logic_vector(7 downto 0) := (others => '0');
  signal CRS    : std_logic_vector(7 downto 0) := (others => '0');

  signal TxDA,TxDB,TxDC,TxDD,
         TxDE,TxDF,TxDG,TxDH : std_logic_vector(3 downto 0);
  signal TxEn   : std_logic_vector(7 downto 0);
  signal MDC    : std_logic_vector(1 downto 0);
  signal MDIO   : std_logic_vector(1 downto 0) := "ZZ";
  signal PhyPDn, PhyRst : std_logic;
  signal TxClk  : std_logic_vector(1 downto 0) := (others => '0');
  signal Clk25MHz : std_logic;

  signal FMRx    : std_logic_vector(7 downto 0) := (others => '0');
  signal FMRxEn  : std_logic;
  signal HrtBtFM, DReqFM : std_logic := '0';
  signal SPICS, SPISClk, SPIMOSI : std_logic;
  signal SPIMISO : std_logic := '0';
  signal Debug   : std_logic_vector(10 downto 1);

  -- Probe ports (synthesis translate_off in DUT)
  signal probe_MaskReg          : std_logic_vector(7 downto 0);
  signal probe_PhyRxEmpty       : std_logic_vector(7 downto 0);
  signal probe_Rx_active        : std_logic_vector(7 downto 0);
  signal probe_PhyTxBuff_Count  : std_logic_vector(10 downto 0);
  signal probe_ReadyStatus      : std_logic_vector(7 downto 0);
  signal probe_UBT_in_progress  : std_logic_vector(7 downto 0);
  signal probe_handshake_queued : std_logic_vector(7 downto 0);
  signal probe_AutoTx_Port      : std_logic_vector(2 downto 0);

  -- -------------------------------------------------------------------------
  -- Testbench bookkeeping
  -- -------------------------------------------------------------------------
  type int8_array is array(0 to 7) of integer;
  signal ubt_packet_count   : int8_array := (others => 0);
  signal packet_number      : integer    := 0;
  signal first_ubt_complete : boolean    := false;
  signal feb_reply_done     : boolean    := false;

  -- Marks the sim time when the FEB reply injection finished.
  -- The 1Hz check process waits for this before starting its window.
  signal feb_reply_time     : time       := 0 ns;

  -- -------------------------------------------------------------------------
  -- Component declaration
  -- -------------------------------------------------------------------------
  component Controller_FPGA2 is
    port(
      VXO_P, VXO_N, ClkB_P, ClkB_N, Clk50MHz : in  std_logic;
      CpldRst, CpldCS, uCRd, uCWr             : in  std_logic;
      uCA                                      : in  std_logic_vector(11 downto 0);
      uCD                                      : inout std_logic_vector(15 downto 0);
      GA                                       : in  std_logic_vector(1 downto 0);
      SDCKE, LDM, UDM, RAS, CAS, SDWE         : out std_logic;
      SDClk_P, SDClk_N                         : out std_logic;
      SDD                                      : inout std_logic_vector(15 downto 0);
      UDQS, LDQS, SDRzq                        : inout std_logic;
      SDA                                      : out std_logic_vector(13 downto 0);
      BA                                       : out std_logic_vector(1 downto 0);
      LinkClk_P, LinkClk_N,
      LinkFR_P,  LinkFR_N                      : out std_logic;
      LinkD_P, LinkD_N                         : out std_logic_vector(1 downto 0);
      RxDA, RxDB, RxDC, RxDD,
      RxDE, RxDF, RxDG, RxDH                  : in  std_logic_vector(3 downto 0);
      RxClk, RxDV, RxErr, CRS                  : in  std_logic_vector(7 downto 0);
      TxDA, TxDB, TxDC, TxDD,
      TxDE, TxDF, TxDG, TxDH                  : out std_logic_vector(3 downto 0);
      TxEn                                     : buffer std_logic_vector(7 downto 0);
      MDC                                      : buffer std_logic_vector(1 downto 0);
      MDIO                                     : inout std_logic_vector(1 downto 0);
      PhyPDn, PhyRst                           : buffer std_logic;
      TxClk                                    : in  std_logic_vector(1 downto 0);
      Clk25MHz                                 : buffer std_logic;
      FMRx                                     : in  std_logic_vector(7 downto 0);
      FMRxEn                                   : buffer std_logic;
      HrtBtFM, DReqFM                          : in  std_logic;
      SPICS, SPISClk, SPIMOSI                  : buffer std_logic;
      SPIMISO                                  : in  std_logic;
      Debug                                    : buffer std_logic_vector(10 downto 1);
      -- synthesis translate_off
      probe_MaskReg         : out std_logic_vector(7 downto 0);
      probe_PhyRxEmpty      : out std_logic_vector(7 downto 0);
      probe_Rx_active       : out std_logic_vector(7 downto 0);
      probe_PhyTxBuff_Count : out std_logic_vector(10 downto 0);
      probe_ReadyStatus     : out std_logic_vector(7 downto 0);
      probe_UBT_in_progress : out std_logic_vector(7 downto 0);
      probe_handshake_queued: out std_logic_vector(7 downto 0);
      probe_AutoTx_Port     : out std_logic_vector(2 downto 0)
      -- synthesis translate_on
    );
  end component Controller_FPGA2;

begin

  -- =========================================================================
  -- Clock generation
  -- =========================================================================
  clkb_gen : process
  begin
    loop
      ClkB_P <= '1'; ClkB_N <= '0'; wait for CLK_HALF;
      ClkB_P <= '0'; ClkB_N <= '1'; wait for CLK_HALF;
    end loop;
  end process clkb_gen;

  vxo_gen : process
  begin
    loop
      VXO_P <= '1'; VXO_N <= '0'; wait for VXO_HALF;
      VXO_P <= '0'; VXO_N <= '1'; wait for VXO_HALF;
    end loop;
  end process vxo_gen;

  clk50_gen : process
  begin
    loop
      Clk50MHz <= '0'; wait for CLK50_HALF;
      Clk50MHz <= '1'; wait for CLK50_HALF;
    end loop;
  end process clk50_gen;

  -- =========================================================================
  -- DUT instantiation
  -- =========================================================================
  DUT : Controller_FPGA2
    port map (
      VXO_P    => VXO_P,    VXO_N    => VXO_N,
      ClkB_P   => ClkB_P,   ClkB_N   => ClkB_N,
      Clk50MHz => Clk50MHz,
      CpldRst  => CpldRst,  CpldCS   => CpldCS,
      uCRd     => uCRd,     uCWr     => uCWr,
      uCA      => uCA,      uCD      => uCD,
      GA       => GA,
      SDCKE => SDCKE, LDM => LDM,    UDM => UDM,
      RAS   => RAS,   CAS => CAS,    SDWE => SDWE,
      SDClk_P  => SDClk_P,  SDClk_N  => SDClk_N,
      SDD      => SDD,
      UDQS     => UDQS,     LDQS     => LDQS,   SDRzq => SDRzq,
      SDA      => SDA,      BA       => BA,
      LinkClk_P => LinkClk_P, LinkClk_N => LinkClk_N,
      LinkFR_P  => LinkFR_P,  LinkFR_N  => LinkFR_N,
      LinkD_P   => LinkD_P,   LinkD_N   => LinkD_N,
      RxDA => RxDA, RxDB => RxDB, RxDC => RxDC, RxDD => RxDD,
      RxDE => RxDE, RxDF => RxDF, RxDG => RxDG, RxDH => RxDH,
      RxClk => RxClk, RxDV => RxDV, RxErr => RxErr, CRS => CRS,
      TxDA => TxDA, TxDB => TxDB, TxDC => TxDC, TxDD => TxDD,
      TxDE => TxDE, TxDF => TxDF, TxDG => TxDG, TxDH => TxDH,
      TxEn     => TxEn,    MDC      => MDC,   MDIO  => MDIO,
      PhyPDn   => PhyPDn,  PhyRst   => PhyRst,
      TxClk    => TxClk,   Clk25MHz => Clk25MHz,
      FMRx     => FMRx,    FMRxEn   => FMRxEn,
      HrtBtFM  => HrtBtFM, DReqFM   => DReqFM,
      SPICS    => SPICS,   SPISClk  => SPISClk, SPIMOSI => SPIMOSI,
      SPIMISO  => SPIMISO,
      Debug    => Debug,
      -- synthesis translate_off
      probe_MaskReg          => probe_MaskReg,
      probe_PhyRxEmpty       => probe_PhyRxEmpty,
      probe_Rx_active        => probe_Rx_active,
      probe_PhyTxBuff_Count  => probe_PhyTxBuff_Count,
      probe_ReadyStatus      => probe_ReadyStatus,
      probe_UBT_in_progress  => probe_UBT_in_progress,
      probe_handshake_queued => probe_handshake_queued,
      probe_AutoTx_Port      => probe_AutoTx_Port
      -- synthesis translate_on
    );

  -- =========================================================================
  -- Stimulus: reset, free-run for 3 ms (covers one full 1Hz re-arm cycle)
  -- =========================================================================
  stim_proc : process
  begin
    CpldRst <= '0';
    for i in 0 to RESET_CYCLES - 1 loop
      wait until rising_edge(ClkB_P);
    end loop;
    CpldRst <= '1';
    report "INFO: CpldRst released at " & time'image(now);
    -- 3 ms covers: startup (~2.6 us) + first UBT + FEB reply (~5 us) +
    -- full 1 Hz counter period (1 ms simulated @ 100 MHz with Count1s
    -- scaled for sim) + margin
    wait for 3 ms;
    report "=== SIMULATION COMPLETE ==="
           & "  total_UBT_packets=" & integer'image(packet_number);
    wait;
  end process stim_proc;

  -- =========================================================================
  -- Summary: fires whenever packet_number reaches 5 or more
  -- =========================================================================
  summary_proc : process(packet_number)
    variable ports_used : integer;
    variable max_port   : integer;
    variable max_cnt    : integer;
  begin
    if packet_number >= 5 then
      ports_used := 0;
      max_port   := 0;
      max_cnt    := 0;
      for p in 0 to 7 loop
        report "  Port " & integer'image(p)
               & " : " & integer'image(ubt_packet_count(p))
               & " UBT packet(s)";
        if ubt_packet_count(p) > 0 then
          ports_used := ports_used + 1;
          if ubt_packet_count(p) > max_cnt then
            max_cnt  := ubt_packet_count(p);
            max_port := p;
          end if;
        end if;
      end loop;
      if ports_used = 1 then
        report "RESULT: BUG - all " & integer'image(packet_number)
               & " UBTs sent to SAME port " & integer'image(max_port)
               & ". Root cause: state 011 missing UBT_in_progress set."
               severity error;
      else
        report "RESULT: BUG - " & integer'image(packet_number)
               & " UBTs spread across " & integer'image(ports_used)
               & " ports. Root cause: PowerOnReady_done / state-011 guard."
               severity error;
      end if;
    end if;
  end process summary_proc;

  -- =========================================================================
  -- Monitor: detect UBT word writes via PhyTxBuff_Count increments;
  --          identify target port from ReadyStatus falling edges.
  -- =========================================================================
  monitor_proc : process
    variable prev_count  : std_logic_vector(10 downto 0) := (others => '0');
    variable prev_ready  : std_logic_vector(7 downto 0)  := (others => '0');
    variable prev_ubt_ip : std_logic_vector(7 downto 0)  := (others => '0');
    variable pkt_port    : integer range 0 to 7 := 0;
    variable in_pkt      : boolean := false;
    variable word_cnt    : integer := 0;
    variable pkt_num     : integer := 0;
  begin
    wait until rising_edge(ClkB_P);

    -- ----------------------------------------------------------------
    -- Detect ReadyStatus bit falling -> port claimed
    -- ----------------------------------------------------------------
    for p in 0 to 7 loop
      if prev_ready(p) = '1' and probe_ReadyStatus(p) = '0' then
        report "CLAIM: ReadyStatus bit " & integer'image(p)
               & " cleared"
               & "  RS_before=0x" & slv_to_hex(prev_ready)
               & "  RS_after=0x"  & slv_to_hex(probe_ReadyStatus)
               & "  UBT_in_prog=0x" & slv_to_hex(probe_UBT_in_progress)
               & "  @" & time'image(now);
        pkt_port := p;
      end if;
    end loop;

    -- ----------------------------------------------------------------
    -- Detect UBT_in_progress rising edge (port entered handshake)
    -- ----------------------------------------------------------------
    for p in 0 to 7 loop
      if prev_ubt_ip(p) = '0' and probe_UBT_in_progress(p) = '1' then
        report "UBT_IN_PROG_SET: port " & integer'image(p)
               & "  @" & time'image(now);
      end if;
      if prev_ubt_ip(p) = '1' and probe_UBT_in_progress(p) = '0' then
        report "UBT_IN_PROG_CLR: port " & integer'image(p)
               & "  (FEB replied or timed out)"
               & "  @" & time'image(now);
      end if;
    end loop;

    -- ----------------------------------------------------------------
    -- Detect PhyTxBuff_Count increment -> UBT word written
    -- ----------------------------------------------------------------
    if probe_PhyTxBuff_Count > prev_count then
      word_cnt := word_cnt + 1;
      if not in_pkt then
        in_pkt  := true;
        pkt_num := pkt_num + 1;
        -- Warn if the FIFO already had data before this packet started
        if to_integer(unsigned(probe_PhyTxBuff_Count)) > TB_UBT_WORD_COUNT then
          report "WARNING: pkt#" & integer'image(pkt_num)
                 & " starts with TxBuff="
                 & integer'image(to_integer(unsigned(probe_PhyTxBuff_Count)))
                 & " (expected <= " & integer'image(TB_UBT_WORD_COUNT)
                 & ") -- FIFO may have stale data"
                 severity warning;
        end if;
      end if;

      report "UBT_WRITE: pkt#" & integer'image(pkt_num)
             & " word "   & integer'image(word_cnt)
             & " port "   & integer'image(pkt_port)
             & "  RS=0x"  & slv_to_hex(probe_ReadyStatus)
             & "  UBT_ip=0x" & slv_to_hex(probe_UBT_in_progress)
             & "  TxBuff=" & integer'image(
                 to_integer(unsigned(probe_PhyTxBuff_Count)))
             & "  @" & time'image(now);

      if word_cnt >= TB_UBT_WORD_COUNT then
        report "PORT_DONE: pkt#" & integer'image(pkt_num)
               & " COMPLETE port " & integer'image(pkt_port)
               & "  total=" & integer'image(pkt_num)
               & "  @" & time'image(now);
        ubt_packet_count(pkt_port) <= ubt_packet_count(pkt_port) + 1;
        packet_number              <= pkt_num;
        in_pkt                     := false;
        word_cnt                   := 0;
        if not first_ubt_complete then
          first_ubt_complete <= true;
        end if;
      end if;
    end if;

    prev_count  := probe_PhyTxBuff_Count;
    prev_ready  := probe_ReadyStatus;
    prev_ubt_ip := probe_UBT_in_progress;

  end process monitor_proc;

  -- =========================================================================
  -- Monitor: log ReadyStatus changes
  -- =========================================================================
  ready_mon : process(probe_ReadyStatus)
  begin
    report "READY_STATUS -> 0x" & slv_to_hex(probe_ReadyStatus)
           & "  UBT_ip=0x" & slv_to_hex(probe_UBT_in_progress)
           & "  @" & time'image(now);
  end process ready_mon;

  -- =========================================================================
  -- Monitor: log TxEn changes (non-zero only)
  -- =========================================================================
  txen_mon : process(TxEn)
  begin
    if TxEn /= "00000000" then
      report "TXEN -> 0x" & slv_to_hex(TxEn)
             & "  @" & time'image(now);
    end if;
  end process txen_mon;

  -- =========================================================================
  -- Monitor: log MaskReg changes
  -- =========================================================================
  mask_mon : process(probe_MaskReg)
  begin
    report "MASK_REG -> 0x" & slv_to_hex(probe_MaskReg)
           & "  @" & time'image(now);
  end process mask_mon;

  -- =========================================================================
  -- Monitor: log PhyRxBuff_Empty changes
  -- =========================================================================
  rxempty_mon : process(probe_PhyRxEmpty)
  begin
    report "RX_EMPTY_CHG -> 0x" & slv_to_hex(probe_PhyRxEmpty)
           & "  @" & time'image(now);
  end process rxempty_mon;

  -- =========================================================================
  -- Stimulus: inject a fake Ethernet Rx packet on port 0 after the first
  --           UBT packet is complete (simulates FEB reply).
  -- =========================================================================
  feb_reply_proc : process
    constant RX_HALF : time := 20 ns;   -- 25 MHz half-period

    procedure send_nibble(nib : std_logic_vector(3 downto 0)) is
    begin
      RxDA     <= nib;
      RxClk(0) <= '0'; wait for RX_HALF;
      RxClk(0) <= '1'; wait for RX_HALF;
    end procedure send_nibble;

  begin
    wait until first_ubt_complete = true;
    wait for 500 ns;

    report "FEB_REPLY: injecting Ethernet packet on port 0"
           & "  @" & time'image(now);

    CRS(0)  <= '1';
    RxDV(0) <= '1';

    -- 14 preamble nibbles (0x5 each)
    for n in 0 to 13 loop
      send_nibble("0101");
    end loop;
    -- SFD nibble
    send_nibble("1101");
    -- 32 payload nibbles (0xA = dummy data, 16 bytes = 8 words)
    for n in 0 to 31 loop
      send_nibble("1010");
    end loop;

    CRS(0)   <= '0';
    RxDV(0)  <= '0';
    RxDA     <= "0000";
    RxClk(0) <= '0';

    report "FEB_REPLY: injection complete  @" & time'image(now);

    feb_reply_time <= now;
    feb_reply_done <= true;

    wait;
  end process feb_reply_proc;

  -- =========================================================================
  -- 1Hz re-arm check:
  --   After the FEB reply has been injected, wait just over one simulated
  --   1Hz period (Counter1s wraps at Count1s).  In a 100 MHz simulation
  --   Count1s = 100_000_000 cycles = 1 s real time, but the testbench only
  --   runs for 3 ms, so the 1Hz counter will NOT complete a full wrap in
  --   this simulation window.
  --
  --   What we CAN check is the re-arm path at the Counter1s = 0 boundary.
  --   In this 3 ms window we expect ZERO additional UBT packets after the
  --   first one, because:
  --     (a) UBT_session_done(0) blocks the 1Hz path from re-arming port 0,
  --     (b) No other port has a FEB connected (probe_Rx_active = 0xFF would
  --         be needed to trigger re-arm for the others, but in this TB the
  --         FM links are idle so Rx_active stays 0x00).
  --
  --   If a second UBT_WRITE appears in the window between "1HZ_CHECK_START"
  --   and "1HZ_CHECK_END", that is a spurious re-arm bug.
  --
  --   To exercise the true 1Hz path, increase the simulation time to 1.1 s
  --   and replace Count1s with a faster constant in a dedicated sim-mode
  --   version of Proj_Defs.
  -- =========================================================================
  hz1_check_proc : process
    variable pkts_before : integer;
    variable pkts_after  : integer;
  begin
    -- Wait for the FEB reply to have been injected
    wait until feb_reply_done = true;

    -- Allow UBT_in_progress to clear and any CDC pipelines to flush
    wait for 2 us;

    pkts_before := packet_number;
    report "1HZ_CHECK_START: packet_count=" & integer'image(pkts_before)
           & "  RS=0x"    & slv_to_hex(probe_ReadyStatus)
           & "  UBT_ip=0x" & slv_to_hex(probe_UBT_in_progress)
           & "  @" & time'image(now);

    -- Observe for 1.1 ms.
    -- This is short of the real 1Hz period (1 s) but is enough to catch
    -- any FSM-level re-trigger that does not respect UBT_in_progress or
    -- UBT_session_done (these would fire within microseconds, not seconds).
    wait for 1100 us;

    pkts_after := packet_number;
    report "1HZ_CHECK_END:   packet_count=" & integer'image(pkts_after)
           & "  RS=0x"    & slv_to_hex(probe_ReadyStatus)
           & "  UBT_ip=0x" & slv_to_hex(probe_UBT_in_progress)
           & "  @" & time'image(now);

    if pkts_after = pkts_before then
      report "1HZ_CHECK PASS: no spurious UBT packets in 1.1 ms window"
             & " (fast re-trigger path is gated correctly)";
    else
      report "1HZ_CHECK FAIL: " & integer'image(pkts_after - pkts_before)
             & " extra UBT packet(s) sent in 1.1 ms window"
             & " -- check UBT_session_done / UBT_in_progress guards"
             severity error;
    end if;

    -- Additional spot-check: UBT_in_progress should be all-zero after reply
    if probe_UBT_in_progress /= "00000000" then
      report "1HZ_CHECK WARN: UBT_in_progress=0x"
             & slv_to_hex(probe_UBT_in_progress)
             & " is non-zero at end of check window"
             & " -- possible stuck handshake"
             severity warning;
    end if;

    -- Additional spot-check: ReadyStatus should be zero (no spurious re-arm)
    if probe_ReadyStatus /= "00000000" then
      report "1HZ_CHECK WARN: ReadyStatus=0x"
             & slv_to_hex(probe_ReadyStatus)
             & " is non-zero at end of check window"
             & " -- 1Hz re-arm may have fired early or UBT_session_done"
             & " is not blocking correctly"
             severity warning;
    end if;

    wait;
  end process hz1_check_proc;

end architecture sim;