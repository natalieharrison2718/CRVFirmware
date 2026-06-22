-- Firmware for Mu2e Controller FPGA 2

-- Sten Hansen Fermilab 10/26/2015

-- FPGA responsible for handling data from eight Ethernet PHY chips,
-- and eight LVDS I/Os
-- Collects data from eight phy chips and sends it to FPGA 1 via an 800 Mbit
-- LVDS Link. A 512Mb LPDDR RAM is available as a data buffer
-- Microcontroller interface

----------------------------- Main Body of design -------------------------

LIBRARY ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

Library UNISIM;
use UNISIM.vcomponents.all;

use work.Proj_Defs.all;


entity Controller_FPGA2 is port(

-- 160 MHz VXO clock, Phy clocks
	VXO_P,VXO_N,ClkB_P,ClkB_N,Clk50MHz,
-- microcontroller strobes
	CpldRst, CpldCS, uCRd, uCWr : in std_logic;
-- microcontroller data, address buses
	uCA : in std_logic_vector(11 downto 0);
	uCD : inout std_logic_vector(15 downto 0);
-- Geographic address pins
	GA : in std_logic_vector(1 downto 0);
-- SDRAM pins
	SDCKE,LDM,UDM,RAS,CAS,SDWE : out std_logic;
	SDClk_P,SDClk_N : out  std_logic;
	SDD : inout std_logic_vector(15 downto 0);
	UDQS,LDQS,SDRzq : inout std_logic;
	SDA : out std_logic_vector(13 downto 0);
	BA : out std_logic_vector(1 downto 0);
-- Link serial transmitter signals
	LinkClk_P,LinkClk_N,LinkFR_P,LinkFR_N : out  std_logic;
	LinkD_P,LinkD_N : out std_logic_vector(1 downto 0);
-- Ethernet PHY Signals
	RxDA,RxDB,RxDC,RxDD,RxDE,RxDF,RxDG,RxDH : in std_logic_vector(3 downto 0);
	RxClk,RxDV,RxErr,CRS : in std_logic_vector(7 downto 0);
	TxDA,TxDB,TxDC,TxDD,TxDE,TxDF,TxDG,TxDH : out std_logic_vector(3 downto 0);
	TxEn : buffer std_logic_vector(7 downto 0);
	MDC : buffer std_logic_vector(1 downto 0);
	MDIO : inout std_logic_vector(1 downto 0);
	PhyPDn,PhyRst : buffer std_logic;
-- Two of eight TxClk chips from the PHYs are connected to the FPGA
	TxClk : in std_logic_vector(1 downto 0);
	Clk25MHz : buffer std_logic;
-- LVDS receivers
	FMRx : in std_logic_vector(7 downto 0);
-- Chip enable for octal LVDS receiver
	FMRxEn : buffer std_logic;
--- Heart beat data, asynchronous packets from top level FPGA
	HrtBtFM,DReqFM : in std_logic;
-- LVDS driver SPI port
	SPICS,SPISClk,SPIMOSI : buffer std_logic;
	SPIMISO : in std_logic;
-- Debug port
	Debug : buffer std_logic_vector(10 downto 1)

	-- debug outputs for testbench visibility
	-- synthesis translate_off
	;
	probe_MaskReg          : out std_logic_vector(7 downto 0);
	probe_PhyRxEmpty       : out std_logic_vector(7 downto 0);
	probe_Rx_active        : out std_logic_vector(7 downto 0);
	probe_PhyTxBuff_Count  : out std_logic_vector(10 downto 0);
	probe_ReadyStatus      : out std_logic_vector(7 downto 0);
	probe_UBT_in_progress  : out std_logic_vector(7 downto 0);
	probe_handshake_queued : out std_logic_vector(7 downto 0);
	probe_AutoTx_Port      : out std_logic_vector(2 downto 0);
	probe_AutoTx_Inhibit   : in  std_logic := '0';   -- TB drives this to gate AutoTx
	probe_PhyTxBuff_Empty  : out std_logic;           -- FIX 3: read-side empty flag
	probe_AutoTx_TimedOut  : out std_logic_vector(7 downto 0)
	-- synthesis translate_on	 
);

end Controller_FPGA2;

architecture behavioural of Controller_FPGA2 is

---------------------- Signal declarations -----------------------

Type Array_2x2 is Array(0 to 1) of std_logic_vector (1 downto 0);
Type Array_2x16 is Array(0 to 1) of std_logic_vector(15 downto 0);
Type Array_8x2 is Array(0 to 7) of std_logic_vector(1 downto 0);
Type Array_8x3 is Array(0 to 7) of std_logic_vector(2 downto 0);
Type Array_8x4 is Array (0 to 7) of std_logic_vector(3 downto 0);
Type Array_8x11 is Array(0 to 7) of std_logic_vector(10 downto 0);
Type Array_8x12 is Array(0 to 7) of std_logic_vector(11 downto 0);
Type Array_8x13 is Array(0 to 7) of std_logic_vector(12 downto 0);
Type Array_8x14 is Array(0 to 7) of std_logic_vector(13 downto 0);
Type Array_8x16 is Array(0 to 7) of std_logic_vector(15 downto 0);
Type Array_8x32 is Array(0 to 7) of std_logic_vector(31 downto 0);
Type Array_3x8x16 is Array(0 to 2) of Array_8x16;



-- Reset synchroniser signals
signal CpldRst_ibuf : std_logic;
signal CpldRst_r    : std_logic_vector(1 downto 0) := "00";
signal CpldRst_sync : std_logic;

-- FIX 1b: PhyPDn_sync(1) is the i50MHz-safe version of PhyPDn.
-- Use PhyPDn_sync(1) in any i50MHz-domain process that needs to READ PhyPDn.
-- Currently PhyPDn is only driven (not read) in i50MHz processes,
-- so no substitution is required today, but the synchroniser is in place
-- for any future i50MHz-domain gating on PhyPDn.
signal PhyPDn_sync  : std_logic_vector(1 downto 0) := "11";


-- Clock and reset signals
signal SysClk, i50MhzClk,ResetHi,GTPRxRst,LinkRst : std_logic;
-- Synchronous edge detectors of uC read and write strobes
Signal RDDL,WRDL : std_logic_vector (1 downto 0);
-- uC data bus
signal iCD,CDStage : std_logic_vector(15 downto 0);
signal AddrReg : std_logic_vector(11 downto 0);
-- TClk begin spill, end spill events
-- signal TrigCtrlReg : std_logic_vector(1 downto 0);

-- Timing interval counters
signal Counter1us : std_logic_vector (7 downto 0);
signal Counter10us : std_logic_vector (9 downto 0);
signal Counter1ms : std_logic_vector (16 downto 0);
signal Counter1s : std_logic_vector (27 downto 0);

signal Seq_Busy : std_logic;

-- Count the number of triggers
-- Make a test counter that increments with each read
signal TestCount : std_logic_vector (31 downto 0);
-- Uptime counter to check for un-anticipated resets
signal UpTimeCount,UpTimeStage : std_logic_vector (31 downto 0);

-- Spill counter, event word cout, spill word count
signal GPIDL,TrigDL,iWrtDL  : Array_2x2;
-- Test Pulse generator signals
signal PhaseAcc : std_logic_vector (31 downto 0);

signal PhyTxFifoRst_stretch : std_logic_vector(3 downto 0) := (others => '0');

signal AutoTx_FifoRst_req : std_logic := '0';

-- MIG LPDDR controller signals 
signal AuxClk : std_logic;
signal SDRdDat,SDWrtDat : std_logic_vector(31 downto 0);
signal DDRRd_Mux : std_logic_vector(15 downto 0);
signal SDWrtAd,SDWrtAdStage,SDRdAD,SDRdPtr : std_logic_vector(29 downto 0);
signal WrtAddrReg,AddrBuff_Out : std_logic_vector(27 downto 0); 
signal AddrBuff_wren,AddrBuff_rden,AddrBuff_full,AddrBuff_empty : std_logic;

signal SDwr_en,SDrd_en,SDCalDn,WrtCmdEn,SDRdCmdEn,SD_RstO,DDR_Reset : std_logic;
signal SDwr_full,SDwr_empty,SDwr_error,SDrd_full,SDrd_empty,
		 SDrd_overflow,SDrd_error,SDwr_underrun,RdHi_LoSel,
		 FifoRdD, WrtHi_LoSel : std_logic;
signal ResetCount,DDRWrtStat : std_logic_vector(3 downto 0);
signal SDrd_enD : std_logic_vector(2 downto 0);
constant RdBrstSiz  : std_logic_vector(5 downto 0) := "000111";
constant WrtBrstSiz : std_logic_vector(5 downto 0) := "000111";
signal SDwr_count,DDR_Rd_Cnt : std_logic_vector(6 downto 0);
signal SDWrtCmd,SDRdCmd : std_logic_vector(2 downto 0);
signal SDcmd_empty,SDcmd_full : std_logic_vector(1 downto 0);



signal DDRRd_EnD : std_logic;
-- Power-on initialisation: set ReadyStatus once after PLL lock, even with DDRRd_en=0
signal PowerOnReady_done : std_logic := '0';

-- Signals used by DDR write sequencer
signal PortNo : Integer range 0 to 7; 
signal MaskReg : std_logic_vector(7 downto 0);
signal EventWdCnt,EventStat : std_logic_vector (15 downto 0);
signal uBunch : std_logic_vector(31 downto 0);
signal PortWdCounter : Array_8x16;
signal EventRdy,FirstActive : std_logic;
signal PhyActivityCounter: Array_8x16;

Type Write_Seq_FSM is (Idle,ChkWrtBuff,SndCmd,WtCmdMtpy,CheckActive0,IncrPort0,
							  Rd_WdCount,Rd_uBunchHi,Rd_uBunchLo,Rd_Stat,IncrPort1,
							  CheckActive1,ResetPortNo,Write_Wd_Count,Wrt_Stat,Wrt_uBunchHi,
							  Wrt_uBunchLo,WrtDDR,WaitBuff,WritePad,IncrBuffCnt);
signal DDR_Write_Seq : Write_Seq_FSM;

-- Signals used by SDRAM readout sequencer
Type Read_Seq_FSM is (Idle,Wait0,SetAddr,CheckEmpty,FirstCmd,CheckRdBuff0,
							  RdWdCount,CheckWdCount,PrepareWordCnt,CheckRdBuff1,RdDataHi,RdDataLo);
signal DDR_Read_Seq : Read_Seq_FSM;
signal WaitCount : std_logic_vector(5 downto 0);
signal EvWdCount : std_logic_vector(15 downto 0);
signal ReadCount,DDRRdStat,TxBlkCount : std_logic_vector(2 downto 0);
-- Count of event pending for readout

-- SMI signals
signal SMI_Full,SMI_Empty,SMI_wreq,SMI_rdreq,ClkDiv,MDIORd : std_logic;
signal SPI_Count : std_logic_vector (10 downto 0); 
signal Strt,TA,R_W,iMDIO,ChainSel : std_logic_vector (1 downto 0);
signal SMI_Out : std_logic_vector (23 downto 0);
signal PhyAd : std_logic_vector (4 downto 0);
signal BitCount : std_logic_vector (5 downto 0);
signal SMIShift : std_logic_vector (31 downto 0);
signal SMIRdReg0,SMIRdReg1 : std_logic_vector (15 downto 0);
Type  SMI_FSM is (Idle,Load,Shift,Done);
Signal SMI_Shift : SMI_FSM;

-- Clock fanout SPI signals
Signal SPI_Adddr,SPI_Out,SPI_Shift : std_logic_vector (15 downto 0);
Signal SPIDiv : std_logic_vector (2 downto 0);
Signal SPIBitCnt : std_logic_vector (3 downto 0);
Signal SPI_WrtReq,SPI_rdreq,SPI_Full,SPI_Empty : std_logic;
Type  SPI_FSM is (Idle,Load_Addr,Shift_Addr,Shift_Data,Done);
Signal SPI_State : SPI_FSM;

signal PreambleTx_d : std_logic := '0';

-- Ethernet PHY signals
-- PhyTx signals
signal PhyRstCnt : std_logic_vector (1 downto 0);
signal PhyTxBuff_Full,PhyTxBuff_Empty,PhyTxBuff_rdreq,PreambleTx,DDRRd_en,
		 PhyTxBuff_wreq,TxEnReq,TxEnAck,DDRWrt_En,DDRWrt_EnD,InitReq,PhyDatSel : std_logic;
signal PhyTxBuff_Count : std_logic_vector (10 downto 0);
-- Add signal declarations:
signal PhyTxBuff_Empty_sync : std_logic_vector(1 downto 0) := "11";
signal PhyTxBuff_Empty_s    : std_logic := '1';  -- SysClk-safe empty
signal PreambleCnt : std_logic_vector (2 downto 0);
signal TxReg : std_logic_vector (3 downto 0);
signal Preamble,CRCErr_Reg,TxEnMask : std_logic_vector (7 downto 0);
signal PhyTxBuff_Out,PhyTxBuff_Dat : std_logic_vector (15 downto 0);
-- Add this signal declaration:
signal PhyTxBuff_Out_r : std_logic_vector(15 downto 0);
signal TxNibbleCount : std_logic_vector (1 downto 0);

-- FIX 4: 2-FF synchronizer for TxEnReq crossing SysClk ? i50MHz
signal TxEnReq_sync : std_logic_vector(1 downto 0) := "00";

-- 2-FF synchronizer for TxEnAck crossing i50MHz ? SysClk.
-- TxEnAck is registered in SMI_Proc (i50MHz domain) and is consumed in the
-- SysClk-domain main process (to clear TxEnReq) and in the SysClk-domain
-- uC readback mux. TxEnAck_sync(1) is the safe SysClk-domain copy and
-- must be used wherever TxEnAck is read inside SysClk logic.
signal TxEnAck_sync : std_logic_vector(1 downto 0) := "00";

-- Phy Rx Signals
signal PhyRxBuff_wreq,PhyRxBuff_rdreq,PhyRxBuff_Empty,HitFlag,
		 PhyRxBuff_Full,iCRS,PhyRxBuff_RdStat : std_logic_vector (7 downto 0);
signal RxBuffRst,FMRxBuffRst : std_logic;
Signal PhyRxBuff_RdCnt : Array_8x12;
signal PhyRxBuff_Out : Array_8x16;
signal RxPipeline : Array_3x8x16;
signal RxNibbleCount,RxClkDL,iRxDV : Array_8x2;
signal StartCount : Array_8x3;
signal FMRxBuff_Count : Array_8x11;
signal PhyRx : Array_8x4;
signal PhyTx : Array_8x4 := (
  0 => (others => '0'),
  1 => (others => '0'),
  2 => (others => '0'),
  3 => (others => '0'),
  4 => (others => '0'),
  5 => (others => '0'),
  6 => (others => '0'),
  7 => (others => '0')
  );
signal Rx_CRC_Out : Array_8x32;
signal RdCRCEn,RxCRCRst : std_logic_vector(7 downto 0);
signal RxFilled_sticky : std_logic := '0';


-- Signal used by timing/trigger LVDS FM receive links
signal RxDat : Array_2x16;
Type Array_OutRec_x2 is Array(0 to 1) of RxOutRec;
signal RxOut : Array_OutRec_x2;
Type Array_InRec_x2 is Array(0 to 1) of RxInRec;
signal RxIn : Array_InRec_x2;
signal i50MHz,RxFMClk,TrigWdCntRst,DRegSrc,LinkFIFOStat : std_logic;
signal TrigWdCount,DReqFMDL : std_logic_vector (3 downto 0);
signal TrigReqCount : std_logic_vector (7 downto 0);
signal DatReqBuff_rdreq,DatReqBuff_Full,DatReqBuff_Empty : std_logic; 
signal DatReqBuff_Count : std_logic_vector (9 downto 0); 
signal DatReqBuff_Out : std_logic_vector (15 downto 0);

-- Signal used by FEB LVDS FM receive links
signal FEBRxBuff_Dat,FEBRxBuff_Out : Array_8x16;
signal FEBRxBuff_wreq,FEBRxBuff_rdreq,FEBRxBuff_Empty,
		 FEBRxBuff_Full,PErrStat : std_logic_vector(7 downto 0);
Type Array_OutRec_x8 is Array(0 to 7) of RxOutRec;
signal FEBRxOut : Array_OutRec_x8;
Type Array_InRec_x8 is Array(0 to 7) of RxInRec;
signal FEBRxIn : Array_InRec_x8;

-- Signals used to determine which ports have FEBs plugged into them
signal RxDl : Array_8x2;
signal TransitionCount : Array_8x4;
signal Rx_active : std_logic_vector(7 downto 0);
signal Rx_active_rxclk    : std_logic_vector(7 downto 0);  -- RxFMClk-domain copy
signal Rx_active_sync     : Array_8x2;                      -- 2-FF synchroniser
signal LinkStatEn : std_logic;
signal LinkTxFullCnt : std_logic_vector(7 downto 0);

-- Serializer signals
signal FrameReg,ClockReg,LinkRegHi,LinkRegLo : std_logic_vector(4 downto 0);
signal TxFIFO_Out : std_logic_vector(8 downto 0);
signal LinkFIFO_Dat : std_logic_vector(17 downto 0);
signal BitClk,WdClk,PllLock,LockOut,Link_Stat_Req : std_logic;
signal LinkTxFull,LinkTxEmpty,LinkTxWrReq,LinkTxRDReq,TxValid,LinkTxTraceWrReq : std_logic;

-- PHY to FEB signal for prefetch filling
-- Ready notification / edge-detect signals (per PHY port)
signal phy_empty_d    : Array_8x2 := (others => (others => '1'));  -- delayed copy for edge detect
signal ReadyStatus    : std_logic_vector(7 downto 0) := (others => '0');  -- sticky ready bits

signal DeadWindowCount : Array_8x4;   -- counts consecutive zero-transition windows

-- debug trace buffer
signal LinkTxTraceRDReq : std_logic;
signal TxFIFOTrace_Out : std_logic_vector(8 downto 0);
signal LinkTxTrace_Cnt : std_logic_vector(12 downto 0);

-- SC: signals to handle overflows
signal tx_overflow : std_logic; -- flag indcating if we are sending an event with overflow
signal tx_overflow_cnt : std_logic_vector(15 downto 0); -- overflow counter for diagnostics


signal TxEnMask_next      : std_logic_vector(7 downto 0) := (others => '0'); -- SysClk domain
signal TxEnMask_sync      : std_logic_vector(7 downto 0) := (others => '0'); -- stage 1
signal TxEnMask_i50       : std_logic_vector(7 downto 0) := (others => '0'); -- stage 2 (i50MHz safe)


--signal tx_word_cnt : std_logic_vector(15 downto 0);
signal word_number : std_logic_vector(1 downto 0); -- shift register used to identify the first (word count) and second (status) words
signal EvWdCountTot : std_logic_vector(15 downto 0); -- latches the total EvWdCount used to send out (for the statemachine we start at +1)
-- TX MAX WORDS
-- we can at least try to cut at hit boundaries
-- the header is 4 words: count, status, uB-low, ub-high
-- each hit currently has 11 words
-- So we want to cut somewhere at 4 + N x 11 with integer N
-- N=1: 11*1 + 4 = 15 = 0x000f, for debugging, testing
-- N=744: 11*744 + 4 = 8188 = 0x1ffc
constant MAX_TX_WORDS : std_logic_vector(15 downto 0) := X"1ffc";-- X"2000";
-- constant UB_MISMATCH_STATUS_BIT : std_logic_vector(7 downto 0) := X"10";
constant OVERFLOW_STATUS_BIT : std_logic_vector(15 downto 0) := X"1000"; -- bit 12

-- Auto-TX FPGA automatic READY Packet sender 
signal PhyTxDin_FPGA      : std_logic_vector(15 downto 0) := (others => '0'); -- FPGA-produced write data
signal PhyTxDin_mux       : std_logic_vector(15 downto 0) := (others => '0'); -- selected DIN to PhyTx_Buff (uC or FPGA)
signal PhyTxBuff_wr_en_mux: std_logic := '0';                                  -- combined wr_en into PhyTx_Buff
signal PhyTxWrReq_FPGA    : std_logic := '0';                                  -- one-cycle FPGA write request
--type AutoTx_FSM is (AT_Idle, AT_WriteWords, AT_WaitTxFill, AT_WaitTxDrain, AT_WaitRxFill, AT_WaitDdrDrain);
type AutoTx_FSM is (AT_Idle, AT_WriteWords, AT_WaitTxFill, AT_WaitTxDrain, AT_WaitRxFill);
signal AutoTx_State : AutoTx_FSM := AT_Idle;
signal AutoTx_Port        : integer range 0 to 7 := 0;
signal AutoTx_WordIdx     : integer range 0 to 15 := 0; -- supports up to 16-word packets if needed
signal AutoTx_Claim : std_logic_vector(7 downto 0) := (others => '0'); -- one-hot claim for main to clear ReadyStatus
signal AutoTx_Claim_d : std_logic_vector(7 downto 0) := (others => '0');  --  delayed claim
signal AutoTx_ReArm : std_logic_vector(7 downto 0) := (others => '0');
signal AutoTx_Active      : std_logic := '0';  
signal AutoTx_Busy : std_logic_vector(7 downto 0) := (others => '0');
signal AutoTx_Target       : std_logic_vector(7 downto 0) := (others => '0'); -- one-hot chosen port
signal AutoTx_TimedOut : std_logic_vector(7 downto 0) := (others => '0');
type TimeoutCnt_t is array(0 to 7) of std_logic_vector(7 downto 0);
signal AutoTx_TimeoutCnt   : TimeoutCnt_t := (others => (others => '0'));
signal AutoTx_TimedOut_d   : std_logic_vector(7 downto 0) := (others => '0');
signal AutoTx_TimeoutClr   : std_logic_vector(7 downto 0) := (others => '0');
signal AutoTx_TxEnReqPulse : std_logic := '0';
signal AutoTx_TxEnReqHold : std_logic := '0';  -- sticky, driven only from main
-- CDC settling delay: counts down in SysClk domain after the last UBT word is
-- written, ensuring the i50MHz gray-code write-pointer synchroniser has had
-- sufficient time to de-assert PhyTxBuff_Empty before TxEnReq is raised.
-- 6 SysClk cycles @ 100 MHz (60 ns) >= 3 i50MHz cycles, which covers the
-- standard 2-FF synchroniser plus one cycle of margin.
signal AutoTx_Inhibit_int : std_logic := '0';    
signal AutoTx_RxFlush : std_logic_vector(7 downto 0) := (others => '0');


-- Sequential UBT handshake: trackd which port we are waiting on
signal AutoTx_WaitPort : integer range 0 to 7 := 0;
signal AutoTx_RxGot : std_logic_vector(7 downto 0) := (others => '0');  -- sticky "reply arrived"
signal PhyRxBuff_WasEmpty : std_logic_vector(7 downto 0) := (others => '1'); -- previous cycle empty
signal PhyRxFilled : std_logic_vector(7 downto 0) := (others => '0'); -- rising edge: empty->non empty
signal AutoTx_WaitTimeout : integer range 0 to 100000 := 0; -- ~100 ms at 100 MHz
-- Startup holdoff counter. Delays the PowerOnReady_done pulse until
-- the reset synchroniser, FIFOs, and AutoTx FSM have all fully settled.
-- 256 SysClk cycles @ 100 MHz = 2.56 us, well after CpldRst_sync stabilises.
signal StartupHoldoff : std_logic_vector(20 downto 0) := (others => '0');

signal ReArm_pending : std_logic_vector(7 downto 0) := (others => '0');

signal PhyRst_AutoDone : std_logic := '0';  -- prevents auto-init firing twice

signal PhyTxFifoRst_pulse : std_logic := '0';  -- one-shot reset for PhyTx FIFO-- Sticky latch: holds CurrentTarget value from the most recent PhyTxBuff_rdreq pulse.
-- Cleared by microcontroller write to LastTxTargetAddr.
signal LastTxTarget : std_logic_vector(7 downto 0) := (others => '0');
-- Add a synchronisation bridge for the µC LastTxTarget clear strobe
signal LastTxTarget_clr_req  : std_logic := '0';  -- set by main (SysClk)
signal LastTxTarget_clr_sync : std_logic_vector(2 downto 0) := "000"; -- sync in i50MHz
signal LastTxTarget_clr_stretch : std_logic_vector(4 downto 0) := (others => '0');
signal RoundRobin_Last : integer range 0 to 7 := 0;


  -- CurrentTarget is derived at transmit time to guarantee only one port
-- actually receives the 4 nibbles for a single FIFO read.  It picks
-- AutoTx_Target (one-hot) if set, otherwise selects the lowest-indexed
-- bit from TxEn so we never drive more than one PHY at once.
signal CurrentTarget : std_logic_vector(7 downto 0) := (others => '0');
constant ZERO4 : std_logic_vector(3 downto 0) := "0000";
constant ZERO8 : std_logic_vector(7 downto 0) := "00000000";
signal TxTarget_hold : std_logic_vector(7 downto 0) := (others => '0');
signal nibble_hold_cnt : integer range 0 to 4 := 0;
-- (3 stages: 2 for metastability, 1 for edge detect)-- Add near other CSR-related signals:

constant READY_WORD_COUNT : integer := 2; -- number of words in the READY packet (update if you change helper)


-- helper: returns true if all bits of the std_logic_vector are '0'
function is_all_zero(vec : std_logic_vector) return boolean is
begin
  for i in vec'range loop
    if vec(i) = '1' then
      return false;
	 end if;
  end loop;
  return true;
end function is_all_zero;

begin CpldRst_ibuf_inst : IBUF
  port map (
    I => CpldRst,
    O => CpldRst_ibuf
  );


-- =============================================================================
-- Process: rst_sync_proc
-- Purpose: Reset Synchroniser (2-stage pipeline)
--
-- Problem being solved:
--   CpldRst_ibuf is an ASYNCHRONOUS external signal. If it de-asserts (goes
--   high) at the same moment as a rising edge of SysClk, the internal flip-flops
--   that depend on CpldRst_sync could enter a metastable state, causing
--   unpredictable behaviour or lock-up. This synchroniser prevents that.
--
-- How it works:
--   ASSERT  (CpldRst_ibuf = '0'): Reset is applied IMMEDIATELY (asynchronously).
--           CpldRst_r is forced to "00" regardless of the clock.
--           This ensures the design goes into reset without delay.
--
--   DE-ASSERT (CpldRst_ibuf = '1'): The value '1' is shifted through a
--           2-stage flip-flop chain on consecutive rising edges of SysClk.
--           Only after TWO clock cycles does CpldRst_sync (= CpldRst_r(1))
--           see '1'. This gives any metastable event on the first FF (stage 0)
--           time to resolve before it is sampled by the second FF (stage 1).
--
-- Timing diagram:
--
--   CpldRst_ibuf  : _____|??????????????????????
--   SysClk        :  _|?|_|?|_|?|_|?|_|?|_|?|_
--   CpldRst_r(0)  : 0   0   0  [1]  1   1   1      <- latches '1' first
--   CpldRst_r(1)  : 0   0   0   0  [1]  1   1      <- CpldRst_sync goes high
--                                                       one cycle later
--
-- Downstream usage:
--   CpldRst_sync <= CpldRst_r(1);   -- used as the system-wide synchronous reset
--   ResetHi      <= not CpldRst_sync; -- active-high version for FIFO rst ports
-- =============================================================================

rst_sync_proc : process (SysClk, CpldRst_ibuf)
begin

  -- ASYNCHRONOUS reset branch:
  -- Fires immediately when the board-level reset pin goes low,
  -- regardless of where SysClk is in its cycle.
  -- Clears both pipeline stages so CpldRst_sync is driven low instantly.
  if CpldRst_ibuf = '0' then
    CpldRst_r <= "00";               -- both stages ? '0'; system is in reset

  -- SYNCHRONOUS de-assertion branch:
  -- Only executed on the rising edge of the 100 MHz system clock.
  elsif rising_edge(SysClk) then

    -- Stage 0: inject '1' into the pipeline.
    -- This FF may go metastable if CpldRst_ibuf de-asserts near the clock edge,
    -- but the following stage gives it a full clock period to resolve.
    CpldRst_r(0) <= '1';

    -- Stage 1: sample the (now resolved) output of stage 0.
    -- After this assignment CpldRst_r(1) ? aliased as CpldRst_sync ? is safe
    -- to use anywhere in the SysClk domain.
    CpldRst_r(1) <= CpldRst_r(0);

  end if;
end process rst_sync_proc;

-- =============================================================================
-- Process : PhyTxEmpty_sync_proc
-- Purpose : 2-stage Clock-Domain Crossing (CDC) synchroniser
--           Safe transfer of PhyTxBuff_Empty from the i50MHz (read-clock)
--           domain into the SysClk (100 MHz write/logic) domain.
--
-- Why this is needed:
--   PhyTx_Buff is a true dual-clock FIFO:
--       wr_clk => SysClk  (100 MHz)   -- written by AutoTx FSM / µC
--       rd_clk => i50MHz  (50 MHz)    -- drained by SMI_Proc (PHY Tx path)
--
--   The FIFO's 'empty' flag is generated and registered on the READ side,
--   meaning PhyTxBuff_Empty lives in the i50MHz domain.
--   Reading it directly inside a SysClk process would violate CDC rules
--   and risk metastability, causing the AutoTx FSM to misread the flag
--   and either send a premature TxEnReq or stall indefinitely.
--
-- Synchroniser architecture (3-register chain):
--
--   i50MHz domain          SysClk domain
--   ?????????????          ?????????????????????????????????????????????
--   PhyTxBuff_Empty ??? [FF0] ??? [FF1] ??? [FF2] ??? PhyTxBuff_Empty_s
--                    _sync(0)  _sync(1)        (registered alias)
--
--   FF0 (_sync(0)) : First capture register. May go metastable if the
--                    i50MHz signal changes near a SysClk rising edge,
--                    but is given a full SysClk cycle to resolve before
--                    being sampled by FF1.
--
--   FF1 (_sync(1)) : Second capture register. Samples the (now resolved)
--                    output of FF0. This is the standard 2-FF metastability
--                    barrier ? sufficient for most FPGA process corners.
--
--   FF2 (_Empty_s) : Clean registered copy of the synchronised flag.
--                    Decouples the synchroniser output from the fan-out
--                    load of downstream logic (AutoTx FSM, TxEnReq path).
--                    Also provides one extra cycle of margin for any
--                    corner-case metastability that survived FF1.
--
-- Reset behaviour:
--   On CpldRst_sync = '0' all three registers are preset to '1'.
--   '1' = FIFO is empty ? the safe conservative default.
--   This prevents AutoTx_Proc from asserting TxEnReq before the FIFO
--   and PHY are fully initialised after a reset event.
--
-- Timing example (100 MHz SysClk, 50 MHz i50MHz):
--
--   i50MHz domain   PhyTxBuff_Empty : ??????????|_____________
--   SysClk          _sync(0)        : ?????????????|___________  (1 cycle latency)
--   SysClk          _sync(1)        : ???????????????|_________  (2 cycle latency)
--   SysClk          _Empty_s        : ?????????????????|_______  (3 cycle latency)
--
--   Total worst-case latency = 3 SysClk cycles = 30 ns @ 100 MHz
--   This is acceptable: AutoTx only needs to know the FIFO went non-empty
--   before it pulses TxEnReq, not on a cycle-exact basis.
--
-- Downstream consumer:
--   PhyTxBuff_Empty_s is read exclusively in AutoTx_Proc (SysClk domain)
--   to gate the AT_WaitTxFill ? AT_WaitTxDrain state transition.
-- =============================================================================

PhyTxEmpty_sync_proc : process(SysClk)
begin
  if rising_edge(SysClk) then

    -- Synchronous reset: preset all stages to '1' (safe "empty" default)
    if CpldRst_sync = '0' then
      PhyTxBuff_Empty_sync <= "11";   -- both CDC stages ? empty
      PhyTxBuff_Empty_s    <= '1';    -- clean output    ? empty

    else

      -- Stage 0: capture PhyTxBuff_Empty from the i50MHz domain.
      -- This FF may briefly enter metastability but resolves within one
      -- SysClk period (10 ns), well inside Xilinx 7-series MTBF targets.
      PhyTxBuff_Empty_sync(0) <= PhyTxBuff_Empty;

      -- Stage 1: sample the resolved output of Stage 0.
      -- After this register the signal is fully safe to use in SysClk logic.
      PhyTxBuff_Empty_sync(1) <= PhyTxBuff_Empty_sync(0);

      -- Stage 2: registered alias for clean fan-out to downstream logic.
      -- AutoTx_Proc reads only PhyTxBuff_Empty_s (never _sync directly).
      PhyTxBuff_Empty_s       <= PhyTxBuff_Empty_sync(1);

    end if;
  end if;
end process;

--=============================================================================
-- Process : PhyPDn_sync_proc
-- Purpose : 2-stage Clock-Domain Crossing (CDC) synchroniser
--           Safe transfer of PhyPDn from the SysClk (100 MHz) domain into
--           the i50MHz (50 MHz) domain.
--
-- Why this is needed:
--   PhyPDn (PHY Power-Down control) is driven by the 'main' process which
--   runs on SysClk (100 MHz). The SMI_Proc and phy_out_gating processes run
--   on i50MHz (50 MHz). Reading PhyPDn directly inside an i50MHz process
--   would violate CDC rules and risk metastability on the receiving FFs.
--
-- Direction of crossing (note: opposite to PhyTxEmpty_sync_proc):
--
--   SysClk domain          i50MHz domain
--   ?????????????          ?????????????????????????????
--   PhyPDn        ??? [FF0] ??? [FF1] ??? PhyPDn_sync(1)
--                   _sync(0)  _sync(1)
--                             (safe to use in i50MHz logic)
--
--   FF0 (_sync(0)) : First capture register. Samples PhyPDn on each rising
--                    edge of i50MHz. May go metastable if PhyPDn changes
--                    near the clock edge, but is given a full i50MHz cycle
--                    (20 ns) to resolve before being sampled by FF1.
--
--   FF1 (_sync(1)) : Second capture register. Samples the (now resolved)
--                    output of FF0. This is the standard 2-FF metastability
--                    barrier ? sufficient for most FPGA process corners.
--                    PhyPDn_sync(1) is the safe, i50MHz-domain copy of PhyPDn.
--
-- Reset behaviour:
--   On CpldRst_sync = '0' both registers are preset to '1'.
--   '1' = PHY is powered DOWN ? the safe conservative default.
--   This guarantees that any i50MHz logic gated on PhyPDn_sync will see
--   the PHY as powered down during and immediately after a reset event,
--   preventing accidental transmission or MDIO traffic before the system
--   has fully initialised.
--
-- Timing example (100 MHz SysClk drives PhyPDn, 50 MHz i50MHz samples it):
--
--   SysClk domain  PhyPDn       : ???????|___________________
--   i50MHz         _sync(0)     : ????????????|______________  (1 i50MHz cycle latency)
--   i50MHz         _sync(1)     : ????????????????|__________  (2 i50MHz cycle latency)
--
--   Total worst-case latency = 2 i50MHz cycles = 40 ns @ 50 MHz.
--   This is acceptable: PhyPDn is a slow control signal that changes
--   only on microcontroller writes, never on a cycle-by-cycle basis.
--
-- Current usage note:
--   PhyPDn is currently only DRIVEN (not read) inside i50MHz processes,
--   so PhyPDn_sync(1) has no active consumer today. The synchroniser is
--   retained as a forward-compatibility measure: any future i50MHz logic
--   that needs to gate on the PHY power state must use PhyPDn_sync(1)
--   rather than PhyPDn directly.
-- =============================================================================

PhyPDn_sync_proc : process(i50MHz)
begin
  if rising_edge(i50MHz) then

    -- Synchronous reset: preset both stages to '1' (safe "PHY powered down" default).
    -- Prevents any i50MHz logic from seeing the PHY as active
    -- before the system has completed its reset sequence.
    if CpldRst_sync = '0' then
      PhyPDn_sync <= "11";

    else

      -- Stage 0: capture PhyPDn from the SysClk domain.
      -- This FF may briefly enter metastability if PhyPDn changes near
      -- the i50MHz clock edge, but resolves within one i50MHz period (20 ns).
      PhyPDn_sync(0) <= PhyPDn;

      -- Stage 1: sample the resolved output of Stage 0.
      -- After this register the signal is fully safe to use in i50MHz logic.
      -- Always reference PhyPDn_sync(1) ? never PhyPDn directly ? from
      -- within any i50MHz-clocked process.
      PhyPDn_sync(1) <= PhyPDn_sync(0);

    end if;
  end if;
end process PhyPDn_sync_proc;

CpldRst_sync <= CpldRst_r(1);
ResetHi <= not CpldRst_sync;

PhyRx(0) <= RxDA; TxDA <= PhyTx(0);
PhyRx(1) <= RxDB; TxDB <= PhyTx(1);
PhyRx(2) <= RxDC; TxDC <= PhyTx(2);
PhyRx(3) <= RxDD; TxDD <= PhyTx(3);
PhyRx(4) <= RxDE; TxDE <= PhyTx(4);
PhyRx(5) <= RxDF; TxDF <= PhyTx(5);
PhyRx(6) <= RxDG; TxDG <= PhyTx(6);
PhyRx(7) <= RxDH; TxDH <= PhyTx(7);

SysPLL : Sys_PLL
  port map
   (-- Clock in ports
    CLK_IN1_P => ClkB_P,
    CLK_IN1_N => ClkB_N,
    -- Clock out ports
    CLK_OUT1 => BitClk, -- 500 MHz serializer bit clock
    CLK_OUT2 => SysClk, -- 100 MHz system clock
    CLK_OUT3 => RxFMClk,-- 200 MHz FM Rx clock
	 CLK_OUT4 => i50MHz,
    -- Status and control signals
    RESET  => not CpldRst_ibuf,
	 LOCKED => PllLock);
	 
	 

-- Serializer to send data at 100MBytes/sec to the top level FPGA
-- The four bits are TX Clock, TX Frame and TxData(1 downto 0)
-- The serialization factor is five. Two five bit words are sent 
-- on two lanes to form a 10 bit result, one bit is used as data valid.

-- Clk    -_-_-_-_-_-_-_-_-_-_
-- Frame  -----_____-----_____
-- Lane 1 V1DDDV1dddV1DDDV1ddd
-- Lane 0 DDDDDdddddDDDDDddddd


FPGALinkTx : LinkTx
generic map ( sys_w => 4,   -- width of the data for the system
				  dev_w => 20)  -- width of the data for the device
port map ( -- From the device out to the system
  DATA_OUT_FROM_DEVICE(19) => ClockReg(0),
  DATA_OUT_FROM_DEVICE(18) => FrameReg(0),
  DATA_OUT_FROM_DEVICE(17) => LinkRegHi(0),
  DATA_OUT_FROM_DEVICE(16) => LinkRegLo(0),
  DATA_OUT_FROM_DEVICE(15) => ClockReg(1),
  DATA_OUT_FROM_DEVICE(14) => FrameReg(1),
  DATA_OUT_FROM_DEVICE(13) => LinkRegHi(1),
  DATA_OUT_FROM_DEVICE(12) => LinkRegLo(1),
  DATA_OUT_FROM_DEVICE(11) => ClockReg(2),
  DATA_OUT_FROM_DEVICE(10) => FrameReg(2),
  DATA_OUT_FROM_DEVICE(9) => LinkRegHi(2),
  DATA_OUT_FROM_DEVICE(8) => LinkRegLo(2),
  DATA_OUT_FROM_DEVICE(7) => ClockReg(3),
  DATA_OUT_FROM_DEVICE(6) => FrameReg(3),
  DATA_OUT_FROM_DEVICE(5) => LinkRegHi(3),
  DATA_OUT_FROM_DEVICE(4) => LinkRegLo(3),
  DATA_OUT_FROM_DEVICE(3) => ClockReg(4),
  DATA_OUT_FROM_DEVICE(2) => FrameReg(4),
  DATA_OUT_FROM_DEVICE(1) => LinkRegHi(4),
  DATA_OUT_FROM_DEVICE(0) => LinkRegLo(4),
  DATA_OUT_TO_PINS_P(3) => LinkClk_P,
  DATA_OUT_TO_PINS_P(2) => LinkFR_P,
  DATA_OUT_TO_PINS_P(1 downto 0) => LinkD_P,
  DATA_OUT_TO_PINS_N(3) => LinkClk_N,
  DATA_OUT_TO_PINS_N(2) => LinkFR_N,
  DATA_OUT_TO_PINS_N(1 downto 0) => LinkD_N,
-- Clock and reset signals
  CLK_IN  => BitClk,      -- Fast clock from PLL/MMCM 
  CLK_DIV_IN => SysClk,   -- Slow clock from PLL/MMCM
  LOCKED_IN => PllLock,
  LOCKED_OUT => LockOut,
  IO_RESET => LinkRst);  -- Reset signal for IO circuit

-- MIG DDR controller
LPDDRCtrl : LPDDR_Ctrl
 generic map(
    C3_P0_MASK_SIZE => 4, C3_P0_DATA_PORT_SIZE => 32,
    C3_P1_MASK_SIZE => 4, C3_P1_DATA_PORT_SIZE => 32,
    C3_MEMCLK_PERIOD => 6277,  C3_RST_ACT_LOW => 0,
    C3_INPUT_CLK_TYPE => "DIFFERENTIAL",
    C3_CALIB_SOFT_IP => "TRUE",  C3_SIMULATION => "FALSE",
    DEBUG_EN => 0, C3_MEM_ADDR_ORDER => "ROW_BANK_COLUMN",
    C3_NUM_DQ_PINS => 16, C3_MEM_ADDR_WIDTH => 14,
    C3_MEM_BANKADDR_WIDTH => 2)
port map (
   mcb3_dram_dq => SDD, mcb3_dram_a => SDA,
   mcb3_dram_ba => BA,  mcb3_dram_cke => SDCKE,
   mcb3_dram_ras_n => RAS, mcb3_dram_cas_n => CAS,
   mcb3_dram_we_n => SDWE, mcb3_dram_dm => LDM,
   mcb3_dram_udqs => UDQS,	mcb3_rzq => SDRzq,
   mcb3_dram_udm =>  UDM,  mcb3_dram_dqs => LDQS,
   mcb3_dram_ck => SDClk_P, mcb3_dram_ck_n => SDClk_N,
   c3_sys_clk_p => VXO_P, c3_sys_clk_n => VXO_N,
   c3_sys_rst_i => DDR_Reset, c3_calib_done => SDCalDn,
	c3_clk0 => AuxClk,
   c3_rst0 => SD_RstO,
   c3_p2_cmd_clk => SysClk,     c3_p2_cmd_en => WrtCmdEn,
   c3_p2_cmd_instr => SDWrtCmd, c3_p2_cmd_bl => WrtBrstSiz,
   c3_p2_cmd_byte_addr => SDWrtAdStage,
   c3_p2_cmd_empty => SDcmd_empty(0), c3_p2_cmd_full => SDcmd_full(0),
   c3_p2_wr_clk => SysClk,  c3_p2_wr_en => SDwr_en,
   c3_p2_wr_mask => "0000",  c3_p2_wr_data => SDWrtDat,
   c3_p2_wr_full => SDwr_full,  c3_p2_wr_empty => SDwr_empty,
   c3_p2_wr_count => SDwr_count,
   c3_p2_wr_underrun => SDwr_underrun,
   c3_p2_wr_error => SDwr_error,  c3_p3_cmd_clk => SysClk,
   c3_p3_cmd_en => SDRdCmdEn, c3_p3_cmd_instr => SDRdCmd,
   c3_p3_cmd_bl => RdBrstSiz, c3_p3_cmd_byte_addr => SDRdAD,
   c3_p3_cmd_empty => SDcmd_empty(1), c3_p3_cmd_full =>  SDcmd_full(1),
   c3_p3_rd_clk => SysClk,  c3_p3_rd_en => SDrd_en,
   c3_p3_rd_data => SDRdDat,  c3_p3_rd_full => SDrd_full,
   c3_p3_rd_empty => SDrd_empty,
	c3_p3_rd_count => DDR_Rd_Cnt,
   c3_p3_rd_overflow => SDrd_overflow,  c3_p3_rd_error => SDrd_error
); 

-- Buffer for Link Tx data
-- Data is 18 bits, output is 9 bits
LinkBuff : LinkTxFIFO
  port map (rst => ResetHi,
	 wr_clk => SysClk, rd_clk => SysClk,
    wr_en => LinkTxWrReq, rd_en => LinkTxRDReq, 
	 din => LinkFIFO_Dat,
    dout => TxFIFO_Out,
    full => LinkTxFull, empty => LinkTxEmpty);
	 
-- copy of the same buffer as trace buffer for debug purpose
LinkBuffTrace : LinkTxFIFOTrace
  port map (rst => ResetHi,
	 wr_clk => SysClk, rd_clk => SysClk,
    wr_en => LinkTxTraceWrReq, rd_en => LinkTxTraceRDReq, 
	 din => LinkFIFO_Dat,
    dout => TxFIFOTrace_Out,
    full => open, empty => open,
	 rd_data_count => LinkTxTrace_Cnt);

LinkRegHi(4) <= TxValid and not LinkTxEmpty;
-- Use this as a Rx_Active data flag
LinkRegHi(3) <= TxFIFO_Out(8);
LinkRegHi(2 downto 0) <= TxFIFO_Out(7 downto 5) when LinkTxEmpty = '0' else "100";
LinkRegLo <= TxFIFO_Out(4 downto 0) when LinkTxEmpty = '0' else "00110";

-- Buffer for MDIO data
SMI_Buff : SMI_FIFO
  PORT MAP (wr_clk => SysClk, rst => ResetHi,
	 rd_clk => i50MHz,
	 din(23) => MDIORd,
	 din(22 downto 16) => uCA(6 downto 0),
    din(15 downto 0) => uCD,
    wr_en => SMI_wreq, rd_en => SMI_rdreq,
    dout => SMI_Out, full => SMI_Full,
    empty => SMI_Empty);


PhyTx_Buff : PhyTxBuff
PORT MAP ( rst => (ResetHi or PhyTxFifoRst_pulse),   wr_clk => SysClk,
  rd_clk => i50MHz, din => PhyTxDin_mux,
  wr_en => PhyTxBuff_wr_en_mux, rd_en => PhyTxBuff_rdreq,
  dout => PhyTxBuff_Out, full => PhyTxBuff_Full,
  empty => PhyTxBuff_Empty,
  wr_data_count => PhyTxBuff_Count);


	PhyTxDin_mux <= uCD when PhyTxBuff_wreq = '1' else PhyTxDin_FPGA;
	PhyTxBuff_wr_en_mux <= PhyTxBuff_wreq or PhyTxWrReq_FPGA;


-- Buffer for SPI data
SPITx_Buff : PhyTxBuff
  PORT MAP ( rst => ResetHi, wr_clk => SysClk,
    rd_clk => i50MHz, din => uCD,
    wr_en => SPI_WrtReq, rd_en => SPI_rdreq,
    dout => SPI_Out, full => SPI_Full,
    empty => SPI_Empty,
	 wr_data_count => SPI_Count);


GTPRxRst <= '1' when CpldRst_sync = '0' 
  	or (CpldCS = '0' and uCWR = '0' and uCA(11 downto 10) = "00" and uCA(9 downto 0) = GTPFIFOAddr and uCD(0) = '1') else '0';

HrtBtRx : FM_Rx
	generic MAP(Pwidth => 16)
	port map(SysClk => SysClk, RxClk => RxFMClk, 
				reset => ResetHi,
				Rx_In => RxIn(1),
				Data => RxDat(1), 
				Rx_Out => RxOut(1));
RxIn(1).FM <= HrtBtFM;

FMRxBuffRst <= '1' when (uCWR = '0' and CpldCS = '0' and AddrReg(11 downto 10) = GA 
						 and AddrReg(9 downto 0) = FMRxErrAddr and uCD(8) = '1') or ResetHi = '1' else '0';

-- add the missing data-request FM receiver on RxIn(0)/RxOut(0)
DReqFMRx : FM_Rx
    generic MAP(Pwidth => 16)
    port map(SysClk  => SysClk,  RxClk => RxFMClk,
             reset   => ResetHi,
             Rx_In   => RxIn(0),
             Data    => RxDat(0),
             Rx_Out  => RxOut(0));
RxIn(0).FM <= DReqFM;


--add the missing DatReqBuff FIFO (SCFifo_512x16 matches the
--      declared 10-bit DatReqBuff_Count and 16-bit DatReqBuff_Out)
DatReq_Buff : SCFifo_512x16
    PORT MAP (
        clk        => SysClk,
        rst        => ResetHi,
        din        => RxDat(0),          -- 16-bit FM packet word
        wr_en      => RxOut(0).Done,     -- pulse when FM word is complete
        rd_en      => DatReqBuff_rdreq,
        dout       => DatReqBuff_Out,
        full       => DatReqBuff_Full,
        empty      => DatReqBuff_Empty,
        data_count => DatReqBuff_Count);


AddrBuff : SCFIFO_1Kx28
  PORT MAP(rst => ResetHi, clk => SysClk,
    din => WrtAddrReg,
    wr_en => AddrBuff_wren,
    rd_en => AddrBuff_rden, 
    dout => AddrBuff_Out,
    full => AddrBuff_full,
    empty => AddrBuff_empty);

----------------------------------------------------------------------------
---- Loop through eight Phy receive channels, eight FM receive channels ----
----------------------------------------------------------------------------

Debug(3) <= FEBRxBuff_Empty(0);
Debug(4) <= FEBRxOut(0).Done;
Debug(5) <= FEBRxBuff_rdreq(0);
Debug(6) <= FMRx(0);

-- 2 BRAM based
Gen_FEBRxBuffs : for i in 0 to 7 generate
-- 4k deep input buffers for eight Rx Phy channels
FEBRx_Buff : PhyRxBuff
  PORT MAP (rst => RxBuffRst or AutoTx_RxFlush(i), rd_clk => SysClk,
    wr_clk => RxFMClk, din => RxPipeline(2)(i),
    wr_en => PhyRxBuff_wreq(i), rd_en => PhyRxBuff_rdreq(i),
    dout => PhyRxBuff_Out(i), full => PhyRxBuff_Full(i),
    empty => PhyRxBuff_Empty(i),
	 rd_data_count => PhyRxBuff_RdCnt(i));
end generate;


Gen_RxBuffs : for i in 0 to 7 generate

-- CRC generators for receive data CRC checking. 
RxCRC : CRC32_D4 
 port map(rst => RxCRCRst(i), clk => RxFMClk, crc_en => RdCRCEn(i), 
			 data_in => PhyRx(i),
			 crc_out => Rx_CRC_Out(i));





-- FM receivers for FEB FM links
FEBFMRx : FM_Rx
	generic MAP(Pwidth => 16)
	port map(SysClk => SysClk, RxClk => RxFMClk, 
				reset => ResetHi,
				Rx_In => FEBRxIn(i),
				Data => FEBRxBuff_Dat(i), 
				Rx_Out => FEBRxOut(i));

FEBRxIn(i).FM <= FMRx(i);
PErrStat(i) <= FEBRxOut(i).Parity_Err;

-- 1k deep input buffers for eight FM Rx links
FMRx_Buff : SCFIFO1Kx16
  PORT MAP ( rst => FMRxBuffRst, clk => SysClk,
    din => FEBRxBuff_Dat(i),
    wr_en => FEBRxOut(i).Done, rd_en => FEBRxBuff_rdreq(i),
    dout => FEBRxBuff_Out(i), full => FEBRxBuff_Full(i),
    empty => FEBRxBuff_Empty(i),
	 data_count => FMRxBuff_Count(i));

PhyRx_Proc : process(CpldRst_sync, RxFMClk)

begin


 if CpldRst_sync = '0' or PllLock = '0' then


    PhyRxBuff_wreq(i) <= '0'; RxClkDL(i) <= "00";
	 RxNibbleCount(i) <= "00"; CRCErr_Reg(i) <= '0';
	 RxPipeline(0)(i) <= (others => '0');
	 RxPipeline(1)(i) <= (others => '0');
	 RxPipeline(2)(i) <= (others => '0');
	 iCRS(i) <= '0'; iRxDV(i) <= "00"; 
	 RdCRCEn(i) <= '0'; RxCRCRst(i) <= '1';
	 StartCount(i) <= "000";
	 PhyActivityCounter(i) <= (others => '0');
	


 elsif rising_edge(RxFMClk) then
 
-- PhyActivityCounter
   if PhyRxBuff_wreq(i) = '1' then 
	    PhyActivityCounter(i) <= PhyActivityCounter(i) + 1;
	else
		 PhyActivityCounter(i) <= PhyActivityCounter(i);
	end if;
	     

-- Synchronous edge detector for 25MHz PhyRx clock
	RxClkDL(i)(0) <= RxClk(i);
	RxClkDL(i)(1) <= RxClkDL(i)(0);

-- Registered copies of the carrier sense and data valid signals
	iRxDV(i)(0) <= RxDV(i); iCRS(i) <= CRS(i);
	iRxDV(i)(1) <= iRxDV(i)(0);

-- CRC Error register
	if iRxDV(i) = 2 and Rx_CRC_Out(i) /= X"C704DD7B" then
	CRCErr_Reg(i) <= '1';
-- Writing a '1' to the appropriate location will clear the error bit
	elsif uCWR = '0' and CpldCS = '0' and uCA(11 downto 10) = GA and uCA(9 downto 0) = CRCErrAddr
		and uCD(i) = '1' then CRCErr_Reg(i) <= '0';
	else CRCErr_Reg(i) <= CRCErr_Reg(i);
	end if;

-- Modulo four counter used to assemble nibbles into words
-- Increment nibble count while DAV is high
   if iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
   then RxNibbleCount(i) <= RxNibbleCount(i) + 1;  
   elsif RxBuffRst = '1' or RxDV(i) = '0' then RxNibbleCount(i) <= "00"; 
   else RxNibbleCount(i) <= RxNibbleCount(i); 
   end if;

-- Reset the CRC generator during the preamble
	if StartCount(i) = 1 and RxClkDL(i) = 2 and RxNibbleCount(i) = 2
	then RxCRCRst(i) <= '1';
	else RxCRCRst(i) <= '0';
	end if;

-- Enable the CRC generator for one clock tick per nibble
	if StartCount(i) > 3 and RxClkDL(i) = 2  then RdCRCEn(i) <= '1'; 
	else RdCRCEn(i) <= '0'; 
	end if;

-- Data valid stays true for eight nibbles during the CRC postamble. We want
-- only payload to go to the receive FIFO. Delay the data by two three word
-- periods with a pipeline. Load the first pipeline resigter one nibble at a time
  if RxClkDL(i) = 2 then
  Case RxNibbleCount(i) is
	When "00" => RxPipeline(0)(i)(3 downto 0) <= PhyRx(i);
	When "01" => RxPipeline(0)(i)(7 downto 4) <= PhyRx(i);
	When "10" => RxPipeline(0)(i)(11 downto 8) <= PhyRx(i);
	When "11" => RxPipeline(0)(i)(15 downto 12) <= PhyRx(i);
	When others => RxPipeline(0)(i) <= RxPipeline(0)(i);
	end Case;
	else RxPipeline(0)(i) <= RxPipeline(0)(i);
	end if;

-- After each group of four nibbles has arrived, advance the data 
-- through the pipeline 
 if iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
		and RxNibbleCount(i) = 0 
  then RxPipeline(1)(i) <= RxPipeline(0)(i); 
		 RxPipeline(2)(i) <= RxPipeline(1)(i); 
  else RxPipeline(1)(i) <= RxPipeline(1)(i);
		 RxPipeline(2)(i) <= RxPipeline(2)(i); 
  end if;

-- Use this counter to skip over the preamble at the beginning of the packet
   if iRxDV(i)(0) = '0' or iCRS(i) = '0' then StartCount(i) <= "000";
elsif StartCount(i) /= 6 and iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
	and RxNibbleCount(i) = 3 
	then StartCount(i) <= StartCount(i) + 1;
else StartCount(i) <= StartCount(i);
end if;

-- When the word has been assembled and pipeline delayed, write to the FIFO
 if StartCount(i) = 6 and iRxDV(i)(0) = '1' and iCRS(i) = '1' and RxClkDL(i) = 2
  and RxNibbleCount(i) = 3  and MaskReg(i) = '1'
 then PhyRxBuff_wreq(i) <= '1'; 
 else PhyRxBuff_wreq(i) <= '0'; 
  end if;

 end if; -- CpldRst

end process PhyRx_Proc;



end generate;


-- Serializer for MDC links on the Phy chips, SPI ports on the LVDS Tx Chips --
-- Clock runs at 50 MHz, MDI bit period is 40ns, SPI bit perios is 80ns
SMI_Proc : process(CpldRst_sync, i50MHz)

begin 

if CpldRst_sync = '0' then

Clk25MHz <= '0'; SMI_rdreq <= '0'; MDC <= "00"; 
TxEn <= (others => '0'); Strt <= "01"; TA <= "10"; 
R_W <= "01"; PhyAd <= "00000"; BitCount <= (others => '0');
SMIShift <= (others => '0'); SMIRdReg0 <= (others => '0');
SMIRdReg1 <= (others => '0'); SMI_Shift <= Idle;
TxNibbleCount <= "00";
PhyTxBuff_rdreq <= '0'; TxEnAck <= '0'; PreambleTx <= '0';
PreambleCnt <= "000"; Preamble <= X"00";
PhyTxBuff_Out_r <= (others => '0');
TxEnMask <= X"00"; -- was X"FF"
 --  TxEnMask resets to X"00" here on CpldRst_sync='0', and
-- the UBTTarget FIFO resets in UBTTarget_FIFO_proc on ResetHi='1' (see
-- ResetHi <= not CpldRst_sync below).  Both are driven by the same reset
-- source, so a soft reset can never leave TxEnMask and the target FIFO
-- out of sync.

-- Clock fanout SPI signals
SPI_Adddr <= X"0800"; SPI_Shift <= (others => '0');
SPIDiv <= "000"; SPIBitCnt <= (others => '0');
SPI_State <= Idle; SPICS <= '1'; SPISClk <= '0'; 
SPI_rdreq <= '0';


elsif rising_edge (i50MHz) then 

Clk25MHz <= not Clk25MHz; 





-------------------- Logic for Phy MDIO serial control --------------------

--(Idle,Load,Shift,Done);
Case SMI_Shift is
	   When Idle => 	
				if SMI_Empty = '0' and MDC(0) = '0' then SMI_Shift <= Load;
				else SMI_Shift <= Idle;
				end if;
		When Load =>
				if MDC(0) = '0' then SMI_Shift <= Shift;
				else SMI_Shift <= Load;
				end if;
		When Shift => if BitCount = 0 and MDC(0) = '0' then SMI_Shift <= Done;
						 else SMI_Shift <= Shift;
						 end if;
		When Done =>
				if MDC(0) = '0' then SMI_Shift <= Idle;
				else SMI_Shift <= Done;
				end if;
		When others => SMI_Shift <= Idle;
 end Case;

-- Map logical onto physical adresses
Case SMI_Out(22 downto 21) is
	When "00" => PhyAd <= "00001";
	When "01" => PhyAd <= "00011";
	When "10" => PhyAd <= "00111";
	When "11" => PhyAd <= "01111";
	When others => PhyAd <= "00001";
end case;

if SMI_Out(23) = '0' then R_W <= "01";
else R_W <= "10";
end if;

-- SMI shift register
if SMI_Shift = Load and MDC(0) = '0'
then SMIShift <= Strt & R_W & PhyAd & SMI_Out(20 downto 16) & TA & SMI_Out(15 downto 0);
elsif BitCount /= 0 and MDC(0) = '0' and SMI_Shift = Shift 
	then SMIShift <= (SMIShift(30 downto 0) & '0');
end if;

-- After one word has been serialized, issue a FIFO read
if SMI_Shift = Done and MDC(0) = '0'
then SMI_rdreq <= '1';
else SMI_rdreq <= '0';
end if;

-- Serial bit counter
if SMI_Shift = Load and BitCount = 0 and MDC(0) = '0'
--then BitCount <= "11111";
then BitCount <= "100000";
elsif BitCount /= 0 and SMI_Shift = Shift and MDC(0) = '0' then BitCount <= BitCount - 1;
end if;

MDC <= not MDC;


-- Clock in any readback data
if R_W = "10" and SMI_Shift = Shift and BitCount /= 0 and MDC(0) = '0'
then SMIRdReg0 <= SMIRdReg0(14 downto 0) & MDIO(0);
	  SMIRdReg1 <= SMIRdReg1(14 downto 0) & MDIO(1);
end if;

-- Choose which MDIO chain is active
if ChainSel(0) = '0' or SMI_Shift /= Shift or (R_W = "10" and BitCount <= 17)
	then MDIO(0) <= 'Z'; 
  else MDIO(0) <= SMIShift(31); 
end if;

if ChainSel(1) = '0' or SMI_Shift /= Shift or (R_W = "10" and BitCount <= 17)
	then MDIO(1) <= 'Z';
 else MDIO(1) <= SMIShift(31);
end if;

---------------------------- Logic for Phy transmit -----------------------

-- TxEn is used to hold off sending Phy data until a block of data has been 
-- loaded into the transmit FIFO
if Clk25MHz = '0' and TxEnAck = '0' and TxEnReq_sync(1) = '1'
   and PhyTxBuff_Empty = '0' then TxEnAck <= '1';
elsif PhyTxBuff_Empty = '1' and Clk25MHz = '0' and TxNibbleCount = "11" then TxEnAck <= '0';
else TxEnAck <= TxEnAck;
end if;


-- Register FIFO output every cycle; use _r in the nibble mux
-- This adds one i50MHz cycle of latency = 20ns, well within nibble period (40ns)
PhyTxBuff_Out_r <= PhyTxBuff_Out;

PreambleTx_d <= PreambleTx;

-- Preamble: X"55",X"55",X"55",X"55",X"55",X"55",X"D5"
-- Use PreambleTx signal to distinguish between preamble and data. When seven 
-- bytes of preamble have been sent, start sending data
   if Clk25MHz = '0' and TxEnAck = '1' and TxEn = 0 and PreambleTx = '0' then PreambleTx <= '1';
elsif Clk25MHz = '0' and PreambleTx = '1' and PreambleCnt = 6 then PreambleTx <= '0';
else PreambleTx <= PreambleTx;
end if;

-- The modulo 4 nibble count needs to do a stutter step due to the odd count
-- of the preamble bytes
if Clk25MHz = '0' and TxEnAck = '1' and not(PreambleTx = '1' and PreambleCnt = 6)
	then TxNibbleCount <= TxNibbleCount + 1; 
elsif Clk25MHz = '0' and (TxEnAck = '0' or (PreambleTx = '1' and PreambleCnt = 6 ))
	then TxNibbleCount <= "00"; 
else TxNibbleCount <= TxNibbleCount; 
end if;

-- Counter used as a timer during preamble transmission
if Clk25MHz = '0' and PreambleTx = '1'
	and TxNibbleCount(0) = '0' and PreambleCnt /= 6 
	then PreambleCnt <= PreambleCnt + 1;
elsif Clk25MHz = '0' and TxNibbleCount(0) = '0' and PreambleCnt = 6 
	then PreambleCnt <= "000";
else PreambleCnt <= PreambleCnt;
end if; 

-- Affter sending six bytes of 0x55, change the preamble value to 0xD5
if Clk25MHz = '0' and PreambleCnt = 5 then Preamble <= X"D5";
elsif Clk25MHz = '0' and PreambleCnt /= 5 then Preamble <= X"55";
else Preamble <= Preamble;
end if;


-- Multiplexer to choose nibble to Tx outs 
Case TxNibbleCount is
  When "00" =>
    if PreambleTx = '1' then TxReg <= Preamble(3 downto 0);
    else TxReg <= PhyTxBuff_Out_r(3 downto 0);
    end if;
  When "01" =>
    if PreambleTx = '1' then TxReg <= Preamble(7 downto 4);
    else TxReg <= PhyTxBuff_Out_r(7 downto 4);
    end if;
  When "10" =>
    if PreambleTx = '1' then TxReg <= Preamble(3 downto 0);
    else TxReg <= PhyTxBuff_Out_r(11 downto 8);
    end if;
  When "11" =>
    if PreambleTx = '1' then TxReg <= Preamble(7 downto 4);
    else TxReg <= PhyTxBuff_Out_r(15 downto 12);
    end if;
  When others =>
    TxReg <= TxReg;
end Case;



-- When four nibbles have been sent, get the next word from the buffer
-- Move rdreq from NibbleCount=2 to NibbleCount=3 to allow FIFO output to settle
-- before the first nibble of the next word is sample
if TxNibbleCount = 2 and Clk25MHz = '0' and PreambleTx = '0' then PhyTxBuff_rdreq <= '1'; 
else PhyTxBuff_rdreq <= '0'; 
end if;

-- When preamble starts, load TxEnMask from the target tag FIFO.
-- ROBUSTNESS FIX: with TxEnAck now gated on UBTTarget_empty='0' above, the
-- empty branch below should be unreachable.  If it is ever taken (e.g. due
-- to a reset glitch or an unexpected TxEnReq path), drive TxEnMask to X"00"
-- so the symptom is "no transmission at all" ? a safe, observable failure ?
-- rather than "transmission to whatever stale port was last loaded".
if PreambleTx = '1' and PreambleTx_d = '0' then
  TxEnMask        <= TxEnMask_i50; -- one-hot lane mask propagated from SysClk domain
else
  TxEnMask        <= TxEnMask;  -- hold
end if;


-- Set Tx Enables high until the Tx FIFO is empty
  if Clk25MHz = '0' and TxEnAck = '1' then TxEn <= TxEnMask;
  elsif Clk25MHz = '0' and TxEnAck = '0'  then TxEn <= X"00"; 
  end if;

---------------- Logic used to set up the LVDS clock buffer ---------------

-- Free-running prescaler: divides i50MHz (50 MHz) by 8 to produce a
-- 6.25 MHz SPI clock reference. All state transitions and shift events
-- are gated on SPIDiv = 7 (falling edge of SPISClk) to ensure every
-- control change is aligned to the same prescaler boundary.
SPIDiv <= SPIDiv + 1;


-- -------------------------------------------------------------------------
-- SPI State Machine (Idle, Load_Addr, Shift_Addr, Shift_Data, Done)
-- -------------------------------------------------------------------------
Case SPI_State is

    -- Wait until the µC has loaded at least one word into the SPI FIFO.
    -- Transition is gated on SPIDiv = 7 so SPICS assertion and the first
    -- shift are both aligned to the prescaler boundary with no partial bit.
    When Idle =>
        if SPI_Empty = '0' and SPIDiv = 7 then SPI_State <= Load_Addr;
        else SPI_State <= Idle;
        end if;

    -- Hold for one full SPIDiv cycle to allow SPI_Shift to be loaded with
    -- the block-write address (X"0800") and SPICS to be asserted before
    -- the first clock edge is produced.
    When Load_Addr =>
        if SPIDiv = 7 then SPI_State <= Shift_Addr;
        else SPI_State <= Load_Addr;
        end if;

    -- Shift out the 16-bit block-write address MSB-first. SPIBitCnt
    -- decrements from 15 to 0 over 16 SPI clock cycles; transition to
    -- the data phase only when the final bit has been clocked out.
    When Shift_Addr =>
        if SPIDiv = 7 and SPIBitCnt = 0 then SPI_State <= Shift_Data;
        else SPI_State <= Shift_Addr;
        end if;

    -- Shift out data words from the SPI FIFO, one 16-bit word per 16
    -- clock cycles. Remain in this state while more FIFO words are
    -- pending; transition to Done only when the last bit of the last
    -- word has been shifted and the FIFO is confirmed empty.
    When Shift_Data =>
        if SPIDiv = 7 and SPIBitCnt = 0 and SPI_Empty = '1'
            then SPI_State <= Done;
        else SPI_State <= Shift_Data;
        end if;

    -- De-assert SPICS and return to Idle. One-cycle state: no condition
    -- needed as the transition is unconditional.
    When Done => SPI_State <= Idle;

end Case;


-- Issue a FIFO read request two bits before the end of the current word
-- (SPIBitCnt = 14). This gives the FIFO one full word period (16 SPI clock
-- cycles = ~2.56 µs) to present the next data word at SPI_Out before the
-- shift register reloads at SPIBitCnt = 0, preventing any stall between
-- consecutive words.
if SPIDiv = 7 and SPI_State = Shift_Data and SPIBitCnt = X"E" then
    SPI_rdreq <= '1';
else
    SPI_rdreq <= '0';
end if;


-- -------------------------------------------------------------------------
-- SPI shift register (MSB-first, left-rotation)
--   Load_Addr              : preload fixed block-write address X"0800"
--                            (block write to register 0 of the fanout chip)
--   Shift_Addr/Data cnt=0  : reload from SPI_Out (next FIFO word, already
--                            presented after the earlier SPI_rdreq strobe)
--   Shift_Addr/Data cnt?0  : rotate left by one bit at each SPIDiv=7 tick,
--                            promoting the next bit into position 15 for
--                            SPIMOSI to present on the next rising clock edge
-- -------------------------------------------------------------------------
if SPI_State = Load_Addr then
    SPI_Shift <= X"0800";                                    -- block-write address word
elsif (SPI_State = Shift_Addr or SPI_State = Shift_Data)
      and SPIDiv = 7 and SPIBitCnt = 0 then
    SPI_Shift <= SPI_Out;                                    -- reload with next FIFO word
elsif (SPI_State = Shift_Addr or SPI_State = Shift_Data)
      and SPIBitCnt /= 0 and SPIDiv = 7 then
    SPI_Shift <= SPI_Shift(14 downto 0) & '0';              -- shift left: next bit to MSB
end if;


-- -------------------------------------------------------------------------
-- Bit counter: tracks how many bits remain in the current 16-bit word.
--   Loaded with 15 (X"F") whenever a new word enters the shift register
--   (at Load_Addr or when SPIBitCnt wraps to 0 in Shift_Addr/Data).
--   Decremented once per SPI clock cycle (SPIDiv = 7) during shifting.
--   Reaching 0 signals the last bit of the current word has been sent.
-- -------------------------------------------------------------------------
if (SPI_State = Load_Addr and SPIDiv = 7)
or ((SPI_State = Shift_Addr or SPI_State = Shift_Data)
    and SPIBitCnt = 0 and SPIDiv = 7) then
    SPIBitCnt <= X"F";                                       -- load for new 16-bit word
elsif (SPI_State = Shift_Addr or SPI_State = Shift_Data)
      and SPIDiv = 7 then
    SPIBitCnt <= SPIBitCnt - 1;                              -- count down remaining bits
else
    SPIBitCnt <= SPIBitCnt;                                  -- hold between prescaler ticks
end if;


-- -------------------------------------------------------------------------
-- SPICS (active low): asserted for the entire multi-word block transfer.
--   Driven low at the Load_Addr / SPIDiv = 7 boundary (one prescaler cycle
--   before the first clock edge) to meet the target device CS setup time.
--   Held low through Shift_Addr and all Shift_Data words.
--   De-asserted (high) only in Done, after the last data bit has been
--   clocked out, to satisfy the CS hold time requirement.
-- -------------------------------------------------------------------------
if SPICS = '1' and SPI_State = Load_Addr and SPIDiv = 7 then
    SPICS <= '0';                                            -- assert: start of transaction
elsif SPI_State = Done then
    SPICS <= '1';                                            -- de-assert: end of transaction
else
    SPICS <= SPICS;                                          -- hold during shifting
end if;


-- -------------------------------------------------------------------------
-- SPISClk: generated only during Shift_Addr and Shift_Data states.
--   Rises  at SPIDiv = 3 ? MOSI has been stable since SPIDiv = 7 of the
--           previous cycle (80 ns setup time at 50 MHz i50MHz).
--   Falls  at SPIDiv = 7 ? coincident with the shift register update and
--           SPIBitCnt decrement, so the next bit is valid on MOSI well
--           before the next rising edge (80 ns setup time).
--   Idle states (Idle, Load_Addr, Done): clock held low so no spurious
--   edges are seen by the target device while CS is de-asserted.
-- -------------------------------------------------------------------------
if SPISClk = '0' and SPIDiv = 3
   and (SPI_State = Shift_Addr or SPI_State = Shift_Data) then
    SPISClk <= '1';                                          -- rising edge: target samples MOSI
elsif SPISClk = '1' and SPIDiv = 7 then
    SPISClk <= '0';                                          -- falling edge: shift register advances
else
    SPISClk <= SPISClk;                                      -- hold between transitions
end if;

end if; -- CpldRst_sync

end process SMI_Proc;

-- Deterministic transmit gating: hold selected target for exactly 4 nibbles
-- =============================================================================
-- Process : Rx_active_cdc
-- Purpose : 8-channel 2-stage Clock-Domain Crossing (CDC) synchroniser
--           Safe transfer of the Rx_active bus (one bit per LVDS FM receiver
--           channel) from the SysClk (100 MHz) domain into the RxFMClk
--           (200 MHz) domain.
--
-- Why this is needed:
--   Rx_active(7:0) is an 8-bit flag vector driven by the 'main' process on
--   SysClk (100 MHz). Each bit indicates whether the corresponding LVDS FM
--   link (FEB channel 0-7) is currently active, based on transition counting
--   over a 10 µs observation window.
--
--   The PhyRx_Proc processes and the CRC generators run on RxFMClk (200 MHz).
--   Reading Rx_active directly inside an RxFMClk process would violate CDC
--   rules: because the two clocks are asynchronous and at different frequencies,
--   a bit change in Rx_active could be sampled during a metastable window,
--   corrupting the receive-path gating logic.
--
-- Direction of crossing:
--
--   SysClk domain (100 MHz)       RxFMClk domain (200 MHz)
--   ???????????????????????        ??????????????????????????????????????????
--   Rx_active(i)   ??? [FF0] ??? [FF1] ??? Rx_active_rxclk(i)
--                    _sync(i)(0) _sync(i)(1)  (safe to use in RxFMClk logic)
--
--   This structure is replicated independently for all 8 channels (i = 0..7).
--   Each channel has its own dedicated 2-FF chain, ensuring that a change on
--   one bit cannot cause a glitch on any other bit through shared logic.
--
-- Two-loop structure:
--   The synchroniser is intentionally split into two separate 'for' loops
--   rather than one combined loop. This is a common FPGA coding style that
--   helps synthesis and static timing tools (e.g. Vivado/ISE) correctly
--   identify and constrain each stage as a distinct synchroniser register,
--   allowing proper application of set_false_path or set_max_delay constraints.
--
--   Loop 1: advance the 2-FF pipeline for all 8 channels simultaneously
--   Loop 2: register the resolved output into a clean 8-bit bus for fan-out
--
-- Reset behaviour:
--   The reset condition is the logical OR of two independent faults:
--     (a) CpldRst_sync = '0' : board-level reset asserted
--     (b) PllLock     = '0'  : system PLL has lost lock (RxFMClk unreliable)
--
--   In either case all synchroniser stages and the output bus are cleared to '0'
--   (all channels inactive). This is the safe conservative default: the receive
--   path should not process any data when the clocking infrastructure is
--   unstable, and no channel should be reported as "active" during reset.
--
-- Timing example (100 MHz SysClk drives Rx_active, 200 MHz RxFMClk samples it):
--
--   SysClk       Rx_active(i) : ???????|_______________________
--   RxFMClk      _sync(i)(0)  : ??????????|____________________  (1 RxFMClk cycle)
--   RxFMClk      _sync(i)(1)  : ?????????????|_________________  (2 RxFMClk cycle)
--   RxFMClk      _rxclk(i)    : ????????????????|______________  (3 RxFMClk cycle)
--
--   Total worst-case latency = 3 RxFMClk cycles = 15 ns @ 200 MHz.
--   Rx_active is a slow control signal that only changes at 10 µs window
--   boundaries, so this latency is completely negligible.
--
-- Downstream consumer:
--   Rx_active_rxclk(i) is used inside PhyRx_Proc (clocked on RxFMClk)
--   to gate receive-side data processing per channel.
-- =============================================================================
Rx_active_cdc : process(RxFMClk)
begin
  if rising_edge(RxFMClk) then

    -- Synchronous reset: assert on board reset OR PLL loss-of-lock.
    -- Clearing to '0' (all channels inactive) is the safe default:
    -- no receive processing should occur when the clock is unreliable.
    if CpldRst_sync = '0' or PllLock = '0' then
      Rx_active_sync   <= (others => "00");  -- clear all 8x2-bit synchroniser stages
      Rx_active_rxclk  <= (others => '0');   -- clear all 8 output bits

    else

      -- Loop 1: advance the 2-stage synchroniser pipeline for all 8 channels.
      -- Each channel's FF chain is independent, preventing inter-channel glitches.
      for i in 0 to 7 loop

        -- Stage 0: capture Rx_active(i) from the SysClk domain.
        -- May go metastable if Rx_active(i) changes near the RxFMClk edge,
        -- but resolves within one RxFMClk period (5 ns @ 200 MHz).
        Rx_active_sync(i)(0) <= Rx_active(i);

        -- Stage 1: sample the resolved output of Stage 0.
        -- After this register, Rx_active_sync(i)(1) is fully safe to consume
        -- in the RxFMClk domain.
        Rx_active_sync(i)(1) <= Rx_active_sync(i)(0);

      end loop;

      -- Loop 2: register the synchronised outputs into a clean 8-bit bus.
      -- Separating this from Loop 1 reduces fan-out on _sync(i)(1) and
      -- helps timing tools apply the correct synchroniser path constraints.
      for i in 0 to 7 loop
        Rx_active_rxclk(i) <= Rx_active_sync(i)(1);
      end loop;

    end if;
  end if;
end process Rx_active_cdc;


-- =============================================================================
-- Process : TxEnReq_cdc
-- Purpose : 2-stage Clock-Domain Crossing (CDC) synchroniser
--           Safe transfer of TxEnReq from the SysClk (100 MHz) domain into
--           the i50MHz (50 MHz) domain.
--
-- Why this is needed:
--   TxEnReq is driven by the 'main' process on SysClk (100 MHz). It is
--   asserted when either:
--     (a) the microcontroller writes bit-0 = '1' to the PHY Tx CSR, or
--     (b) the AutoTx FSM completes writing a UBT packet into PhyTx_Buff.
--
--   TxEnReq is consumed by SMI_Proc which runs on i50MHz (50 MHz). Reading
--   TxEnReq directly inside SMI_Proc without synchronisation would violate
--   CDC rules: if TxEnReq changes near an i50MHz rising edge, the receiving
--   FF could enter a metastable state, causing TxEnAck to be asserted or
--   suppressed unpredictably and corrupting the PHY transmit handshake.
--
-- Direction of crossing:
--
--   SysClk domain (100 MHz)        i50MHz domain (50 MHz)
--   ???????????????????????         ??????????????????????????????????????
--   TxEnReq        ??? [FF0] ??? [FF1] ??? TxEnReq_sync(1)
--                    _sync(0)  _sync(1)     (safe to use in i50MHz logic)
--
--   FF0 (_sync(0)) : First capture register. Samples TxEnReq on each rising
--                    edge of i50MHz. May go metastable if TxEnReq changes
--                    near the clock edge, but resolves within one i50MHz
--                    period (20 ns), well inside Xilinx 7-series MTBF targets.
--
--   FF1 (_sync(1)) : Second capture register. Samples the (now resolved)
--                    output of FF0. After this register, TxEnReq_sync(1) is
--                    fully safe to consume anywhere in i50MHz logic.
--
-- Reset behaviour:
--   On CpldRst_sync = '0', both stages are cleared to '0'.
--   '0' = no transmit request pending ? the correct safe default.
--   This prevents SMI_Proc from asserting TxEnAck during or immediately
--   after reset before the PHY and FIFO infrastructure is ready.
--
-- Timing example (100 MHz SysClk drives TxEnReq, 50 MHz i50MHz samples it):
--
--   SysClk    TxEnReq      : _____|????????????????????|______
--   i50MHz    _sync(0)     : ________|????????????????????|___  (1 i50MHz cycle latency)
--   i50MHz    _sync(1)     : ____________|??????????????????|_  (2 i50MHz cycle latency)
--
--   Total worst-case latency = 2 i50MHz cycles = 40 ns @ 50 MHz.
--   This is acceptable: TxEnReq is a level signal held high until TxEnAck
--   is returned. A 40 ns delay has no functional impact on the handshake.
--
-- Handshake flow (after synchronisation):
--   SysClk domain                      i50MHz domain
--   ?????????????                       ?????????????
--   TxEnReq   asserted ??? [2-FF CDC] ??? TxEnReq_sync(1) seen high
--                                         TxEnAck asserted (in SMI_Proc)
--   TxEnReq   cleared  ??? TxEnAck seen in main process
--
-- Downstream consumer:
--   TxEnReq_sync(1) is read exclusively in SMI_Proc (i50MHz domain):
--     "if TxEnAck = '0' and TxEnReq_sync(1) = '1'
--         and PhyTxBuff_Empty = '0' then TxEnAck <= '1';"
--   It gates the TxEnAck response that enables the PHY transmit path.
-- =============================================================================

TxEnReq_cdc : process(i50MHz)
begin
  if rising_edge(i50MHz) then

    -- Synchronous reset: clear both stages to '0' (no transmit request).
    -- Prevents a spurious TxEnAck during reset or PLL startup.
    if CpldRst_sync = '0' then
      TxEnReq_sync <= "00";

    else

      -- Stage 0: capture TxEnReq from the SysClk domain.
      -- This FF may briefly enter metastability but resolves within
      -- one i50MHz period (20 ns @ 50 MHz).
      TxEnReq_sync(0) <= TxEnReq;

      -- Stage 1: sample the resolved output of Stage 0.
      -- TxEnReq_sync(1) is the safe i50MHz-domain copy of TxEnReq.
      -- Only this signal should ever be read inside i50MHz processes.
      TxEnReq_sync(1) <= TxEnReq_sync(0);

    end if;
  end if;
end process TxEnReq_cdc;



-- =============================================================================
-- Process : TxEnAck_cdc
-- Purpose : 2-FF synchroniser carrying TxEnAck from i50MHz into SysClk.
--
-- Source domain  : i50MHz (50 MHz) - TxEnAck is driven by SMI_Proc.
-- Dest. domain   : SysClk (100 MHz) - read by the main process (to clear
--                   TxEnReq) and by the uC readback mux (PhyTxCSRAddr).
--
-- Reading TxEnAck directly in SysClk logic without synchronisation would
-- violate CDC rules: TxEnAck can change near a SysClk rising edge, putting
-- the receiving FF into metastability and corrupting the TxEnReq/TxEnAck
-- handshake (TxEnReq could be cleared prematurely or not at all).
--
--   i50MHz domain                 SysClk domain (100 MHz)
--   --------------                ----------------------
--   TxEnAck   --> [FF0] --> [FF1] --> TxEnAck_sync(1)
--                _sync(0)  _sync(1)   (safe to use in SysClk logic)
--
-- Reset behaviour:
--   On CpldRst_sync = '0' both stages are cleared to '0' (no Ack pending),
--   matching the reset state of TxEnAck inside SMI_Proc.
-- =============================================================================
TxEnAck_cdc : process(SysClk)
begin
  if rising_edge(SysClk) then
    if CpldRst_sync = '0' then
      TxEnAck_sync <= "00";
    else
      -- Stage 0: first metastability capture FF.
      TxEnAck_sync(0) <= TxEnAck;
      -- Stage 1: resolved, SysClk-safe copy. Use this signal exclusively
      -- whenever TxEnAck is read inside a SysClk process or combinational
      -- output that is consumed by SysClk logic.
      TxEnAck_sync(1) <= TxEnAck_sync(0);
    end if;
  end if;
end process TxEnAck_cdc;



-- =============================================================================
-- Process : phy_out_gating
-- Purpose : Deterministic PHY transmit output routing at 25 MHz (MII rate)
--
-- Responsibilities:
--   1. Synchronise the SysClk-domain LastTxTarget clear strobe into i50MHz.
--   2. Determine which single PHY port should receive the current nibble
--      (one-hot target selection with priority: AutoTx > lowest TxEn bit).
--   3. Freeze the selected target for exactly 4 nibbles (one 16-bit FIFO
--      word) to guarantee a single consistent PHY drives all 4 nibbles of
--      every read word, even if TxEn or AutoTx_Target changes mid-word.
--   4. Update the LastTxTarget diagnostic register when a real transmission
--      begins.
--   5. Route TxReg (the current nibble from SMI_Proc) to exactly one PHY
--      output; all other PHY outputs are held at zero.
--
-- Clock structure:
--   The outer clock is i50MHz (50 MHz).
--   The LastTxTarget_clr_sync synchroniser (Section 1) runs at full i50MHz
--   rate so it never misses a clear strobe edge.
--   All PHY output logic (Sections 2-5) is gated behind Clk25MHz = '1',
--   making it execute on every OTHER i50MHz rising edge ? an effective
--   rate of 25 MHz, aligned to the MII nibble clock.
--
--   Why gate at 25 MHz?
--   The Ethernet MII interface clocks nibbles at 25 MHz (100BASE-TX).
--   Each PhyTxBuff FIFO word is 16 bits = 4 nibbles.  Gating the output
--   logic at 25 MHz ensures one and only one nibble is driven per MII
--   cycle, giving maximum setup/hold margin on the PHY Tx pins.
--
--   Clk25MHz is generated in SMI_Proc by toggling once per i50MHz cycle.
--   Checking Clk25MHz = '1' at rising_edge(i50MHz) samples the value
--   registered in the PREVIOUS cycle, so this gate fires on the i50MHz
--   edge that corresponds to the falling phase of the 25 MHz derived clock.
--
-- Variable declarations (process-local, not retained between cycles):
--   tgt_candidate : working one-hot vector; resolved target for this cycle.
--   lowest_mask   : temporary used when scanning TxEn for fallback target.
-- =============================================================================

phy_out_gating : process(i50MHz)
  variable tgt_candidate : std_logic_vector(7 downto 0);
  variable lowest_mask   : std_logic_vector(7 downto 0);
begin
  if rising_edge(i50MHz) then

    -- -------------------------------------------------------------------------
    -- Section 1: CDC synchroniser for LastTxTarget clear strobe
    --            Runs at FULL i50MHz rate (outside the Clk25MHz gate) to
    --            ensure the edge is never missed between 25 MHz windows.
    --
    -- Source   : LastTxTarget_clr_stretch(0) ? a 5-cycle stretched pulse
    --            driven by SysClk-domain logic whenever the µC writes to
    --            LastTxTargetAddr. Stretching guarantees the pulse is wide
    --            enough for the i50MHz synchroniser to reliably capture it.
    --
    -- Pipeline : 3-stage shift register
    --   _sync(0) : Stage 0 ? first metastability capture FF
    --   _sync(1) : Stage 1 ? second metastability capture FF (resolved)
    --   _sync(2) : Stage 2 ? one-cycle-delayed copy of Stage 1,
    --              used as the "previous" value for rising-edge detection.
    --
    -- Edge detect (used in Section 3 below):
    --   Rising edge on _sync(1) detected when:
    --   _sync(2) = '0'  (was low last 25 MHz cycle) AND
    --   _sync(1) = '1'  (is high this 25 MHz cycle)
    -- -------------------------------------------------------------------------
    LastTxTarget_clr_sync <=
        LastTxTarget_clr_sync(1 downto 0) & LastTxTarget_clr_stretch(0);

	 -- Section 1b: handle the clear at full i50MHz rate so we never
    -- miss it when the 25 MHz gate happens to be open on a rdreq cycle.
    if LastTxTarget_clr_sync(2) = '0' and LastTxTarget_clr_sync(1) = '1' then
      LastTxTarget <= (others => '0');
    end if;

    -- =========================================================================
    -- All logic below is gated at 25 MHz (one execution per MII nibble period)
    -- =========================================================================
    if Clk25MHz = '1' then

      -- -----------------------------------------------------------------------
      -- Section 2: One-hot transmit target selection
      --
      -- Priority order (highest first):
      --   (a) AutoTx_Target ? set by AutoTx_Proc when the FSM has chosen a
      --       specific port for a UBT packet. Takes priority over any µC-driven
      --       TxEn bits so that automatic handshake traffic always reaches the
      --       intended FEB.
      --   (b) Lowest-indexed set bit in TxEn ? fallback for µC-driven
      --       transmissions where no AutoTx port is active. Selecting only the
      --       lowest bit prevents multi-port simultaneous drive even if the µC
      --       accidentally sets multiple TxEn bits.
      --
      -- Result stored in variable tgt_candidate; used in Sections 3 and 4.
      -- -----------------------------------------------------------------------
      if not is_all_zero(AutoTx_Target) then
        -- AutoTx has claimed a port: use its one-hot mask directly.
        tgt_candidate := AutoTx_Target;
      else
        -- No AutoTx claim: scan TxEn and pick only the lowest-indexed '1' bit.
        -- The 'exit' statement ensures at most one bit is set in lowest_mask.
        lowest_mask := ZERO8;
        for i in 0 to 7 loop
          if TxEn(i) = '1' then
            lowest_mask(i) := '1';
            exit;
          end if;
        end loop;
        tgt_candidate := lowest_mask;
      end if;


      -- -----------------------------------------------------------------------
      -- Section 3: FIFO read ? freeze target and update diagnostics
      --
      -- Fires when SMI_Proc has issued a PhyTxBuff_rdreq (NibbleCount = 2,
      -- Clk25MHz = '0' in SMI_Proc, which means we see rdreq one i50MHz cycle
      -- later here).
      --
      -- On each read:
      --   TxTarget_hold  : latches tgt_candidate for exactly 4 nibbles so the
      --                    entire 16-bit word is steered to one consistent port.
      --   nibble_hold_cnt: countdown from 4 to 0; CurrentTarget follows
      --                    TxTarget_hold while non-zero (Section 4).
      --   LastTxTarget   : diagnostic register ? records which PHY last received
      --                    a FIFO word. Updated only when BOTH the candidate AND
      --                    TxEn are non-zero: this prevents LastTxTarget from
      --                    being latched when AutoTx_Target has been set but
      --                    TxEnMask has not yet propagated through the CDC path
      --                    (i50MHz domain), which would make the diagnostic
      --                    register report "UBT fired" when nothing was actually
      --                    driven onto the wire.
      -- -----------------------------------------------------------------------
		-- Inside phy_out_gating, replace Section 3 + Section 3b with:

-- Use the LEVEL of the synchronised stretch pulse (not just the rising edge)
-- so LastTxTarget cannot be re-latched anywhere inside the µC's clear
-- window. The stretch is 5 SysClk cycles ? this ends up being ~2 i50MHz
-- cycles where _sync(1) is high, which fully covers the µC read-back path.
if LastTxTarget_clr_sync(1) = '1' then
  LastTxTarget <= (others => '0');
  if PhyTxBuff_rdreq = '1' then
    TxTarget_hold   <= tgt_candidate;
    nibble_hold_cnt <= 4;
    -- DELIBERATELY do NOT update LastTxTarget here.
  end if;
elsif PhyTxBuff_rdreq = '1' then
  TxTarget_hold <= tgt_candidate;
  if tgt_candidate /= ZERO8 and TxEn /= ZERO8 then
    LastTxTarget <= tgt_candidate;
  end if;
  nibble_hold_cnt <= 4;
end if;


      -- -----------------------------------------------------------------------
      -- Section 4: CurrentTarget ? 4-nibble hold window
      --
      -- Holds TxTarget_hold as CurrentTarget for exactly 4 consecutive 25 MHz
      -- cycles (one full 16-bit FIFO word). This guarantees all 4 nibbles of a
      -- single read are routed to the same PHY even if tgt_candidate changes
      -- partway through the word (e.g. TxEn cleared or AutoTx_Target updated).
      --
      -- When the hold window expires (nibble_hold_cnt reaches 0):
      --   - TxTarget_hold is cleared to ZERO8 on the last active cycle.
      --   - CurrentTarget falls back to the live tgt_candidate value.
      -- -----------------------------------------------------------------------
      if nibble_hold_cnt > 0 then
        CurrentTarget   <= TxTarget_hold;
        nibble_hold_cnt <= nibble_hold_cnt - 1;
        if nibble_hold_cnt = 1 then
          TxTarget_hold <= ZERO8;   -- pre-clear ready for next read
        end if;
      else
        CurrentTarget <= tgt_candidate;   -- no hold active: follow live target
      end if;


      -- -----------------------------------------------------------------------
      -- Section 5: PHY nibble output routing
      --
      -- Routes TxReg (the 4-bit nibble selected by SMI_Proc's nibble mux) to
      -- exactly the PHY(s) whose bit in CurrentTarget is '1'.
      -- All other PHY Tx outputs are driven to "0000" to prevent crosstalk or
      -- accidental transmission on idle ports.
      --
      -- In normal operation CurrentTarget is one-hot (exactly one bit set),
      -- so exactly one PhyTx(i) carries live data at any time.
      -- -----------------------------------------------------------------------
      for i in 0 to 7 loop
        if CurrentTarget(i) = '1' then
          PhyTx(i) <= TxReg;    -- forward nibble to selected PHY
        else
          PhyTx(i) <= ZERO4;    -- idle: hold all Tx pins low
        end if;
      end loop;

    end if;  -- Clk25MHz = '1'  (25 MHz gate)

  end if;  -- rising_edge(i50MHz)
end process phy_out_gating;


-- Drives the MOSI pin directly from the MSB of the 16-bit SPI shift register,
-- implementing MSB-first serial transmission to the LVDS clock fanout chip.
-- Each left-rotation of SPI_Shift in SMI_Proc promotes the next data bit into
-- position 15, so this concurrent assignment always presents the correct bit
-- to the pin without an additional output register.
SPIMOSI <= SPI_Shift(15);

-- =============================================================================
-- Process : LastTxTarget_clr_stretching
-- Purpose : Pulse stretcher ? widens the single-cycle SysClk clear request
--           for LastTxTarget into a 5-cycle pulse before it crosses into the
--           i50MHz clock domain.
--
-- The µC asserts LastTxTarget_clr_req for exactly one SysClk cycle (10 ns)
-- when it writes to LastTxTargetAddr. A pulse that narrow cannot be reliably
-- captured by the 3-stage synchroniser in phy_out_gating, which samples on
-- i50MHz (50 MHz, 20 ns period): the pulse could arrive and disappear entirely
-- between two i50MHz edges and be missed completely. To guarantee capture, the
-- 5-bit register LastTxTarget_clr_stretch is loaded with "11111" on the
-- request cycle and then shifted right by one position each SysClk cycle,
-- inserting '0' from the MSB. This produces a clean 5-cycle (50 ns) active
-- pulse on bit 0 ? the output fed to the synchroniser ? which is wide enough
-- to span at least two i50MHz rising edges regardless of the phase relationship
-- between the two clocks, satisfying the minimum pulse-width requirement of
-- the 3-FF CDC chain in phy_out_gating.
-- =============================================================================
LastTxTarget_clr_stretching : process(SysClk)
begin
    if rising_edge(SysClk) then

        -- Synchronous reset: clear the stretcher so no stale pulse
        -- propagates into the i50MHz domain after a system reset.
        if CpldRst_sync = '0' then
            LastTxTarget_clr_stretch <= "00000";

        -- Request asserted: load all ones to begin the 5-cycle pulse.
        elsif LastTxTarget_clr_req = '1' then
            LastTxTarget_clr_stretch <= "11111";

        -- No request: shift right each cycle, inserting '0' at the MSB.
        -- Bit 0 remains '1' for 5 consecutive cycles then naturally falls
        -- to '0' once the last '1' has shifted out of the register.
        else
            LastTxTarget_clr_stretch <= '0' & LastTxTarget_clr_stretch(4 downto 1);

        end if;
    end if;
end process;
         

-- =============================================================================
-- Process : TxEnMask_cdc
-- Purpose : 2-stage CDC synchroniser ? transfers the one-hot PHY transmit
--           lane mask from the SysClk (100 MHz) domain into the i50MHz
--           (50 MHz) domain ahead of each UBT packet transmission.
--
-- TxEnMask_next is a one-hot 8-bit vector driven by AutoTx_Proc on SysClk.
-- It is loaded with AutoTx_Target (the port selected for the current UBT
-- handshake) once per AutoTx cycle, and held stable for the entire duration
-- of the packet transmission. Because it must be consumed by SMI_Proc
-- (i50MHz domain) at the moment PreambleTx rises ? to load TxEnMask and
-- thereby gate TxEn onto the correct PHY ? it must be safely transferred
-- across the clock boundary before that event occurs. The two-stage pipeline
-- (_sync then _i50) provides the standard metastability barrier: the first
-- register captures the SysClk-domain value and may briefly be metastable,
-- while the second register samples the resolved output and presents a clean
-- signal to all i50MHz consumers. No explicit reset branch is required here
-- because TxEnMask_next is already cleared to X"00" by AutoTx_Proc on
-- CpldRst_sync = '0', so the synchroniser will naturally propagate the safe
-- all-zeros (no PHY enabled) value into i50MHz logic within two cycles of
-- any reset event.
-- =============================================================================
TxEnMask_cdc : process(i50MHz)
begin
    if rising_edge(i50MHz) then

        -- Stage 1: capture TxEnMask_next from the SysClk domain.
        -- May be briefly metastable but resolves within one i50MHz period (20 ns).
        TxEnMask_sync <= TxEnMask_next;

        -- Stage 2: sample the resolved output of Stage 1.
        -- TxEnMask_i50 is the safe i50MHz-domain copy, read by SMI_Proc
        -- to load TxEnMask at the start of each preamble transmission.
        TxEnMask_i50  <= TxEnMask_sync;

    end if;
end process;

-- synthesis translate_off
AutoTx_Inhibit_int <= probe_AutoTx_Inhibit;
-- synthesis translate_on

-- =============================================================================
-- Process : AutoTx_Proc
-- Purpose : Autonomous UBT (µBunch Transfer) handshake sequencer
--
-- Overview:
--   This FSM automatically manages the full request-response cycle between
--   FPGA2 and each Front-End Board (FEB) connected via Ethernet PHY. When a
--   FEB port is marked ready in ReadyStatus, the FSM:
--     1. Claims the port and writes a UBT packet into the PhyTx FIFO.
--     2. Waits for the FIFO contents to propagate to the i50MHz read side.
--     3. Triggers SMI_Proc to transmit the packet by pulsing TxEnReq.
--     4. Waits for the packet to drain completely from the FIFO.
--     5. Waits for the FEB to respond (PHY Rx FIFO goes non-empty).
--     6. Waits for the DDR write sequencer to consume the FEB response.
--     7. Re-arms the port in ReadyStatus so the next round-robin iteration
--        can service it again.
--   If any stage exceeds its timeout the port is released, AutoTx_TimedOut
--   is flagged, and the FSM returns to idle for a retry on the next scan.
--
-- Round-robin arbitration:
--   AT_Idle scans ReadyStatus starting one position past RoundRobin_Last,
--   wrapping modulo 8. The first port that is both ready (ReadyStatus = '1')
--   and enabled (MaskReg = '1') is selected. This ensures no single port
--   monopolises the bus when multiple FEBs are active simultaneously.
--
-- Clock domain interactions:
--   The entire process runs on SysClk (100 MHz). Several signals cross into
--   the i50MHz domain (50 MHz) and require explicit CDC handling:
--     TxEnMask_next  -> TxEnMask_cdc  -> TxEnMask_i50   (lane mask for SMI_Proc)
--     AutoTx_TxEnReqPulse -> main     -> TxEnReq        (transmit enable request)
--     PhyTxBuff_Empty (i50MHz) -> PhyTxEmpty_sync_proc -> PhyTxBuff_Empty_s
--   The FSM always waits for PhyTxBuff_Empty_s (the synchronised empty flag)
--   rather than reading PhyTxBuff_Empty directly, preventing metastability
--   on all FIFO-state decisions.
--
-- Default signal assignments (applied every rising SysClk edge before the
-- state machine, so any state that does not explicitly override them sees
-- the safe pulsed-low / cleared value):
--   AutoTx_Claim        : cleared to X"00"  (one-shot port claim pulse)
--   AutoTx_FifoRst_req  : cleared to '0'    (one-shot FIFO reset request)
--   AutoTx_ReArm        : cleared to X"00"  (one-shot re-arm pulse)
--   PhyTxWrReq_FPGA     : cleared to '0'    (one-shot FIFO write strobe)
--   AutoTx_TxEnReqPulse : cleared to '0'    (one-shot TxEnReq trigger)
--   TxEnMask_next       : held at AutoTx_Target (stable during transmission)
--
-- Variable declarations (resolved combinatorially each cycle, not retained):
--   found_port : index of the next round-robin candidate port (0-7)
--   have_port  : true when a valid candidate was found in the scan
--   onehot     : temporary one-hot encoding of found_port
--
-- State machine summary:
--   AT_Idle        : scan ReadyStatus round-robin; claim first ready+masked port
--   AT_WriteWords  : write UBT_ASC_COUNT words into PhyTxBuff one per cycle
--   AT_WaitTxFill  : wait for PhyTxBuff_Empty_s to fall (CDC settling, ~3 cycles)
--   AT_WaitTxDrain : pulse TxEnReq; wait for SMI_Proc to drain the FIFO
--   AT_WaitRxFill  : wait for FEB to respond (PhyRxBuff goes non-empty)
--   AT_WaitDdrDrain: wait for DDR write sequencer to drain the Rx FIFO
-- =============================================================================

AutoTx_Proc : process(SysClk, CpldRst_sync)
    variable found_port : integer range 0 to 7;
    variable have_port  : boolean;
    variable onehot     : std_logic_vector(7 downto 0);
begin

  -- ---------------------------------------------------------------------------
  -- Asynchronous reset: return all outputs and FSM state to safe defaults.
  -- TxEnMask_next cleared to X"00" ensures no PHY lane is enabled before
  -- the system is fully initialised. AutoTx_Busy cleared so the main process
  -- does not see any port as in-flight across a reset boundary.
  -- ---------------------------------------------------------------------------
  if CpldRst_sync = '0' then
    PhyTxDin_FPGA       <= (others => '0');
    PhyTxWrReq_FPGA     <= '0';
    AutoTx_State        <= AT_Idle;
    AutoTx_Port         <= 0;
    AutoTx_WordIdx      <= 0;
    AutoTx_Claim        <= (others => '0');
    AutoTx_ReArm        <= (others => '0');
    AutoTx_Busy         <= (others => '0');
    AutoTx_Target       <= (others => '0');
    AutoTx_TxEnReqPulse <= '0';
    AutoTx_TimedOut     <= (others => '0');
    AutoTx_WaitTimeout  <= 0;
    RxFilled_sticky     <= '0';
    TxEnMask_next       <= (others => '0');
    RoundRobin_Last     <= 0;
    AutoTx_FifoRst_req  <= '0';
	 AutoTx_RxFlush <= (others => '0');

  elsif rising_edge(SysClk) then

    -- -------------------------------------------------------------------------
    -- Default pulse clears: these signals are one-shot strobes. Clearing them
    -- here means each state only needs to assert them for a single cycle;
    -- they are automatically de-asserted the following cycle without any
    -- explicit else branch in the state machine.
    -- -------------------------------------------------------------------------
    AutoTx_Claim        <= X"00";
    AutoTx_FifoRst_req  <= '0';
    AutoTx_ReArm        <= (others => '0');
    PhyTxWrReq_FPGA     <= '0';
    AutoTx_TxEnReqPulse <= '0';

    -- Keep TxEnMask_next stable at the currently selected target throughout
    -- the entire handshake cycle. TxEnMask_cdc transfers this into i50MHz
    -- domain within 2 cycles so SMI_Proc always sees the correct lane mask.
    TxEnMask_next <= AutoTx_Target;

    -- -------------------------------------------------------------------------
    -- RxFilled_sticky: latches a PhyRxBuff fill event while the FSM is in
    -- AT_WaitRxFill. Used as a fallback in case the fill edge is missed on
    -- the exact cycle the state machine checks PhyRxBuff_Empty. Cleared on
    -- state exit and on reset.
    -- -------------------------------------------------------------------------
    if AutoTx_State = AT_WaitRxFill
       and PhyRxFilled(AutoTx_Port) = '1' then
        RxFilled_sticky <= '1';
    end if;


    case AutoTx_State is

      -- -----------------------------------------------------------------------
      -- AT_Idle: scan ReadyStatus for the next port to service.
      --
      -- Performs a round-robin scan starting one position past RoundRobin_Last
      -- so that no port is permanently prioritised over another. The loop exits
      -- as soon as it finds a port that is both ready (ReadyStatus = '1') and
      -- enabled by the channel mask (MaskReg = '1').
      --
      -- The FSM only leaves Idle when ALL of the following are true:
      --   (a) A valid candidate port was found.
      --   (b) PhyTxBuff_Empty_s = '1': the transmit FIFO is empty and safe to
      --       write into without corrupting a preceding packet.
      --   (c) PhyTxBuff_Full  = '0':  the FIFO has space for a new packet.
      --   (d) PhyTxBuff_wreq  = '0':  the µC is not simultaneously writing to
      --       the FIFO, preventing arbitration conflicts on the write port.
      --
      -- On departure:
      --   AutoTx_Claim(found_port) pulsed for one cycle so the main process
      --   can clear ReadyStatus(found_port) one cycle later, after the FSM has
      --   already observed and acted on the set bit.
      --   AutoTx_Busy(found_port) held high for the entire handshake cycle to
      --   prevent re-entry for the same port until the cycle completes.
      -- -----------------------------------------------------------------------
      when AT_Idle =>
          have_port := false;
          for i in 0 to 7 loop
              if ReadyStatus((RoundRobin_Last+1+i) mod 8) = '1'
                 and MaskReg((RoundRobin_Last+1+i) mod 8) = '1' then
                  found_port := (RoundRobin_Last+1+i) mod 8;
                  have_port  := true;
                  exit;
              end if;
          end loop;

          if have_port
             and PhyTxBuff_Empty_s = '1'
             and PhyTxBuff_Full    = '0'
             and PhyTxBuff_wreq    = '0'
				 and AutoTx_Inhibit_int = '0' then  -- TB hold-off
              AutoTx_Port              <= found_port;
              RoundRobin_Last          <= found_port;
              AutoTx_WordIdx           <= 0;
              AutoTx_Claim(found_port) <= '1';       -- one-shot: clears ReadyStatus next cycle
              AutoTx_Busy(found_port)  <= '1';       -- hold until full cycle completes
              onehot                   := (others => '0');
              onehot(found_port)       := '1';
              AutoTx_Target            <= onehot;    -- one-hot lane selection
              TxEnMask_next            <= onehot;    -- pre-load CDC register immediately
				  AutoTx_RxFlush <= (others => '0');
              AutoTx_State             <= AT_WriteWords;
          end if;


      -- -----------------------------------------------------------------------
      -- AT_WriteWords: write the UBT packet into PhyTxBuff one word per cycle.
      --
      -- ubt_ascii_word(index, '1') returns the pre-computed 16-bit packet word
      -- for the given word index. Words are written sequentially from index 0
      -- to UBT_ASC_COUNT-1. PhyTxWrReq_FPGA is gated to '0' by the default
      -- assignment above, so it is naturally a single-cycle strobe each
      -- iteration without an explicit clear.
      --
      -- The guard (PhyTxBuff_Full = '0' and PhyTxWrReq_FPGA = '0') prevents a
      -- write when the FIFO is full or when the previous write strobe is still
      -- being processed, ensuring no words are dropped.
      --
      -- On the last word (AutoTx_WordIdx = UBT_ASC_COUNT - 1):
      --   TxEnReq is NOT raised here. PhyTx_Buff is a true dual-clock FIFO
      --   (wr_clk = SysClk, rd_clk = i50MHz); after the final write the i50MHz
      --   read-side Empty flag requires approximately 3 i50MHz cycles to
      --   de-assert. Raising TxEnReq before Empty falls would cause SMI_Proc
      --   to miss the enable entirely. Instead, the FSM transitions to
      --   AT_WaitTxFill and waits for the synchronised empty flag to confirm
      --   the FIFO contents are visible on the read side.
      -- -----------------------------------------------------------------------
      when AT_WriteWords =>
          if PhyTxBuff_Full = '0' and PhyTxWrReq_FPGA = '0' then
              PhyTxDin_FPGA   <= ubt_ascii_word(AutoTx_WordIdx, '1');
              PhyTxWrReq_FPGA <= '1';
              if AutoTx_WordIdx >= UBT_ASC_COUNT - 1 then
                  AutoTx_WordIdx     <= 0;
                  AutoTx_WaitTimeout <= 1000;          -- 1 ms CDC settling timeout
                  AutoTx_State       <= AT_WaitTxFill;
              else
                  AutoTx_WordIdx <= AutoTx_WordIdx + 1;
              end if;
          end if;


      -- -----------------------------------------------------------------------
      -- AT_WaitTxFill: wait for the FIFO write to become visible on the read side.
      --
      -- Holds until PhyTxBuff_Empty_s (the 3-stage SysClk-domain synchronised
      -- version of the i50MHz-domain empty flag) falls to '0', confirming that
      -- SMI_Proc can now see data waiting in the FIFO. Only at that point is
      -- AutoTx_TxEnReqPulse asserted to trigger the TxEnReq handshake in main,
      -- which in turn causes SMI_Proc to assert TxEnAck and begin transmission.
      --
      -- Timeout (1000 x 1 µs = 1 ms):
      --   If the FIFO never goes non-empty the CDC path or FIFO is wedged.
      --   AutoTx_FifoRst_req is asserted to force a FIFO reset via the
      --   PhyTxFifoRst_stretch mechanism, the port is released, and the FSM
      --   returns to idle. The port is NOT re-armed: a persistent fill failure
      --   suggests a hardware fault that should be surfaced to the µC via the
      --   timeout counter rather than silently retried.
      -- -----------------------------------------------------------------------
      when AT_WaitTxFill =>
          if PhyTxBuff_Empty_s = '0' then
              AutoTx_TxEnReqPulse <= '1';              -- triggers TxEnReq in main
              AutoTx_WaitTimeout  <= 1000;             -- 1 ms transmit drain timeout
              AutoTx_State        <= AT_WaitTxDrain;
          elsif AutoTx_WaitTimeout > 0 then
              if Counter1us = X"00" then
                  AutoTx_WaitTimeout <= AutoTx_WaitTimeout - 1;
              end if;
          else
              AutoTx_TimedOut(AutoTx_Port) <= '1';
              AutoTx_Busy(AutoTx_Port)     <= '0';
              AutoTx_FifoRst_req           <= '1';     -- attempt FIFO recovery
              AutoTx_State                 <= AT_Idle;
          end if;


      -- -----------------------------------------------------------------------
      -- AT_WaitTxDrain: wait for SMI_Proc to fully transmit the UBT packet.
      --
      -- Holds until PhyTxBuff_Empty_s rises back to '1', confirming that the
      -- i50MHz read side has consumed all words and the packet has been fully
      -- clocked out to the PHY. RxFilled_sticky is cleared here so the
      -- subsequent AT_WaitRxFill state starts with a clean latch.
      --
      -- Timeout (1000 x 1 µs = 1 ms):
      --   If the FIFO does not drain (e.g. SMI_Proc is stalled or TxEnAck was
      --   never asserted), AutoTx_FifoRst_req resets the FIFO, the port is
      --   released, AutoTx_ReArm re-queues the port for a retry on the next
      --   idle scan, and AutoTx_TimedOut is flagged for the µC.
      -- -----------------------------------------------------------------------
      when AT_WaitTxDrain =>
          if PhyTxBuff_Empty_s = '1' then
              RxFilled_sticky    <= '0';
              AutoTx_WaitTimeout <= 10000;             -- 10 ms FEB reply timeout
              AutoTx_State       <= AT_WaitRxFill;
          elsif AutoTx_WaitTimeout > 0 then
              if Counter1us = X"00" then
                  AutoTx_WaitTimeout <= AutoTx_WaitTimeout - 1;
              end if;
          else
              AutoTx_TimedOut(AutoTx_Port) <= '1';
              AutoTx_Busy(AutoTx_Port)     <= '0';
              AutoTx_FifoRst_req           <= '1';     -- attempt FIFO recovery
              AutoTx_ReArm(AutoTx_Port)    <= '1';     -- retry this port next idle scan
              AutoTx_State                 <= AT_Idle;
          end if;


      -- -----------------------------------------------------------------------
      -- AT_WaitRxFill: wait for the FEB to respond to the UBT packet.
      --
      -- The FEB sends its reply over the Ethernet link. The FPGA receives it
      -- in PhyRx_Proc, which writes the incoming words into PhyRxBuff. This
      -- state monitors PhyRxBuff_Empty(AutoTx_Port) directly: the moment it
      -- falls to '0', data has arrived and the FSM advances.
      -- PhyRxBuff is clocked on SysClk (rd_clk = SysClk) so this read is
      -- safe without an additional synchroniser.
      --
      -- Timeout (10000 x 1 µs = 10 ms):
      --   If no reply arrives within 10 ms the FEB is assumed unresponsive.
      --   AutoTx_ReArm re-queues the port so the next idle scan will retry,
      --   and AutoTx_TimedOut is flagged for diagnostic visibility.
      -- -----------------------------------------------------------------------
--      when AT_WaitRxFill =>
--          if PhyRxBuff_Empty(AutoTx_Port) = '0' then
--              RxFilled_sticky    <= '0';
--              AutoTx_WaitTimeout <= 10000;             -- 10 ms DDR drain timeout
--              AutoTx_State       <= AT_WaitDdrDrain;
--          elsif AutoTx_WaitTimeout > 0 then
--              if Counter1us = X"00" then
--                  AutoTx_WaitTimeout <= AutoTx_WaitTimeout - 1;
--              end if;
--          else
--              AutoTx_TimedOut(AutoTx_Port) <= '1';
--              AutoTx_Busy(AutoTx_Port)     <= '0';
--              AutoTx_ReArm(AutoTx_Port)    <= '1';     -- retry this port next idle scan
--              AutoTx_State                 <= AT_Idle;
--          end if;

		when AT_WaitRxFill =>
    if PhyRxBuff_Empty(AutoTx_Port) = '0' then
        -- FEB has replied; that's all we need to re-arm.
        -- Flush the per-port Rx FIFO so the DDR sequencer (or anything else)
        -- starts clean next cycle, and immediately release + re-arm the port.
        AutoTx_RxFlush(AutoTx_Port) <= '1';
        AutoTx_Busy   (AutoTx_Port) <= '0';
        AutoTx_ReArm  (AutoTx_Port) <= '1';
        AutoTx_State                <= AT_Idle;
    elsif AutoTx_WaitTimeout > 0 then
        if Counter1us = X"00" then
            AutoTx_WaitTimeout <= AutoTx_WaitTimeout - 1;
        end if;
    else
        AutoTx_TimedOut(AutoTx_Port) <= '1';
        AutoTx_Busy   (AutoTx_Port)  <= '0';
        AutoTx_ReArm  (AutoTx_Port)  <= '1';
        AutoTx_State                 <= AT_Idle;
    end if;



      -- -----------------------------------------------------------------------
      -- AT_WaitDdrDrain: wait for the DDR write sequencer to consume the reply.
      --
      -- The DDR_Write_Seq FSM in main reads PhyRxBuff(AutoTx_Port) and writes
      -- the FEB response data to LPDDR. This state waits until PhyRxBuff_Empty
      -- returns to '1', confirming that all reply words have been transferred
      -- to DDR and the receive FIFO is fully drained. Only then is the port
      -- released (AutoTx_Busy cleared) and AutoTx_ReArm pulsed to prime the
      -- same port for its next round-robin turn.
      --
      -- Timeout (10000 x 1 µs = 10 ms):
      --   If the DDR write sequencer stalls and never drains the FIFO within
      --   10 ms, the port is released and AutoTx_TimedOut is flagged. The port
      --   is NOT re-armed on this timeout path: a DDR stall is a systemic fault
      --   (not a per-port issue) and immediate retry would simply queue up
      --   more unprocessed data. The µC can inspect AutoTx_TimedOut and
      --   intervene via ReadyForceAddr if a manual retry is desired.
      -- -----------------------------------------------------------------------
--		when AT_WaitDdrDrain =>
--			if PhyRxBuff_Empty(AutoTx_Port) = '1' then
--				AutoTx_Busy(AutoTx_Port)  <= '0';
--				AutoTx_ReArm(AutoTx_Port) <= '1';
--				AutoTx_State              <= AT_Idle;
--			elsif AutoTx_WaitTimeout > 0 then
--				if Counter1us = X"00" then
--					AutoTx_WaitTimeout <= AutoTx_WaitTimeout - 1;
--				end if;
--			else
--				-- Recovery path: malformed FEB reply (RdCnt < header word count).
--				-- Flush the offending port's PhyRxBuff so the next cycle is clean,
--				-- mark the timeout, release the port, AND re-arm so traffic resumes.
--				AutoTx_TimedOut(AutoTx_Port) <= '1';
--				AutoTx_RxFlush(AutoTx_Port)  <= '1';   -- one-shot per-port flush
--				AutoTx_Busy(AutoTx_Port)     <= '0';
--				AutoTx_ReArm(AutoTx_Port)    <= '1';   -- <-- currently missing; without this the port goes dark
--				AutoTx_State                 <= AT_Idle;
--			end if;

    end case;
  end if;
end process AutoTx_Proc;


----------------------- 100 Mhz clocked logic -----------------------------

main : process(SysClk, CpldRst_sync)
variable rs_next : std_logic_vector(7 downto 0);
 begin 

-- reset/preset
 if CpldRst_sync = '0' then
-- Synchronous edge detectors for various strobes
	RDDL <= "00"; WRDL <= "00"; PortNo <= 0;
-- Upper DRAM word staging register
	CDStage <= (others => '0'); 
-- Control bits written by the uC
	EventWdCnt <= (others => '0'); EventStat <= (others => '0'); 
	PortWdCounter <= (others => (others => '0')); 
	SDWrtAd <= (others => '0'); SDWrtAdStage <= (others => '0'); 
	SDRdAD <= (others => '0'); SDRdPtr <= (others => '0'); 
   SDWrtCmd <= "000"; WrtCmdEn <= '0'; SDrd_en <= '0'; SDrd_enD <= "000";
	SDRdCmd <= "000";  SDRdCmdEn <= '0'; RdHi_LoSel <= '0'; EventRdy <= '0';
   SDWrtDat <= (others => '0'); SDwr_en <= '0'; WrtHi_LoSel <= '0';
	UpTimeStage <= (others => '0'); UpTimeCount <= (others => '0');
	Counter1us <= X"00"; Counter10us <= (others => '0'); Counter1ms <= (others => '0'); 
	Counter1s <= (others => '0');	TestCount <= (others => '0'); 
	ResetCount <= (others => '0'); TrigWdCntRst <= '0';
	DDR_Reset <= '0';	DDR_Write_Seq <= Idle; DDR_Read_Seq <= Idle; 
   AddrBuff_wren <= '0'; AddrBuff_rden <= '0'; WrtAddrReg <= (others => '0');
	Seq_Busy <= '0'; EvWdCount <= (others => '0');  TxBlkCount <= "000"; DRegSrc <= '0';
	ReadCount <= "000"; MaskReg <= X"FF"; FirstActive <= '0';
	DDRRd_en <= '0'; DDRWrt_En <= '0'; DDRRd_EnD <= '0'; DDRWrt_EnD <= '0'; WaitCount <= (others => '0');
	LinkTxWrReq <= '0'; LinkTxTraceWrReq <= '0'; DatReqBuff_rdreq <= '0'; Rx_active <= X"00";
	SMI_wreq <= '0'; ChainSel <= "11"; PhyDatSel <= '0'; InitReq <= '0';
	PhyTxBuff_wreq <= '0'; TrigWdCount <= X"0"; MDIORd <= '0'; PhyPDn <= '0'; PhyRst <= '0';
	TxEnReq <= '0'; LinkTxRDReq <= '0'; TxValid <= '0'; 
	TrigReqCount <= X"00"; Link_Stat_Req <= '0'; HitFlag <= X"00";
	PhyRstCnt <= "11"; FMRxEn <= '0'; PhyRxBuff_RdStat <= X"00"; 
	PhyRxBuff_rdreq <= X"00"; RxBuffRst <= '0'; SPI_WrtReq <= '0'; DDRWrtStat <= X"0";
	ClockReg <= "10101"; -- initial clock pattern
	FrameReg <= "11111"; -- initial framing pattern
	RxDl <= (others => "00"); TransitionCount <= (others => X"0"); 
	DReqFMDL <= X"0"; LinkFIFOStat <= '0';
	LinkStatEn <= '1';
	LinkTxFullCnt <= X"00";
	tx_overflow <= '0';
	tx_overflow_cnt <= (others => '0');
	word_number <= (others => '0');
	EvWdCountTot <= (others => '0');
	ReadyStatus <= (others => '0');
	phy_empty_d <= (others => (others => '1')); 
        LastTxTarget_clr_req <= '0';
	TransitionCount    <= (others => X"0");
	DeadWindowCount    <= (others => X"0");
	PowerOnReady_done <= '0';
        StartupHoldoff    <= (others => '0');
	AutoTx_Claim_d <= (others => '0');
	PhyRst_AutoDone <= '0';	 
	AutoTx_TxEnReqHold <= '0';
	AutoTx_TimedOut_d  <= (others => '0');
   for p in 0 to 7 loop
    AutoTx_TimeoutCnt(p) <= (others => '0');
   end loop;
	
elsif rising_edge (SysClk) then 


-- register the claim so the clear arrives one cycle after the set
AutoTx_Claim_d <= AutoTx_Claim;

  -- Sticky timeout counters: rising-edge detect on AutoTx_TimedOut(p),
  -- saturate at 0xFF, clearable by µC write of a one-hot mask.
  AutoTx_TimedOut_d <= AutoTx_TimedOut;
  for p in 0 to 7 loop
    if AutoTx_TimeoutClr(p) = '1' then
      AutoTx_TimeoutCnt(p) <= (others => '0');
    elsif AutoTx_TimedOut(p) = '1' and AutoTx_TimedOut_d(p) = '0'
          and AutoTx_TimeoutCnt(p) /= X"FF" then
      AutoTx_TimeoutCnt(p) <= AutoTx_TimeoutCnt(p) + 1;
    end if;
  end loop;


-- Drive TxEnReq: set on AutoTx pulse, hold until FIFO drains (SysClk-safe empty)
    if AutoTx_TxEnReqPulse = '1' then
        AutoTx_TxEnReqHold <= '1';
    elsif TxEnReq = '1' then
        AutoTx_TxEnReqHold <= '0';
    end if;
	 	 
-- Snapshot current ReadyStatus into a working variable so all updates
-- this clock cycle are applied atomically before writing back.
rs_next := ReadyStatus;

-- Shift each per-port PHY Rx FIFO empty flag through a 2-stage delay
-- pipeline.  phy_empty_d(p)(0) holds the value from last cycle;
-- phy_empty_d(p)(1) holds the value from two cycles ago.
-- This pipeline is used elsewhere to detect a rising edge on the
-- empty flag (i.e. the FIFO transitioning from non-empty back to empty),
-- which signals that the DDR sequencer has drained the received data.
for p in 0 to 7 loop
  phy_empty_d(p)(1) <= phy_empty_d(p)(0);  -- age the older sample
  phy_empty_d(p)(0) <= PhyRxBuff_Empty(p); -- capture the current sample
end loop;





-- P5: Startup holdoff + PowerOnReady_done
-- Increment the holdoff counter until it saturates at 0xFF.
-- Only fire the one-shot ReadyStatus initialisation after the counter
-- has reached its maximum AND CpldRst_sync is confirmed stable at '1'.
if StartupHoldoff /= (StartupHoldoff'range => '1') then
  StartupHoldoff <= StartupHoldoff + 1;
end if;


-- P4: Power-on initialisation: arm exactly one FEB port to trigger the first UBT
-- handshake after reset.  The StartupHoldoff counter ensures all FIFOs,
-- synchronisers, and the AutoTx FSM have fully settled before any traffic is
-- generated.  Only the lowest-indexed masked port is armed here; subsequent
-- ports are armed one-at-a-time by AutoTx_ReArm after each complete
-- UBT ? FEB-reply ? DDR-drain cycle, preventing a burst of simultaneous
-- requests at startup.  PowerOnReady_done is a one-shot flag that prevents
-- this block from re-firing on subsequent clock cycles.
if PowerOnReady_done = '0'
   and StartupHoldoff = std_logic_vector(to_unsigned(2000000, 21))
   and CpldRst_sync = '1' then
  for p in 0 to 7 loop
    if MaskReg(p) = '1' then
      rs_next(p) := '1';
      exit;
    end if;
  end loop;
  PowerOnReady_done <= '1';
end if;

if DDRRd_en = '1' and DDRRd_EnD = '0' then
  rs_next := rs_next or MaskReg;
end if;

-- Auto-enable DDR and FM Rx after startup holdoff, exactly once
if PowerOnReady_done = '0'
   and StartupHoldoff = std_logic_vector(to_unsigned(2000000, 21))
   and CpldRst_sync = '1' then
  FMRxEn    <= '1';
  DDRWrt_En <= '1';
  DDRRd_en  <= '1';
end if;

-- Microcontroller force-set: the µC can directly assert any combination of
-- ReadyStatus bits by writing a one-hot (or multi-hot) mask to ReadyForceAddr.
-- This is primarily a debug/diagnostic tool that allows the host software to
-- manually re-arm one or more FEB ports without waiting for the normal
-- AutoTx handshake cycle to complete.  Bits set here are subject to the same
-- downstream clearing logic (AutoTx_Claim_d, ReadyClearAddr) as
-- autonomously-set bits.
if WRDL = 1 and uCA(11 downto 10) = GA
   and uCA(9 downto 0) = ReadyForceAddr then
  rs_next := rs_next or uCD(7 downto 0);
end if;




-- P2: Clear ReadyStatus for any port that AutoTx_Proc has just claimed (one cycle
-- delayed so the FSM has a full cycle to observe the set bit before it is
-- cleared), then re-arm any port signalled by AutoTx_ReArm (set at the end
-- of a successful UBT?reply?DDR-drain cycle to prime the next round-robin
-- iteration).
if AutoTx_Claim_d /= X"00" then
  rs_next := rs_next and (not AutoTx_Claim_d);
end if;

--if AutoTx_ReArm /= X"00" then
--  --  rs_next := rs_next or AutoTx_ReArm;
--	ReArm_pending <= (ReArm_pending or AutoTx_ReArm) and (not AutoTx_Claim_d);
--	rs_next := rs_next or ReArm_pending;
--end if;

-- 1) ReArm_pending: registered, OUTSIDE any gate, masked every cycle
ReArm_pending <= (ReArm_pending or AutoTx_ReArm) and (not AutoTx_Claim_d);

-- 2) Combinational OR into rs_next: same-cycle visibility for fresh pulse,
--    plus held value for following cycles
if AutoTx_ReArm /= X"00" then
  rs_next := rs_next or AutoTx_ReArm;          -- restore original line
end if;
rs_next := rs_next or ReArm_pending;           -- OUTSIDE the if, every cycle

-- 3) Reset (in your CpldRst_sync='0' branch of the main process):
ReArm_pending <= (others => '0');


if WRDL = 1 and uCA(11 downto 10) = GA
   and uCA(9 downto 0) = AutoTxTimeoutClrAddr then
  AutoTx_TimeoutClr <= uCD(7 downto 0);   -- one-hot mask of ports to clear
else
  AutoTx_TimeoutClr <= (others => '0');
end if;


-- P1: microcontroller explicit clear
if WRDL = 1 and uCA(11 downto 10) = GA
   and uCA(9 downto 0) = ReadyClearAddr then
  rs_next := rs_next and (not uCD(7 downto 0));
end if;

ReadyStatus <= rs_next;

   
-- Synchronous edge detectors for read and write strobes
RDDL(0) <= not uCRD and not CpldCS;
RDDL(1) <= RDDL(0); 

WRDL(0) <= not uCWR and not CpldCS;
WRDL(1) <= WRDL(0);

-- debug probes for testbench
-- synthesis translate_off
probe_ReadyStatus      <= ReadyStatus;
probe_MaskReg          <= MaskReg;
probe_PhyRxEmpty       <= PhyRxBuff_Empty;
probe_Rx_active        <= Rx_active;
probe_PhyTxBuff_Count  <= PhyTxBuff_Count;
probe_UBT_in_progress  <= AutoTx_Busy;
probe_handshake_queued <= AutoTx_Busy;
probe_AutoTx_Port      <= std_logic_vector(to_unsigned(AutoTx_Port, 3));
probe_PhyTxBuff_Empty  <= PhyTxBuff_Empty;
probe_AutoTx_TimedOut  <= AutoTx_TimedOut;
-- synthesis translate_on


-- Latch the address for post increment during reads
if RDDL = 1 or WRDL = 1 then AddrReg <= uCA;
else AddrReg <= AddrReg;
end if;


if WRDL = "01" and uCA(11 downto 10) = GA
        and uCA(9 downto 0) = LastTxTargetAddr then
    LastTxTarget_clr_req <= '1';
else
    LastTxTarget_clr_req <= '0';
end if;

-- 1us time base
if Counter1us /= Count1us then Counter1us <= Counter1us + 1;
else Counter1us <= X"00";
end if;

-- 10us time base
if Counter10us /= Count10us then Counter10us <= Counter10us + 1;
else Counter10us <= (others => '0');
end if;

-- 1ms time base
if Counter1ms /= Count1ms then Counter1ms <= Counter1ms + 1;
else Counter1ms <= (others => '0');
end if;

-- 1 second time base
if	Counter1s /= Count1s then Counter1s <= Counter1s + 1;
else Counter1s <= (others => '0');
end if;

-- Uptime in seconds since the last FPGA configure
if	Counter1s = Count1s then UpTimeCount <= UpTimeCount + 1;
else UpTimeCount <= UpTimeCount;
end if;

-- Register for staging uptime count.
if CpldCS = '1' then UpTimeStage <= UpTimeCount;
else UpTimeStage <= UpTimeStage;
end if;

-- Loop over eight LVDS receiver channels
for i in 0 to 7 loop

    -- FM edge detectors (registered on SysClk)
    RxDl(i)(0) <= FMRx(i);
    RxDl(i)(1) <= RxDl(i)(0);

    -- At the window boundary: evaluate, then reset for the next window
    --if Counter10us(5 downto 0) = "00" & X"0" then
	 if Counter10us = 0 then 
        TransitionCount(i) <= X"0";   -- reset counter for next window

        if TransitionCount(i) = 15 and MaskReg(i) = '1' then
            -- Link was active in the previous window
            Rx_active(i)       <= '1';
            DeadWindowCount(i) <= X"0";

        elsif TransitionCount(i) = 0 then
            -- No transitions: increment dead-window counter
            if DeadWindowCount(i) /= 15 then
                DeadWindowCount(i) <= DeadWindowCount(i) + 1;
            end if;
            -- De-assert Rx_active after 4 consecutive dead windows
            if DeadWindowCount(i) >= 4 or MaskReg(i) = '0' then
                Rx_active(i) <= '0';
            end if;
        end if;

    else
        -- Not a window boundary: apply immediate mask clear ...
        if MaskReg(i) = '0' then
            Rx_active(i) <= '0';
        end if;

        -- FIX: ... and count FM edges within this window
        if RxDl(i)(0) /= RxDl(i)(1) and TransitionCount(i) /= X"F" then
            TransitionCount(i) <= TransitionCount(i) + 1;
        end if;

    end if;

end loop;

-- Every sysclk reverses the clock and frame pattern
if LockOut = '1' then
	ClockReg <= not ClockReg; 
	FrameReg <= not FrameReg;
else
	ClockReg <= "10101"; 
	FrameReg <= "11111";
end if;
	
--- Channel mask register. One bit corresponds to one FEB
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = InputMaskAddr
then MaskReg <= uCD(7 downto 0);
else MaskReg <= MaskReg;
end if;

-- Data used to initialize the LVDS transmit fanout chip
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SPIWrtAddr
then SPI_WrtReq <= '1';
else SPI_WrtReq <= '0';
end if;

-- Buffer TDAQ trigger packets
if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = PhyTxFIFOWrtAd)
				 or uCA(9 downto 0) = PhyTxBroadCastAd)
then PhyTxBuff_wreq <= '1'; 
else PhyTxBuff_wreq <= '0';
end if;


-- stretch pulse
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TxFifoResetAddr)
   or AutoTx_FifoRst_req = '1' then
  PhyTxFifoRst_stretch <= X"F";
elsif PhyTxFifoRst_stretch /= 0 then
  PhyTxFifoRst_stretch <= PhyTxFifoRst_stretch - 1;
end if;

if PhyTxFifoRst_stretch /= 0 then
  PhyTxFifoRst_pulse <= '1';
else
  PhyTxFifoRst_pulse <= '0';
end if;



-- When at least one packet has bee received, examine it. If it is a trigger request packet,
-- Start the data transfer from DRAM to the serial link transmitter
if DatReqBuff_Count > 8 and DDR_Read_Seq = Idle then DatReqBuff_rdreq <= '1';
elsif TrigWdCount = 8 or TrigWdCntRst = '1' then DatReqBuff_rdreq <= '0';
else DatReqBuff_rdreq <= DatReqBuff_rdreq;
end if;

-- Use this counter to extract one packet at a time
if DatReqBuff_rdreq = '1' and TrigWdCount /= 8 then TrigWdCount <= TrigWdCount + 1;
elsif TrigWdCount = 8 or TrigWdCntRst = '1' then TrigWdCount <= X"0";
else TrigWdCount <= TrigWdCount;
end if;



if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
                or (uCA(9 downto 0) = CSRBroadCastAd))
then
  LinkRst      <= uCD(1);
  PhyPDn       <= not uCD(2);
  FMRxEn       <= uCD(3);
  DDRWrt_En    <= uCD(5);
  PhyDatSel    <= uCD(6);
  DDRRd_en     <= uCD(7);
  InitReq      <= uCD(8);
  TrigWdCntRst <= uCD(9);
else
  PhyPDn       <= PhyPDn;
  LinkRst      <= '0';
  FMRxEn       <= FMRxEn;
  DDRWrt_En    <= DDRWrt_En;
  PhyDatSel    <= PhyDatSel;
  DDRRd_en     <= DDRRd_en;
  InitReq      <= '0';
  TrigWdCntRst <= '0';
end if;

-- RxBuffRst is a single-cycle pulse ? decoded separately
-- so that unrelated CSR writes (e.g. PhyPDn=0) do NOT accidentally pulse it
if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
                or (uCA(9 downto 0) = CSRBroadCastAd))
  and uCD(0) = '1'
then RxBuffRst <= '1';
else RxBuffRst <= '0';
end if;




-- This is a copy of the data bit set in FPGA 1. 0 : the DReqFM bit becomes the "and" of the 
-- three link FIFO empty flags. 1 : The bit is FM data containig data request packets.
if WRDL = 1 and uCA = 0 then DRegSrc <= uCD(6);
else DRegSrc <= DRegSrc;
end if;
Debug(2) <= DRegSrc;
DDRWrt_EnD <= DDRWrt_En;DDRRd_EnD <= DDRRd_en;


-- AUTO-RELEASE: trigger PhyRst sequence once after startup holdoff saturates,
-- exactly as if the microcontroller had written bit0=1 to the CSR.
-- PhyRst_AutoDone prevents it firing more than once per power cycle.
if PhyRst_AutoDone = '0'
   and StartupHoldoff = std_logic_vector(to_unsigned(2000000, 21))
   and CpldRst_sync = '1'
   and PhyRstCnt = 0
then
   PhyRstCnt      <= "11";
   PhyRst_AutoDone <= '1';

-- MANUAL: microcontroller writes bit0=1 to CSR (re-triggers reset anytime)
elsif WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
                 or (uCA(9 downto 0) = CSRBroadCastAd))
      and uCD(0) = '1' and PhyRstCnt = 0
then
   PhyRstCnt <= "11";

-- COUNT DOWN: decrement at 1us intervals
elsif PhyRstCnt /= 0 and Counter1us = Count1us
then
   PhyRstCnt <= PhyRstCnt - 1;
end if;

-- PhyRst output: low while counting (PHY in reset), high when done
if PhyRstCnt = 0 and PhyRst_AutoDone = '1' then
   PhyRst <= '1';                              -- released after auto-init
elsif PhyRstCnt = 0 and PhyRst_AutoDone = '0' then
   PhyRst <= '0';                              -- still waiting for auto-init
elsif WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
                 or (uCA(9 downto 0) = CSRBroadCastAd))
      and uCD(0) = '1' and PhyRstCnt = 0
then
   PhyRst <= '0';                              -- start of manual reset pulse
elsif PhyRstCnt = 1 and Counter1us = Count1us
then
   PhyRst <= '1';                              -- end of manual reset pulse
else
   PhyRst <= PhyRst;
end if;

-- Timer to set width of DDR MIG reset
if WRDL = 1 and ((uCA(11 downto 10) = GA and uCA(9 downto 0) = CSRRegAddr)
					or (uCA(9 downto 0) = CSRBroadCastAd))
and uCD(4) = '1' and ResetCount = 0 
then ResetCount <= X"F";
elsif ResetCount /= 0 then ResetCount <= ResetCount - 1;
end if;

-- MIG Reset
if ResetCount /= 0 then DDR_Reset <= '1';
else DDR_Reset <= '0';
end if;

-- Testcounter counter is writeable. For each read of the lower half, the entire
-- 32 bit counter increments
if    WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterHiAd 
then TestCount <= (uCD & TestCount(15 downto 0));
elsif WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = TestCounterLoAd 
then TestCount <= (TestCount(31 downto 16) & uCD);
elsif RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = TestCounterLoAd 
then TestCount <= TestCount + 1;
else TestCount <= TestCount;
end if;

-- Write to the SMI serial interface
if WRDL = 1 and uCA(9 downto 0) >= SMIArrayMin and uCA(9 downto 0) <= SMIArrayMax
then SMI_wreq <= '1';
else SMI_wreq <= '0';
end if;

if WRDL = 1 and uCA(9 downto 0) = SMICtrlAddr 
	then ChainSel <= uCD(1 downto 0);
		  MDIORd <= uCD(2);
   else ChainSel <= ChainSel;
		  MDIORd <= MDIORd;
end if;

for i in 0 to 7 loop

-- Mocrocontroller read of the FEB FM Rx FIFOs
if RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FEBFMRdAddr(i) 
then FEBRxBuff_rdreq(i) <= '1';
else FEBRxBuff_rdreq(i) <= '0';
end if;

-- FEB FM receiver clear parity error
if ((WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FMRxErrAddr)
	and (uCD(i) = '1' or uCD(8) = '1')) or CpldRst_sync = '0'
then FEBRxIn(i).Clr_Err <= '1';
else FEBRxIn(i).Clr_Err <= '0';
end if;

end loop;


if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FMRxErrAddr
then RxIn(0).Clr_Err <= uCD(9);
else RxIn(0).Clr_Err <= '0';
end if;

if WRDL = 1 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = FMRxErrAddr
then RxIn(1).Clr_Err <= uCD(10);
else RxIn(1).Clr_Err <= '0';
end if;




-- TxEnReq: set when µC writes CSR bit-0=1, or AutoTx fires;
-- clear only after SMI_Proc has acknowledged (TxEnAck='1').
-- TxEnAck is generated in the i50MHz domain (SMI_Proc); read the
-- 2-FF-synchronised SysClk copy TxEnAck_sync(1) here to avoid CDC.
if TxEnReq = '0' and TxEnAck_sync(1) = '0' and (
     ( WRDL = 1 and (
         (uCA(11 downto 10) = GA and uCA(9 downto 0) = PhyTxCSRAddr   and uCD(0) = '1')
       or (uCA(9 downto 0) = PhyTxCSRBroadCastAd                       and uCD(0) = '1')
     ))
     or AutoTx_TxEnReqPulse = '1'
     or AutoTx_TxEnReqHold  = '1'
   )
then
    TxEnReq <= '1';
elsif TxEnReq = '1' and TxEnAck_sync(1) = '1' then
    TxEnReq <= '0';
-- else: hold current value (implicit in clocked process)
end if;

-------------------------------- DDR Macro Interfaces -------------------------------

-- Read_Seq_FSM is (Idle,Wait0,SetAddr,CheckEmpty,FirstCmd,CheckRdBuff0,RdWdCount,CheckWdCount,PrepareWordCnt,
-- CheckRdBuff1,RdDataHi,RdDataLo
case DDR_Read_Seq is
	When Idle => DDRRdStat <= "000"; --Debug(10 downto 8) <= "000";
		if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
		then DDR_Read_Seq <= CheckEmpty;
		elsif DDRRd_En = '1' and 
-- Copy data to the link FIFO when the reads lag the writes by at least one event 
			  ((AddrBuff_empty = '0' and TxBlkCount /= 0 and DDRWrt_En = '1') 
		   or (TrigReqCount /= 0 and DDRWrt_En = '0'))
		then DDR_Read_Seq <= Wait0;
		else DDR_Read_Seq <= Idle;
		end if;
	When Wait0 => 
			if WaitCount = 1 then DDR_Read_Seq <= SetAddr;
	        else DDR_Read_Seq <= Wait0;
			end if;
	When SetAddr => DDR_Read_Seq <= CheckEmpty;  DDRRdStat <= "001"; --Debug(10 downto 8) <= "001";
-- Clear out any stale data from the read FIFO before starting a new block read
	When CheckEmpty => DDRRdStat <= "010"; --Debug(10 downto 8) <= "010";
		if SDrd_empty = '1' then DDR_Read_Seq <= FirstCmd;
		else DDR_Read_Seq <= CheckEmpty;
		end if;
	When FirstCmd => DDRRdStat <= "011"; --Debug(10 downto 8) <= "011";
		if (AddrBuff_empty = '0' and DDRWrt_En = '1')
		or	(TrigReqCount /= 0 and DDRWrt_En = '0')
		then DDR_Read_Seq <= CheckRdBuff0;
		else DDR_Read_Seq <= Idle;
		end if;
	When CheckRdBuff0 => DDRRdStat <= "111";  --Debug(10 downto 8) <= "111";
			if SDrd_empty = '0' then DDR_Read_Seq <= RdWdCount; 
			else DDR_Read_Seq <= CheckRdBuff0;
			end if;
	When RdWdCount => DDRRdStat <= "100"; --Debug(10 downto 8) <= "100";
	    DDR_Read_Seq <= CheckWdCount;
	When CheckWdCount => DDRRdStat <= "100";
	    DDR_Read_Seq <= PrepareWordCnt;
	When PrepareWordCnt => DDRRdStat <= "100";
			if RdHi_LoSel = '0'
			then DDR_Read_Seq <= RdDataHi;
			else DDR_Read_Seq <= RdDataLo;
			end if;
	When CheckRdBuff1 => DDRRdStat <= "101"; --Debug(10 downto 8) <= "101";
		if SDrd_empty = '0' then DDR_Read_Seq <= RdDataHi;
		else DDR_Read_Seq <= CheckRdBuff1;
		end if;
	When RdDataHi => DDRRdStat <= "110"; --Debug(10 downto 8) <= "110";
		if EvWdCount = 0 then DDR_Read_Seq <= Idle;
		else DDR_Read_Seq <= RdDataLo;
		end if;
	When RdDataLo => DDRRdStat <= "111"; --Debug(10 downto 8) <= "111";
 		   if EvWdCount /= 0 and SDrd_en = '1' then DDR_Read_Seq <= CheckRdBuff1;
		elsif EvWdCount = 0 then DDR_Read_Seq <= Idle;
		else DDR_Read_Seq <= RdDataLo;
		end if;
	When others => DDR_Read_Seq <= Idle;
end case;

-- Wait for the DDR write data to get all the way to the inernal capacitor array..
 if DDR_Read_Seq = Wait0 and WaitCount = 0
	then WaitCount <= (others => '1');
 elsif WaitCount /= 0
   then WaitCount <= WaitCount - 1;
 else WaitCount <= WaitCount;
 end if;

-- Filter the "link FOFOs all empty" flag from FPGA 1. 
 DReqFMDL(3 downto 0) <= DReqFMDL(2 downto 0) & DReqFM; 
    if DReqFMDL = X"F" then LinkFIFOStat <= '1';
 elsif DReqFMDL = 0 then LinkFIFOStat <= '0';
 else LinkFIFOStat <= LinkFIFOStat;
 end if;
 
-- Wait for the link FIFOs to go empty before sending a block of events to FPGA 1
 if DRegSrc = '0' and TxBlkCount = 0 and LinkFIFOStat = '1' 
  then TxBlkCount <= "100";
 elsif TxBlkCount /= 0 and DDR_Read_Seq = FirstCmd
  then TxBlkCount <= TxBlkCount - 1;
 else TxBlkCount <= TxBlkCount;
 end if;
Debug(1) <= DReqFM;

	if DDR_Read_Seq = RdWdCount and RdHi_LoSel = '0' then EvWdCount <= SDRdDat(31 downto 16) + 1;
elsif DDR_Read_Seq = RdWdCount and RdHi_LoSel = '1' then EvWdCount <= SDRdDat(15 downto 0) + 1;
elsif DDR_Read_Seq = CheckWdCount and EvWdCount > MAX_TX_WORDS then EvWdCount <= MAX_TX_WORDS + 1;
elsif EvWdCount /= 0 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1'))
   then EvWdCount <= EvWdCount - 1;
	elsif DDRRd_en = '0' then EvWdCount <= (others => '0');
else EvWdCount <= EvWdCount;
end if;

if DDR_Read_Seq = PrepareWordCnt and EvWdCount > MAX_TX_WORDS then 
    tx_overflow <= '1';
	 tx_overflow_cnt <= tx_overflow_cnt + 1;
elsif DDR_Read_Seq = PrepareWordCnt and EvWdCount <= MAX_TX_WORDS then 
    tx_overflow <= '0';
	 tx_overflow_cnt <= tx_overflow_cnt;
else 
    tx_overflow <= tx_overflow;
	 tx_overflow_cnt <= tx_overflow_cnt;
end if;


-- DDR Read address register
-- Microcontroller access upper
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrHiAd 
then SDRdAD <= uCD(13 downto 0) & SDRdAD(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd 
then SDRdAD <= SDRdAD(29 downto 16) & uCD;
-- Increment by 8 long words for each burst read command
elsif SDRdCmdEn = '1' and DDR_Read_Seq /= CheckEmpty then SDRdAD <= SDRdAD + 32;
elsif InitReq = '1' then SDRdAD <= (others => '0');
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '1'
 then SDRdAD(29 downto 2) <= AddrBuff_Out;
	  SDRdAD(1 downto 0) <= "00";
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '0' and SDRdPtr(4 downto 0) /= 0 
then SDRdAD(29 downto 5) <= SDRdPtr(29 downto 5) + 1;
	  SDRdAD(4 downto 0) <= "00000";
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '0' and SDRdPtr(4 downto 0) = 0 
then SDRdAD <= SDRdPtr;
else SDRdAD <= SDRdAD;
end if;

-- Use this pointer to keep track of the read address for every read 
 if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrHiAd 
then SDRdPtr <= uCD(13 downto 0) & SDRdPtr(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd 
then SDRdPtr <= SDRdPtr(29 downto 16) & uCD;
elsif SDrd_en = '1' and DDR_Read_Seq /= CheckEmpty then SDRdPtr <= SDRdPtr + 4;
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '1'
then SDRdPtr(29 downto 2) <= AddrBuff_Out;
	  SDRdPtr(1 downto 0) <= "00";
elsif DDR_Read_Seq = SetAddr and DDRWrt_En = '0' and SDRdPtr(4 downto 0) /= 0 
then SDRdPtr(29 downto 5) <= SDRdPtr(29 downto 5) + 1;
	  SDRdPtr(4 downto 0) <= "00000";
else SDRdPtr <= SDRdPtr;
end if;

-- DDR controller output FIFO is 32 bits. Clock once per two uC reads
  if (RDDL = 2 and AddrReg(11 downto 10) = GA 
		and (AddrReg(9 downto 0) = SDRamPortAd or AddrReg(9 downto 0) = SDRamSwapPort) 
		and RdHi_LoSel = '1')
	or (DDR_Read_Seq = CheckEmpty and SDrd_empty = '0')
	or (SDrd_en = '0' and EvWdCount /= 0 and DDR_Read_Seq = RdDataLo)
	then SDrd_en <= '1'; 
   else SDrd_en <= '0';
  end if;

-- Send a read command to fetch burst size number of long words
 if (SDrd_en = '1' and SDRdPtr(4 downto 0) = "01100" and SDcmd_full(1) = '0' and DDR_Read_Seq /= CheckEmpty) 
  or DDR_Read_Seq = FirstCmd
then SDRdCmdEn <= '1'; 
	  SDRdCmd <= ReadCmd; 
else SDRdCmdEn <= '0';   
	  SDRdCmd <= "000";  
end if;

-- Use this counter to time the burst reads.
if DDR_Read_Seq = FirstCmd then ReadCount <= "111";
elsif SDrd_en = '1' then ReadCount <= ReadCount - 1;
else ReadCount <= ReadCount;
end if;

-- Toggle between upper and lower words during reads from the DDR
    if DDR_Read_Seq = SetAddr or InitReq = '1'
or (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamRdPtrLoAd)
then RdHi_LoSel <= '0'; 
 elsif (RDDL = 2 and AddrReg(11 downto 10) = GA 
 and (AddrReg(9 downto 0) = SDRamPortAd or AddrReg(9 downto 0) = SDRamSwapPort))
  or (EvWdCount /= 0 and (DDR_Read_Seq = RdDataHi
  or (DDR_Read_Seq = RdDataLo and EvWdCount /= 0 and SDrd_en = '0')))
then RdHi_LoSel <= not RdHi_LoSel; 
end if;

--- Link FPGA2-FPG1 configuration
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = LinkCtrlAd
then LinkStatEn <= uCD(0);
else LinkStatEn <= LinkStatEn;
end if;

-- At 1Hz SEND A request to update the FM activity bits via the link to FPGA 1
-- Make this a lower priority than data transmission
-- DEBUG 
if Counter1s = Count1s and LinkTxEmpty = '1' and DDR_Read_Seq = Idle and LinkStatEn = '1'
  then Link_Stat_Req <= '1';
 elsif LinkTxWrReq = '1'
  then Link_Stat_Req <= '0';
end if;


-- Serial link write to the top level FPGA
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TxFIFOWrtAd)
  or (EvWdCount > 1 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1')))
  or (LinkTxWrReq = '0' and LinkTxEmpty = '1' and DDR_Read_Seq = Idle and Link_Stat_Req = '1')
then 
  LinkTxWrReq <= '1'; 
  if LinkTxFull = '1'
  then LinkTxFullCnt <= LinkTxFullCnt + 1;
  else LinkTxFullCnt <= LinkTxFullCnt;
  end if;  
else 
  LinkTxWrReq <= '0';
  LinkTxFullCnt <= LinkTxFullCnt;
end if;

-- the same for the buffer, don't buffer the status packages though
if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = TxFIFOWrtAd)
  or (EvWdCount > 1 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1')))
then LinkTxTraceWrReq <= '1';  
else LinkTxTraceWrReq <= '0'; 
end if;


-- Track word position
if DDR_Read_Seq = PrepareWordCnt then 
    --tx_word_cnt <= (others => '0');  -- Reset at start of event
	 word_number <= "01";
elsif EvWdCount /= 0 and (DDR_Read_Seq = RdDataHi or (DDR_Read_Seq = RdDataLo and SDrd_en = '1')) 
    then 
	 -- tx_word_cnt <= tx_word_cnt + 1;
	 word_number(0) <= '0';
	 word_number(1 downto 1) <= word_number(0 downto 0);
else 
    --tx_word_cnt <= tx_word_cnt;
	 word_number <= word_number;
end if;

if DDR_Read_Seq = PrepareWordCnt then
    EvWdCountTot <= EvWdCount - 1; -- EvWdCountTot goes to the header, the +1 is needed for state machine
else
    EvWdCountTot <= EvWdCountTot;
end if;

-- Serial link transmit data
if Link_Stat_Req = '0' and DDR_Read_Seq = RdDataHi then
	 if word_number(0) = '1' then
	     LinkFIFO_Dat(17 downto 9) <= '1' & EvWdCountTot(15 downto 8);
        LinkFIFO_Dat(8 downto 0)  <= '1' & EvWdCountTot( 7 downto 0);	 
	 elsif word_number(1) = '1' and tx_overflow = '1' then
        LinkFIFO_Dat(17 downto 9) <= '1' & (SDRdDat(31 downto 24) or OVERFLOW_STATUS_BIT(15 downto 8));
        LinkFIFO_Dat(8 downto 0) <= '1' &  (SDRdDat(23 downto 16) or OVERFLOW_STATUS_BIT( 7 downto 0)); 
	 else
        LinkFIFO_Dat(17 downto 9) <= '1' & SDRdDat(31 downto 24);
        LinkFIFO_Dat(8 downto 0) <= '1' & SDRdDat(23 downto 16);
	 end if;
elsif Link_Stat_Req = '0' and DDR_Read_Seq = RdDataLo then 
	 if word_number(0) = '1' then
	     LinkFIFO_Dat(17 downto 9) <= '1' & EvWdCountTot(15 downto 8);
        LinkFIFO_Dat(8 downto 0)  <= '1' & EvWdCountTot( 7 downto 0);
	 elsif word_number(1) = '1' and tx_overflow = '1' then
        LinkFIFO_Dat(17 downto 9) <= '1' & (SDRdDat(15 downto 8) or OVERFLOW_STATUS_BIT(15 downto 8));
		  LinkFIFO_Dat(8 downto 0) <= '1' &   (SDRdDat(7 downto 0) or OVERFLOW_STATUS_BIT( 7 downto 0));
    else
	     LinkFIFO_Dat(17 downto 9) <= '1' & SDRdDat(15 downto 8);
		  LinkFIFO_Dat(8 downto 0) <= '1' & SDRdDat(7 downto 0);
    end if;
elsif Link_Stat_Req = '1' then LinkFIFO_Dat <= '0' & X"00" & '0' & Rx_active;
else LinkFIFO_Dat(17 downto 9) <= '1' & uCD(15 downto 8);
	   LinkFIFO_Dat(8 downto 0) <= '1' & uCD(7 downto 0);
end if;

-- Send link data until the buffer is empty
if LinkTxEmpty = '0' and FrameReg = "00000" 
then LinkTxRDReq <= '1';
elsif LinkTxEmpty = '1' then LinkTxRDReq <= '0';
end if;

-- Append the valid bit to the data stream when transmitting
if LinkTxEmpty = '0' and FrameReg = "00000"
then TxValid <= '1';
elsif LinkTxEmpty = '1' then TxValid <= '0';
else  TxValid <= TxValid;
end if;

if DDR_Write_Seq = Rd_WdCount then WrtAddrReg <= SDWrtAd(29 downto 2);
else WrtAddrReg <= WrtAddrReg;
end if;

if DDR_Write_Seq = IncrBuffCnt and SDwr_empty = '1' 
then AddrBuff_wren <= '1';
else AddrBuff_wren <= '0';
end if;

if DDR_Read_Seq = FirstCmd or (DDRRd_En = '0' and AddrBuff_empty = '0') then AddrBuff_rden <= '1';
else AddrBuff_rden <= '0';
end if;

-- DDR Write address register
-- Microcontroller access upper
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrHiAd 
then SDWrtAd <= uCD(13 downto 0) & SDWrtAd(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd
then SDWrtAd <= SDWrtAd(29 downto 16) & uCD;
-- Reset the address when writes are enabled in preparation for incoming FEB data
elsif DDRWrt_En = '1' and DDRWrt_EnD = '0' then SDWrtAd <= (others => '0');
-- Increment by 4 for each long word write
elsif SDwr_en = '1'
then SDWrtAd <= SDWrtAd + 4;
else SDWrtAd <= SDWrtAd;
end if;

-- DDR Write address staging register
-- Microcontroller access upper
if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrHiAd 
then SDWrtAdStage <= uCD(13 downto 0) & SDWrtAdStage(15 downto 0);
-- Microcontroller access lower
elsif WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd
then SDWrtAdStage <= SDWrtAdStage(29 downto 16) & uCD;
-- Reset the address when writes are enabled in preparation for incoming FEB data
elsif DDRWrt_En = '1' and DDRWrt_EnD = '0' then SDWrtAdStage <= (others => '0');
-- Keep the address from the last update until the write command has been sent
elsif WrtCmdEn = '1' then SDWrtAdStage <= SDWrtAd;
else SDWrtAdStage <= SDWrtAdStage;
end if;

-- DDR write data staging register
if WrtHi_LoSel = '0' then 
	if WRDL = 1 and  uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd
	 then CDStage <= uCD;
  elsif DDR_Write_Seq = Write_Wd_Count
		then CDStage <= EventWdCnt;
  elsif DDR_Write_Seq = Wrt_uBunchHi
		then  CDStage <= uBunch(31 downto 16);
  elsif DDR_Write_Seq = Wrt_uBunchLo 
		then CDStage <= uBunch(15 downto 0);
  elsif DDR_Write_Seq = Wrt_Stat
		then CDStage <= EventStat;
	elsif  DDR_Write_Seq = WrtDDR
	 then CDStage <= PhyRxBuff_Out(PortNo);
	end if;
  else CDStage <= CDStage;
 end if;

-- Multiplexer to feed the appropriate data to the DRAM write FIFO
	if WrtHi_LoSel = '1' then
	  SDWrtDat(31 downto 16) <= CDStage;
	else SDWrtDat(31 downto 16) <= SDWrtDat(31 downto 16);
	end if;

	 if WrtHi_LoSel = '1' then
	  if WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd
	    then SDWrtDat(15 downto 0) <= uCD;
     elsif DDR_Write_Seq = WrtDDR 
	     then SDWrtDat(15 downto 0) <= PhyRxBuff_Out(PortNo);
	  elsif DDR_Write_Seq = Write_Wd_Count
		  then SDWrtDat(15 downto 0) <= EventWdCnt;
	  elsif DDR_Write_Seq = Wrt_uBunchHi 
	     then SDWrtDat(15 downto 0) <= uBunch(31 downto 16);
     elsif DDR_Write_Seq = Wrt_uBunchLo
		  then SDWrtDat(15 downto 0) <= uBunch(15 downto 0);
     elsif DDR_Write_Seq = Wrt_Stat
		   then SDWrtDat(15 downto 0) <= EventStat;
	  end if;
	  else SDWrtDat(15 downto 0) <= SDWrtDat(15 downto 0);
 end if;

-- Writes to the MIG write FIFO
if (WrtHi_LoSel = '1'
	and ((WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd) 
	  or DDR_Write_Seq = Write_Wd_Count or DDR_Write_Seq = Wrt_Stat or DDR_Write_Seq = WrtDDR
	  or DDR_Write_Seq = Wrt_uBunchHi or DDR_Write_Seq = Wrt_uBunchLo))
  or (DDR_Write_Seq = WritePad and (SDwr_en = '0' or SDWrtAd(4 downto 0) /= "11100")) 
then SDwr_en <= '1'; --Debug(1) <= '1';
else SDwr_en <= '0'; --Debug(1) <= '0';
end if;

-- When the number of writes = burst size, send a write command
	if (SDwr_en = '1' and SDWrtAd(4 downto 0) = "11100")
-- Issue a write when the write address is being set by the microcontroller so the write FIFO is empty
-- when the write data is loaded into the write FIFO
		or(WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd)
		or DDR_Write_Seq = SndCmd
			then SDWrtCmd <= "010";
				  WrtCmdEn <= '1';   
			else SDWrtCmd <= "000";
				  WrtCmdEn <= '0';
	  end if;

-- out trace buffer
--	Read of the trigger request trace buffer
	if (RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = LinkTxTraceAd ) or 
	    (LinkTxTrace_Cnt >= "1" & X"F00" ) -- this should make this buffer to a trace buffer.
	then LinkTxTraceRDReq <= '1';
	else LinkTxTraceRDReq <= '0';
	end if;



-- Sum the word counts from the eight PHY receive FIFOs. The four header 
-- words from each FEB are stripped off, then the four header words for the 
-- concatenated data are appended
if DDR_Write_Seq = Idle 
	  then EventWdCnt <= (others => '0');
  elsif DDR_Write_Seq = Rd_WdCount and PhyRxBuff_Out(PortNo) >= 4
	  then EventWdCnt <= EventWdCnt + (PhyRxBuff_Out(PortNo) - 4);
  elsif DDR_Write_Seq = CheckActive0 and PortNo = 7 
	  then EventWdCnt <= EventWdCnt + 4;
 else EventWdCnt <= EventWdCnt;
end if;

-- Signal indicating the first active port
 if DDR_Write_Seq = Idle then FirstActive <= '1';
	elsif DDR_Write_Seq = Rd_Stat then FirstActive <= '0';
   else FirstActive <=  FirstActive;
 end if;	

-- Load the microbunch number from the first active port
	if FirstActive = '1' then
	  if DDR_Write_Seq = Rd_uBunchHi then uBunch <= PhyRxBuff_Out(PortNo) & uBunch(15 downto 0); 
	   elsif DDR_Write_Seq = Rd_uBunchLo then uBunch <= uBunch(31 downto 16) & PhyRxBuff_Out(PortNo);
     end if;
    else uBunch <= uBunch;
	end if;

-- "OR" the status bits from the FEBs and compare the first microbunch to any additional microbunches.
-- EventStat
-- bit 0 to 7: indicate error on port
-- bit 8 to 10: what error(s)
-- bit 11: uB mismatch between ports

if DDR_Write_Seq = Idle then EventStat <= (others => '0'); 
elsif	DDR_Write_Seq = Rd_Stat then
    -- OR each EventStat bit with the corresponding 4-bit group
	 EventStat(PortNo) <= PhyRxBuff_Out(PortNo)(0)  or PhyRxBuff_Out(PortNo)(1)  or PhyRxBuff_Out(PortNo)(2)  or PhyRxBuff_Out(PortNo)(3) or
	                      PhyRxBuff_Out(PortNo)(4)  or PhyRxBuff_Out(PortNo)(5)  or PhyRxBuff_Out(PortNo)(6)  or PhyRxBuff_Out(PortNo)(7) or
                         PhyRxBuff_Out(PortNo)(8)  or PhyRxBuff_Out(PortNo)(9)  or PhyRxBuff_Out(PortNo)(10) or PhyRxBuff_Out(PortNo)(11) or
                         PhyRxBuff_Out(PortNo)(12) or PhyRxBuff_Out(PortNo)(13) or PhyRxBuff_Out(PortNo)(14) or PhyRxBuff_Out(PortNo)(15);								 
	 EventStat(8)  <= EventStat(8) or PhyRxBuff_Out(PortNo)(3)  or PhyRxBuff_Out(PortNo)(2)  or PhyRxBuff_Out(PortNo)(1)  or PhyRxBuff_Out(PortNo)(0);
	 EventStat(9)  <= EventStat(9) or PhyRxBuff_Out(PortNo)(7)  or PhyRxBuff_Out(PortNo)(6)  or PhyRxBuff_Out(PortNo)(5)  or PhyRxBuff_Out(PortNo)(4)
	                              or PhyRxBuff_Out(PortNo)(11) or PhyRxBuff_Out(PortNo)(10) or PhyRxBuff_Out(PortNo)(9)  or PhyRxBuff_Out(PortNo)(8);
	 EventStat(10) <= EventStat(10) or PhyRxBuff_Out(PortNo)(15) or PhyRxBuff_Out(PortNo)(14) or PhyRxBuff_Out(PortNo)(13) or PhyRxBuff_Out(PortNo)(12);
    EventStat(15 downto 11) <= EventStat(15 downto 11); -- Keep upper bits unchanged
elsif FirstActive = '0' then
	 if (DDR_Write_Seq = Rd_uBunchHi and uBunch(31 downto 16) /= PhyRxBuff_Out(PortNo))
	 or (DDR_Write_Seq = Rd_uBunchLo and uBunch(15 downto 0) /= PhyRxBuff_Out(PortNo))
	  then  -- EventStat(7 downto 0) <= EventStat(7 downto 0) or UB_MISMATCH_STATUS_BIT; 
	      EventStat(11) <= EventStat(11) or '1';
	 end if;
else EventStat(15 downto 0) <= EventStat(15 downto 0);
end if;


-- Toggle between upper and lower words during writes to the DDR
    if (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamWrtPtrLoAd)
	 or (DDR_Write_Seq = Idle and DDRWrt_EnD = '1' and EventRdy = '1') then WrtHi_LoSel <= '0'; 
 elsif (WRDL = 1 and uCA(11 downto 10) = GA and uCA(9 downto 0) = SDRamPortAd)
	  or DDR_Write_Seq = Write_Wd_Count or DDR_Write_Seq = Wrt_Stat or DDR_Write_Seq = WrtDDR
	  or DDR_Write_Seq = Wrt_uBunchHi or DDR_Write_Seq = Wrt_uBunchLo
then WrtHi_LoSel <= not WrtHi_LoSel;
else WrtHi_LoSel <= WrtHi_LoSel;
end if;

-- loop through eight Phy Rx buffer read request and event status lines
for i in 0 to 7 loop

-- Sequencer read to microcontroller reads of the PhyRx FIFOs  
if (RDDL = 2 and AddrReg(11 downto 10) = GA and AddrReg(9 downto 0) = PhyRxRdAddr(i))
--or (PortNo = i and 
--  ((DDR_Write_Seq = CheckActive0 and Rx_active(PortNo) = '1') 
--or (DDR_Write_Seq = CheckActive1 and Rx_active(PortNo) = '1' and HitFlag(PortNo) = '1')
--  or DDR_Write_Seq = Rd_WdCount or DDR_Write_Seq = Rd_uBunchHi 
--  or DDR_Write_Seq = Rd_uBunchLo 
--  or (DDR_Write_Seq = WrtDDR and PortWdCounter(PortNo) > 1 )))
-- comment this code
or (PortNo = i and
  ((DDR_Write_Seq = CheckActive0
        and (Rx_active(PortNo) = '1' or PhyRxBuff_Empty(PortNo) = '0'))
or (DDR_Write_Seq = CheckActive1
        and (Rx_active(PortNo) = '1' or PhyRxBuff_Empty(PortNo) = '0')
        and HitFlag(PortNo) = '1')
   or DDR_Write_Seq = Rd_WdCount
   or DDR_Write_Seq = Rd_uBunchHi
   or DDR_Write_Seq = Rd_uBunchLo
   or (DDR_Write_Seq = WrtDDR and PortWdCounter(PortNo) > 1)))
then PhyRxBuff_rdreq(i) <= '1';
else PhyRxBuff_rdreq(i) <= '0';
end if;

---- Status indicating a port is active and a complete event is available for readout

if MaskReg(i) = '1'
   and PhyRxBuff_Empty(i) = '0'
   and PhyRxBuff_Out(i) >= X"0004"             -- minimum header size
   and PhyRxBuff_Out(i) <= X"0FFF"             -- sanity cap (FIFO depth)
   and PhyRxBuff_RdCnt(i) >= PhyRxBuff_Out(i)
  then PhyRxBuff_RdStat(i) <= '1';
  else PhyRxBuff_RdStat(i) <= '0';
end if;


end loop;

-- Flag to indicate an FEB had hits for this event.
-- This expression didn't work inside a loop statement. 
 if DDR_Write_Seq = Idle then HitFlag <= X"00";
 elsif DDR_Write_Seq = Rd_WdCount 
	 then 
		Case PortNo is
		 when 0 => if PhyRxBuff_Out(0) > 4 then HitFlag(0) <= '1';
						else HitFlag(0) <= '0';
						end if;
		 when 1 => if PhyRxBuff_Out(1) > 4 then HitFlag(1) <= '1';
						else HitFlag(1) <= '0';
						end if;
		 when 2 => if PhyRxBuff_Out(2) > 4 then HitFlag(2) <= '1';
						else HitFlag(2) <= '0';
						end if;
		 when 3 => if PhyRxBuff_Out(3) > 4 then HitFlag(3) <= '1';
						else HitFlag(3) <= '0';
						end if;
		 when 4 => if PhyRxBuff_Out(4) > 4 then HitFlag(4) <= '1';
						else HitFlag(4) <= '0';
						end if;
		 when 5 => if PhyRxBuff_Out(5) > 4 then HitFlag(5) <= '1';
						else HitFlag(5) <= '0';
						end if;
		 when 6 => if PhyRxBuff_Out(6) > 4 then HitFlag(6) <= '1';
						else HitFlag(6) <= '0';
						end if;
		 when 7 => if PhyRxBuff_Out(7) > 4 then HitFlag(7) <= '1';
						else HitFlag(7) <= '0';
						end if;
		end Case;
   else HitFlag <= HitFlag;
 end if;

-- Signal to indicate if all active ports have an event ready

if PhyRxBuff_RdStat /= X"00" then EventRdy <= '1';
else EventRdy <= '0';
end if;


-- Use this variable to cycle through the input ports during DDR writes
if PortNo /= 7 and (DDR_Write_Seq = IncrPort0 or DDR_Write_Seq = IncrPort1)
	      then PortNo <= PortNo + 1;
           elsif DDR_Write_Seq = ResetPortNo or (DDR_Write_Seq = Idle and EventRdy = '1') then PortNo <= 0; 
       else PortNo <= PortNo;
end if;

-- Load the words counts from the headers into eight counters, one for each port
	if DDR_Write_Seq = Idle then PortWdCounter <= (others => (others => '0'));
	  elsif DDR_Write_Seq = Rd_WdCount and PhyRxBuff_Out(PortNo) /= 0
		then PortWdCounter(PortNo) <= PhyRxBuff_Out(PortNo) - 1;
	  elsif DDR_Write_Seq = Rd_WdCount and PhyRxBuff_Out(PortNo) = 0
		then PortWdCounter(PortNo) <= (others => '0');
	  elsif PortWdCounter(PortNo) /= 0 and PhyRxBuff_rdreq(PortNo) = '1' and PhyRxBuff_Empty(PortNo) = '0' 
		then PortWdCounter(PortNo) <= PortWdCounter(PortNo) - 1;
	 else PortWdCounter(PortNo) <= PortWdCounter(PortNo);
	end if;



Case DDR_Write_Seq is
   When Idle => DDRWrtStat <= X"0"; Debug(10 downto 7) <= X"0"; 
-- Make sure the DDR write FIFO is empty before taking the first trigger
		if DDRWrt_En = '1' and DDRWrt_EnD = '0' then DDR_Write_Seq <= ChkWrtBuff;
-- When the input FIFOs have at least one event, copy the data to the DDR
		elsif DDRWrt_EnD = '1' and EventRdy = '1' then DDR_Write_Seq <= CheckActive0;
		else DDR_Write_Seq <= Idle;
		end if;
-- If there is stale data in the MIG transmit FIFO, force a burst write
	When ChkWrtBuff => 
		if SDwr_count /= 0 then DDR_Write_Seq <= SndCmd;
		else DDR_Write_Seq <= Idle;
		end if;
-- Send a MIG burst write command
	When SndCmd => DDR_Write_Seq <= WtCmdMtpy; 
-- Wait for the MIG write FIFO to go empty
	When WtCmdMtpy =>  
		if SDwr_empty = '1' then DDR_Write_Seq <= Idle;
		else DDR_Write_Seq <= WtCmdMtpy;
		end if;

	When CheckActive0 =>
    -- Only abort to Idle when BOTH FM links AND all Rx FIFOs are idle.
    if Rx_active = 0 and PhyRxBuff_Empty = X"FF" then
        DDR_Write_Seq <= Idle;
    -- Process this port if FM link is active OR its Rx FIFO has data.
    elsif Rx_active(PortNo) = '1' or PhyRxBuff_Empty(PortNo) = '0' then
        DDR_Write_Seq <= Rd_WdCount;
    -- Nothing on this port; try the next one.
    elsif PortNo /= 7 then
        DDR_Write_Seq <= IncrPort0;
    else
        DDR_Write_Seq <= Write_Wd_Count;
    end if;
	When Rd_WdCount => DDR_Write_Seq <= Rd_uBunchHi;  DDRWrtStat <= X"3"; Debug(10 downto 7) <= X"3";
	When Rd_uBunchHi => DDR_Write_Seq <= Rd_uBunchLo; DDRWrtStat <= X"4"; Debug(10 downto 7) <= X"4"; 
	When Rd_uBunchLo => DDR_Write_Seq <= Rd_Stat; DDRWrtStat <= X"5"; Debug(10 downto 7) <= X"5";
	When Rd_Stat =>  DDRWrtStat <= X"6"; Debug(10 downto 7) <= X"6"; 
	      if PortNo = 7 then DDR_Write_Seq <= Write_Wd_Count;
		   else DDR_Write_Seq <= IncrPort0;
			end if;
	When Write_Wd_Count => DDR_Write_Seq <= Wrt_Stat; DDRWrtStat <= X"7"; Debug(10 downto 7) <= X"7";
	When Wrt_Stat => DDR_Write_Seq <= Wrt_uBunchHi;  DDRWrtStat <= X"A"; Debug(10 downto 7) <= X"8";
	When Wrt_uBunchHi => DDR_Write_Seq <= Wrt_uBunchLo;  DDRWrtStat <= X"8"; Debug(10 downto 7) <= X"9"; 
	When Wrt_uBunchLo => DDR_Write_Seq <= ResetPortNo; DDRWrtStat <= X"9"; Debug(10 downto 7) <= X"A"; 
	When ResetPortNo => DDR_Write_Seq <= CheckActive1; DDRWrtStat <= X"B"; Debug(10 downto 7) <= X"B";
	When IncrPort1 => DDR_Write_Seq <= CheckActive1; DDRWrtStat <= X"C"; Debug(10 downto 7) <= X"C"; 
	When CheckActive1 => DDRWrtStat <= X"D"; Debug(10 downto 7) <= X"D"; 
		if PortWdCounter(PortNo) /= 0
		then DDR_Write_Seq <= WrtDDR;
		elsif PortNo = 7 and PortWdCounter(PortNo) = 0
		then DDR_Write_Seq <= WritePad;
		else DDR_Write_Seq <= IncrPort1; 
		end if;
	When WrtDDR => DDRWrtStat <= X"E"; Debug(10 downto 7) <= X"E"; 
		if PortNo /= 7 and (PortWdCounter(PortNo) = 1 or PortWdCounter(PortNo) = 0)
			then DDR_Write_Seq <= IncrPort1;
		elsif (PortNo = 7 and (PortWdCounter(PortNo) = 1 or PortWdCounter(PortNo) = 0)) 
				or DDRWrt_En = '0'
		then DDR_Write_Seq <= WritePad;
		else DDR_Write_Seq <= WrtDDR;
		end if;
	When WritePad => DDRWrtStat <= X"F"; Debug(10 downto 7) <= X"F"; 
		if SDWrtAd(4 downto 0) = "11100" 
		then DDR_Write_Seq <= IncrBuffCnt;
		else DDR_Write_Seq <= WritePad;
		end if;
	When IncrBuffCnt => 
		if SDwr_empty = '1' then 
		 DDR_Write_Seq <= Idle;
		else DDR_Write_Seq <= IncrBuffCnt;
		end if;
	When others => DDR_Write_Seq <= Idle;  
	end case;

	if DDR_Write_Seq = Idle then Seq_Busy <= '0';
	 else Seq_Busy <= '1';
	end if;



end if; -- CpldRst

end process main;

------------------- mux for reading back registers -------------------------

DDRRd_Mux <= SDRdDat(31 downto 16) when RdHi_LoSel = '0' else SDRdDat(15 downto 0);


with uCA(9 downto 0) select


-- New (symmetric with write decoder):
-- bit 15..10 : pad zeros
-- bit  9     : TrigWdCntRst (write-only; read back as '0')   or DatReqBuff_Empty (status)
-- bit  8     : InitReq      (write-only; read back as '0')
-- bit  7     : DDRRd_en
-- bit  6     : PhyDatSel
-- bit  5     : DDRWrt_En
-- bit  4     : '0' (reserved)
-- bit  3     : FMRxEn
-- bit  2     : not PhyPDn
-- bit  1     : LinkRst (write-only; read back as '0')
-- bit  0     : RxBuffRst (write-only; read back as '0')
iCD <= "000000" & DatReqBuff_Empty & '0' & DDRRd_en & PhyDatSel & DDRWrt_En & '0' 
       & FMRxEn & (not PhyPDn) & '0' & RxBuffRst when CSRRegAddr,
		 X"00" & MaskReg when InputMaskAddr,
		 UpTimeStage(31 downto 16) when UpTimeRegAddrHi,
		 UpTimeStage(15 downto 0) when UpTimeRegAddrLo,
		 TestCount(31 downto 16) when TestCounterHiAd,
		 TestCount(15 downto 0) when TestCounterLoAd,
		 DDRWrtStat & X"00" & '0' & DDRRdStat when DDRRdStatAd,
		 "00" & SDWrtAd(29 downto 16) when SDRamWrtPtrHiAd,
		 SDWrtAd(15 downto 0) when SDRamWrtPtrLoAd,
		 "00" & SDRdAD(29 downto 16) when SDRamRdPtrHiAd,
		 SDRdAD(15 downto 0) when SDRamRdPtrLoAd,
		 DDRRd_Mux(7 downto 0) & DDRRd_Mux(15 downto 8) when SDRamSwapPort,
		 DDRRd_Mux when SDRamPortAd,
		 '0' & DDR_Rd_Cnt & '0' & SDwr_count when DDRCountAddr,
		 X"0" & "00" & SDrd_empty & SDrd_full & SDcmd_empty(1) & SDcmd_full(1) 
						 & SDwr_empty & SDwr_full & SDcmd_empty(0) & SDcmd_full(0) 
		 & SDCalDn & SD_RstO when DDRStatAddr,
	    SMIRdReg0 when SMIRdDataAd0,
		 SMIRdReg1 when SMIRdDataAd1,
		 "0000" & PhyRxBuff_RdCnt(0) when PhyRxWdUsedRdAddr(0),
		 "0000" & PhyRxBuff_RdCnt(1) when PhyRxWdUsedRdAddr(1),
		 "0000" & PhyRxBuff_RdCnt(2) when PhyRxWdUsedRdAddr(2),
		 "0000" & PhyRxBuff_RdCnt(3) when PhyRxWdUsedRdAddr(3),
		 "0000" & PhyRxBuff_RdCnt(4) when PhyRxWdUsedRdAddr(4),
		 "0000" & PhyRxBuff_RdCnt(5) when PhyRxWdUsedRdAddr(5),
		 "0000" & PhyRxBuff_RdCnt(6) when PhyRxWdUsedRdAddr(6),
		 "0000" & PhyRxBuff_RdCnt(7) when PhyRxWdUsedRdAddr(7),
		 PhyRxBuff_Out(0) when PhyRxRdAddr(0),
		 PhyRxBuff_Out(1) when PhyRxRdAddr(1),
		 PhyRxBuff_Out(2) when PhyRxRdAddr(2),
		 PhyRxBuff_Out(3) when PhyRxRdAddr(3),
		 PhyRxBuff_Out(4) when PhyRxRdAddr(4),
		 PhyRxBuff_Out(5) when PhyRxRdAddr(5),		 
		 PhyRxBuff_Out(6) when PhyRxRdAddr(6),
	    PhyRxBuff_Out(7) when PhyRxRdAddr(7),
		 X"00" & Rx_active when FEBFMActiveAD,
		 FEBRxBuff_Out(0) when FEBFMRdAddr(0),
		 FEBRxBuff_Out(1) when FEBFMRdAddr(1),
		 FEBRxBuff_Out(2) when FEBFMRdAddr(2),
		 FEBRxBuff_Out(3) when FEBFMRdAddr(3),
		 FEBRxBuff_Out(4) when FEBFMRdAddr(4),
		 FEBRxBuff_Out(5) when FEBFMRdAddr(5),
		 FEBRxBuff_Out(6) when FEBFMRdAddr(6),
		 FEBRxBuff_Out(7) when FEBFMRdAddr(7),
		 X"0" & '0' & FMRxBuff_Count(0) when FEBFMWdsUsedAddr(0),
		 X"0" & '0' & FMRxBuff_Count(1) when FEBFMWdsUsedAddr(1),
		 X"0" & '0' & FMRxBuff_Count(2) when FEBFMWdsUsedAddr(2),
		 X"0" & '0' & FMRxBuff_Count(3) when FEBFMWdsUsedAddr(3),
		 X"0" & '0' & FMRxBuff_Count(4) when FEBFMWdsUsedAddr(4),
		 X"0" & '0' & FMRxBuff_Count(5) when FEBFMWdsUsedAddr(5),
    	 X"0" & '0' & FMRxBuff_Count(6) when FEBFMWdsUsedAddr(6),
		 X"0" & '0' & FMRxBuff_Count(7) when FEBFMWdsUsedAddr(7),
		 FEBRxBuff_Full & FEBRxBuff_Empty when FMRxStatAddr,
		 Rx_CRC_Out(0)(31 downto 16) when RdCRCAddr(0),
		 Rx_CRC_Out(0)(15 downto 0)  when RdCRCAddr(1),
		 Rx_CRC_Out(1)(31 downto 16) when RdCRCAddr(2),
		 Rx_CRC_Out(1)(15 downto 0)  when RdCRCAddr(3),
		 Rx_CRC_Out(2)(31 downto 16) when RdCRCAddr(4),
    	 Rx_CRC_Out(2)(15 downto 0)  when RdCRCAddr(5),
		 Rx_CRC_Out(3)(31 downto 16) when RdCRCAddr(6),
		 Rx_CRC_Out(3)(15 downto 0)  when RdCRCAddr(7),
		 Rx_CRC_Out(4)(31 downto 16) when RdCRCAddr(8),
		 Rx_CRC_Out(4)(15 downto 0)  when RdCRCAddr(9),
		 Rx_CRC_Out(5)(31 downto 16) when RdCRCAddr(10),
		 Rx_CRC_Out(5)(15 downto 0)  when RdCRCAddr(11),
		 Rx_CRC_Out(6)(31 downto 16) when RdCRCAddr(12),
		 Rx_CRC_Out(6)(15 downto 0)  when RdCRCAddr(13),
		 Rx_CRC_Out(7)(31 downto 16) when RdCRCAddr(14),
		 Rx_CRC_Out(7)(15 downto 0)  when RdCRCAddr(15),
		 PhyActivityCounter(0) when PHYActivityCntAdd(0),
		 PhyActivityCounter(1) when PHYActivityCntAdd(1),
		 PhyActivityCounter(2) when PHYActivityCntAdd(2),
		 PhyActivityCounter(3) when PHYActivityCntAdd(3),
		 PhyActivityCounter(4) when PHYActivityCntAdd(4),
		 PhyActivityCounter(5) when PHYActivityCntAdd(5),
		 PhyActivityCounter(6) when PHYActivityCntAdd(6),
		 PhyActivityCounter(7) when PHYActivityCntAdd(7),
		 X"00" & CRCErr_Reg when CRCErrAddr,
		 X"0" & "00" & RxOut(1).Parity_Err & RxOut(0).Parity_Err & PErrStat when FMRxErrAddr,
		 X"000" & '0' & MDIORd & ChainSel when SMICtrlAddr,
		 X"00" & RxErr when RxErrAddr,
		 X"00" & CRS when RxCRSAddr,
		 X"00" & TxEnMask when TxEnMaskAd,
		 X"00" & not PhyRxBuff_Empty when RxDAVAddr,
		 X"000" & "00" & PhyTxBuff_Empty & TxEnAck_sync(1) when PhyTxCSRAddr,
       "00000" & PhyTxBuff_Count when PhyTxCntAddr,
		 X"00" & CurrentTarget when TxCurrentTargetAddr,
		 TrigWdCount & DRegSrc & '0' & Debug when DebugAddr,
		 "00" & SDRdPtr(29 downto 16) when SDRdPtrAddrHi,
		 SDRdPtr(15 downto 0) when SDRdPtrAddrLo,
       "0000000" & TxFIFOTrace_Out when LinkTxTraceAd,	
		 LinkTxFullCnt & "00" & LinkTxFull & LinkTxEmpty & 
				           "000" & LinkStatEn when LinkCtrlAd,		
		 tx_overflow_cnt when OverflowCntAd,
		 (15 downto 1 => '0') & PhyTxBuff_Empty when TxFifoRawEmptyAddr,
		 X"00" & LastTxTarget  when LastTxTargetAddr,
                 X"0011" when DebugVersion,		
		 X"00" & AutoTx_TimeoutCnt(0) when AutoTxTimeoutCntAd0,
		 X"00" & AutoTx_TimeoutCnt(1) when AutoTxTimeoutCntAd1,
		 X"00" & AutoTx_TimeoutCnt(2) when AutoTxTimeoutCntAd2,
		 X"00" & AutoTx_TimeoutCnt(3) when AutoTxTimeoutCntAd3,
       X"00" & AutoTx_TimeoutCnt(4) when AutoTxTimeoutCntAd4,
		 X"00" & AutoTx_TimeoutCnt(5) when AutoTxTimeoutCntAd5,
		 X"00" & AutoTx_TimeoutCnt(6) when AutoTxTimeoutCntAd6,
		 X"00" & AutoTx_TimeoutCnt(7) when AutoTxTimeoutCntAd7,
		 X"0000"              when ReadyClearAddr,   -- write-only, read returns 0                                
       X"0000"              when ReadyForceAddr,   -- write-only, read returns 0
		 X"00" & ReadyStatus when ReadyStatusAddr,
		 X"0000" when others;



uCD <= iCD when uCRd = '0' and CpldCS = '0' and uCA(11 downto 10) = GA 
		 else (others => 'Z');

end behavioural;
