-- ============================================================================
-- File: uart_spram_top.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 03 March 2026
-- Last Updated: 07 April 2026
--
-- Description:
--   Top-level integration for a UART-controlled SPRAM memory interface
--   implemented on the Lattice MachXO3D FPGA.
--
--   Earlier revisions of this design directly connected the UART protocol
--   parser and memory execution logic inside Bus_Master.vhd. The design
--   has since been refactored to separate protocol handling from the
--   memory arbitration layer.
--
--   The system is currently organized into the following components:
--
--       uart_rs232_fifo_wrap_controller.vhd
--           Handles UART RX/TX FIFOs, packet parsing, checksum validation,
--           command decoding, and response packet generation.
--
--       pwm_spram_client.vhd
--           Highest-priority memory-mapped PWM peripheral. Periodically polls
--           a block of SPRAM control registers and drives four PWM outputs
--           using locally latched shadow registers.
--
--       adc_spram_client.vhd
--           Middle-priority memory-mapped ADC peripheral. Periodically starts
--           AD7928 conversions and writes the latest valid sample into SPRAM
--           through the shared Bus_Master interface.
--
--       Bus_Master.vhd
--           Dedicated prioritized SPRAM arbitration wrapper that services
--           read/write requests from multiple clients.
--
--       SPRAM.vhd
--           SCUBA-generated MachXO3D on-chip SPRAM block.
--
--       pll_24m93.vhd
--           PLL generated from the MachXO3D internal oscillator.
--
--       reset_delay.vhd
--           Holds global reset active for an exact number of clock cycles
--           after the PLL lock signal is achieved.
--
--   This separation allows the Bus_Master module to be reused in future
--   projects as a general-purpose memory arbitration layer while keeping
--   UART protocol handling isolated to the controller module. It also allows
--   memory-mapped peripherals, such as PWM and ADC, to source and store their
--   operating values directly through SPRAM without depending on any one
--   writer.
--
--
-- Top-level integration:
--
--   - MachXO3D internal oscillator
--         clock.vhd
--
--   - PLL clock generator
--         pll_24m93.vhd
--
--   - Delayed reset release
--         reset_delay.vhd
--
--   - UART controller with RX/TX FIFOs and packet protocol
--         uart_rs232_fifo_wrap_controller.vhd
--
--   - Memory-mapped 4-channel PWM client
--         pwm_spram_client.vhd
--
--   - Memory-mapped ADC client
--         adc_spram_client.vhd
--
--   - Prioritized SPRAM wrapper / arbiter
--         Bus_Master.vhd
--
--   - On-chip SPRAM block
--         SPRAM.vhd
--
--
-- Notes:
--
--   - SPRAM is 1024 x 16 words.
--   - Address bus width is therefore 10 bits.
--   - PWM control registers begin at SPRAM address 0x0100.
--   - PWM duty-cycle registers are 16 bits wide.
--   - ADC sample registers begin at SPRAM address 0x0200 and are written to
--     sequential words by channel number.
--   - Client 0 of Bus_Master is assigned to the PWM client so PWM refresh
--     traffic has highest priority.
--   - Client 1 of Bus_Master is assigned to the ADC client.
--   - Client 2 of Bus_Master is assigned to the UART controller.
--   - Global reset is held active while PLL is unlocked.
--   - After PLL lock is asserted, reset_delay.vhd holds reset active for
--     exactly RESET_DELAY_CYCLES additional clk_sys cycles.
--   - PWM client outputs are internally active-high and inverted at the
--     top-level outputs for compatibility with active-low LEDs.
--   - LED_4 and LED_5 are held inactive high to suppress faint glow.
--   - LED_6 and LED_7 mirror the RX and TX UART lines for activity indication.
--
--
-- Revision History:
--
--   2026-03-04
--     - Fixed FSM timing issues caused by signal update latency when
--       assembling address and data words.
--     - Added intermediate FSM states to safely capture UART bytes
--       before using them in memory operations.
--     - Corrected write bug where the low data byte was written with
--       a stale value.
--
--   2026-03-05
--     - Investigated RX FIFO behavior where the Empty flag did not
--       update as expected during command parsing.
--     - Replaced vendor FIFO implementation with a custom synchronous
--       FIFO module (My_Own_FN_FIFO.vhd).
--     - Updated UART FIFO wrapper to use the custom FIFO for both RX
--       and TX data paths.
--
--   2026-03-16
--     - Major architectural refactor.
--     - Separated UART packet protocol logic from memory arbitration.
--     - Introduced uart_rs232_fifo_wrap_controller.vhd to handle
--       packet parsing, command execution, and response generation.
--     - Bus_Master.vhd redesigned as a reusable SPRAM arbitration layer.
--     - Top-level updated to connect UART controller -> Bus_Master -> SPRAM.
--     - Replaced long power-up counter reset with PLL-lock-based delayed reset.
--     - Added N-cycle reset release delay.
--
--   2026-03-18
--     - Refactored Bus_Master client wiring to use structured request and
--       response records defined in bus_pkg.vhd.
--     - Updated the UART controller and reserved client interface signals
--       from flat bundles to t_bus_req / t_bus_rsp records.
--     - No intended functional change to top-level system behavior.
--
--   2026-03-24
--     - Added pwm_spram_client.vhd as a memory-mapped PWM peripheral.
--     - Reserved SPRAM address 0x0004 as the PWM duty-cycle register.
--     - Added top-level PWM output port.
--     - Reassigned Bus_Master client priority so PWM is client 0 and UART
--       controller is client 1.
--     - Added periodic PWM duty refresh from SPRAM using a 10-bit PWM value.
--
--   2026-04-05
--     - Removed legacy debug LED output port and associated top-level wiring.
--     - Removed UART controller LED debug signal routing.
--     - Expanded top-level PWM interface from one output to eight outputs:
--       PWM_0 through PWM_7.
--     - Updated PWM client generics to use 16-bit duty-cycle operation and
--       the expanded control register block beginning at 0x0100.
--     - No intended functional change to UART or SPRAM arbitration behavior.
--
--   2026-04-06
--     - Added internal PWM raw signals.
--     - Inverted top-level PWM outputs for active-low LED drive.
--
--   2026-04-07
--     - Reduced top-level PWM outputs from PWM_0 through PWM_7 to
--       PWM_0 through PWM_3 only.
--     - Removed PWM_4 through PWM_7 ports and associated top-level wiring.
--     - Added adc_spram_client.vhd to the top-level design.
--     - Added SPI top-level ports for the AD7928 interface.
--     - Reassigned Bus_Master client priority so PWM is client 0,
--       ADC is client 1, and UART controller is client 2.
--     - Added LED_4 and LED_5 outputs tied high to suppress faint glow.
--     - Added LED_6 and LED_7 outputs tied to RX and TX for UART activity.
--     - Added internal UART TX signal so TX activity can also drive LED_7
--       without reading back an output port.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.bus_pkg.all;

entity uart_spram_top is
  generic (
    OSC_FREQ           : string  := "8.31";     -- MHz internal oscillator setting
    CLK_FREQ_HZ        : integer := 24930000;   -- System clock after PLL
    BAUD               : integer := 9600;
    DATA_BITS          : integer := 8;
    STOP_BITS          : integer := 1;
    PARITY             : integer := 0;
    RESET_DELAY_CYCLES : integer := 16          -- extra clocks held in reset after PLL lock
  );
  port (
    RX       : in  std_logic;
    TX       : out std_logic;

    SPI_Dout : in  std_logic;
    SPI_clk  : out std_logic;
    SPI_Din  : out std_logic;
    SPI_CSn  : out std_logic;

    PWM_0    : out std_logic;
    PWM_1    : out std_logic;
    PWM_2    : out std_logic;
    PWM_3    : out std_logic;

    LED_4    : out std_logic;
    LED_5    : out std_logic;
    LED_6    : out std_logic;
    LED_7    : out std_logic
  );
end entity;

architecture rtl of uart_spram_top is

  ---------------------------------------------------------------------------
  -- Clocking signals
  ---------------------------------------------------------------------------
  signal clk_osc      : std_logic;
  signal clk_sys      : std_logic;
  signal pll_lock     : std_logic;
  signal pll_not_lock : std_logic;

  ---------------------------------------------------------------------------
  -- Global synchronous reset
  ---------------------------------------------------------------------------
  signal reset_i : std_logic := '1';

  ---------------------------------------------------------------------------
  -- Internal UART TX signal
  ---------------------------------------------------------------------------
  signal tx_uart : std_logic := '1';

  ---------------------------------------------------------------------------
  -- Bus_Master client 0 interface
  -- Highest priority: PWM SPRAM client
  ---------------------------------------------------------------------------
  signal c0_req : t_bus_req := (
    req   => '0',
    we    => '0',
    addr  => (others => '0'),
    wdata => (others => '0')
  );

  signal c0_rsp : t_bus_rsp := (
    ack   => '0',
    rdata => (others => '0')
  );

  ---------------------------------------------------------------------------
  -- Bus_Master client 1 interface
  -- Middle priority: ADC SPRAM client
  ---------------------------------------------------------------------------
  signal c1_req : t_bus_req := (
    req   => '0',
    we    => '0',
    addr  => (others => '0'),
    wdata => (others => '0')
  );

  signal c1_rsp : t_bus_rsp := (
    ack   => '0',
    rdata => (others => '0')
  );

  ---------------------------------------------------------------------------
  -- Bus_Master client 2 interface
  -- Lowest priority: UART controller
  ---------------------------------------------------------------------------
  signal c2_req : t_bus_req := (
    req   => '0',
    we    => '0',
    addr  => (others => '0'),
    wdata => (others => '0')
  );

  signal c2_rsp : t_bus_rsp := (
    ack   => '0',
    rdata => (others => '0')
  );

  ---------------------------------------------------------------------------
  -- SPRAM interface signals (Bus_Master / SPRAM core)
  ---------------------------------------------------------------------------
  signal mem_addr : std_logic_vector(9 downto 0);
  signal mem_din  : std_logic_vector(15 downto 0);
  signal mem_dout : std_logic_vector(15 downto 0);
  signal mem_we   : std_logic;
  signal mem_en   : std_logic;

  ---------------------------------------------------------------------------
  -- Internal PWM signals before active-low output inversion
  ---------------------------------------------------------------------------
  signal pwm_raw : std_logic_vector(3 downto 0);

  ---------------------------------------------------------------------------
  -- PWM configuration constants
  ---------------------------------------------------------------------------
  constant C_PWM_WIDTH       : integer := 12;
  constant C_PWM_DIVIDER     : integer := 1;  -- Let's get us back up to ~380 Hz
  constant C_REG_BASE_ADDR   : integer := 16#0100#;
  constant C_PWM_PERIOD_CLKS : integer := C_PWM_DIVIDER * (2 ** C_PWM_WIDTH);
  constant C_POLL_CYCLES     : integer := C_PWM_PERIOD_CLKS;

  ---------------------------------------------------------------------------
  -- ADC configuration constants
  ---------------------------------------------------------------------------
  --Channel 0 is ADC_Vin
  --Channel 1 is ADC_VBuck
  --Channel 2 is ADC_VBoost
  --Channel 3 is ADC_VPot
  constant C_ADC_CLK_DIV        : integer := 5;
  constant C_SAMPLE_PERIOD_CLKS : integer := C_PWM_PERIOD_CLKS;
  constant C_ADC_BASE_ADDR      : integer := 16#0200#;

begin

  pll_not_lock <= not pll_lock;

  ---------------------------------------------------------------------------
  -- UART top-level output
  ---------------------------------------------------------------------------
  TX <= tx_uart;

  ---------------------------------------------------------------------------
  -- Active-low LED output inversion
  ---------------------------------------------------------------------------
  PWM_0 <= not pwm_raw(0);
  PWM_1 <= not pwm_raw(1);
  PWM_2 <= not pwm_raw(2);
  PWM_3 <= not pwm_raw(3);

  ---------------------------------------------------------------------------
  -- Additional LED assignments
  ---------------------------------------------------------------------------
  LED_4 <= '1';
  LED_5 <= '1';
  LED_6 <= RX;
  LED_7 <= tx_uart;

  ---------------------------------------------------------------------------
  -- Internal oscillator clock module
  ---------------------------------------------------------------------------
  u_clk : entity work.clock
    generic map (
      OSC_FREQ => OSC_FREQ
    )
    port map (
      clk => clk_osc
    );

  ---------------------------------------------------------------------------
  -- PLL:
  -- Convert internal oscillator clock to the desired system clock
  ---------------------------------------------------------------------------
  u_pll : entity work.pll_24m93
    port map (
      CLKI  => clk_osc,
      CLKOP => clk_sys,
      LOCK  => pll_lock
    );

  ---------------------------------------------------------------------------
  -- Reset delay
  ---------------------------------------------------------------------------
  u_reset_delay : entity work.reset_delay
    generic map (
      DELAY_CYCLES => RESET_DELAY_CYCLES
    )
    port map (
      clk     => clk_sys,
      rst_in  => pll_not_lock,
      rst_out => reset_i
    );

  ---------------------------------------------------------------------------
  -- PWM SPRAM client
  -- Highest-priority Bus_Master client
  ---------------------------------------------------------------------------
  u_pwm_client : entity work.pwm_spram_client
    generic map (
      CLK_FREQ_HZ   => CLK_FREQ_HZ,
      PWM_WIDTH     => C_PWM_WIDTH,
      PWM_DIVIDER   => C_PWM_DIVIDER,
      REG_BASE_ADDR => C_REG_BASE_ADDR,
      POLL_CYCLES   => C_POLL_CYCLES
    )
    port map (
      clk_sys => clk_sys,
      reset_i => reset_i,

      bus_req => c0_req,
      bus_rsp => c0_rsp,

      PWM_0 => pwm_raw(0),
      PWM_1 => pwm_raw(1),
      PWM_2 => pwm_raw(2),
      PWM_3 => pwm_raw(3)
    );

  ---------------------------------------------------------------------------
  -- ADC SPRAM client
  -- Middle-priority Bus_Master client
  ---------------------------------------------------------------------------
  u_adc_client : entity work.adc_spram_client
    generic map (
      CLK_FREQ_HZ        => CLK_FREQ_HZ,
      ADC_CLK_DIV        => C_ADC_CLK_DIV,
      SAMPLE_PERIOD_CLKS => C_SAMPLE_PERIOD_CLKS,
      ADC_BASE_ADDR      => C_ADC_BASE_ADDR
    )
    port map (
      clk_sys => clk_sys,
      reset_i => reset_i,

      bus_req => c1_req,
      bus_rsp => c1_rsp,

      adc_cs_n_o => SPI_CSn,
      adc_sclk_o => SPI_clk,
      adc_din_o  => SPI_Din,
      adc_dout_i => SPI_Dout
    );

  ---------------------------------------------------------------------------
  -- UART protocol controller
  -- Lowest-priority Bus_Master client
  ---------------------------------------------------------------------------
  u_uart_ctrl : entity work.uart_rs232_fifo_wrap_controller
    generic map (
      CLK_FREQ_HZ => CLK_FREQ_HZ,
      BAUD        => BAUD,
      DATA_BITS   => DATA_BITS,
      STOP_BITS   => STOP_BITS,
      PARITY      => PARITY
    )
    port map (
      clk_sys => clk_sys,
      reset_i => reset_i,

      RX => RX,
      TX => tx_uart,

      bus_req => c2_req,
      bus_rsp => c2_rsp
    );

  ---------------------------------------------------------------------------
  -- Dedicated SPRAM wrapper / arbiter
  -- Priority: client 0 > client 1 > client 2
  ---------------------------------------------------------------------------
  u_bus : entity work.Bus_Master
    port map (
      clk_sys => clk_sys,
      reset_i => reset_i,

      -- Client 0: PWM client
      c0_req => c0_req,
      c0_rsp => c0_rsp,

      -- Client 1: ADC client
      c1_req => c1_req,
      c1_rsp => c1_rsp,

      -- Client 2: UART controller
      c2_req => c2_req,
      c2_rsp => c2_rsp,

      -- SPRAM side
      mem_addr => mem_addr,
      mem_din  => mem_din,
      mem_dout => mem_dout,
      mem_we   => mem_we,
      mem_en   => mem_en
    );

  ---------------------------------------------------------------------------
  -- SPRAM core (SCUBA generated)
  ---------------------------------------------------------------------------
  u_spram : entity work.SPRAM
    port map (
      Address => mem_addr,
      Data    => mem_din,
      Clock   => clk_sys,
      WE      => mem_we,
      ClockEn => mem_en,
      Q       => mem_dout
    );

end architecture;
