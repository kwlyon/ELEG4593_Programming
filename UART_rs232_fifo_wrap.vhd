-- ============================================================================
-- File: uart_rs232_fifo_wrap.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 04 March 2026
--
-- Description:
--   Wrapper around uart_rs232 that adds simple RX and TX FIFOs.
--   This module decouples the UART core from downstream logic by:
--     - buffering received bytes (RX FIFO)
--     - buffering bytes to transmit (TX FIFO)
--
--   RX Operation:
--     - Accepts UART RX bytes when RX FIFO is not full.
--     - Presents FIFO output as rxq_data.
--     - User logic pops bytes with a 1-cycle pulse on rxq_rd_en.
--
--   TX Operation:
--     - User logic pushes bytes with a 1-cycle pulse on txq_wr_en.
--     - When TX FIFO is non-empty, uart_tx_valid is asserted continuously.
--     - FIFO is popped only when the UART asserts tx_ready (TX idle),
--       ensuring bytes transmit back-to-back as fast as the UART allows.
--
-- Interfaces:
--   - Connects directly to: uart_rs232.vhd
--   - FIFO implementation: My_Own_FN_FIFO.vhd
--
-- Notes:
--   - Single-clock domain: clk_sys
--   - Active-high synchronous reset: reset_i
--   - FIFO push/pop strobes are assumed to be single-clock pulses.
--
-- Revision History:
--   2026-03-04  - Initial version.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_rs232_fifo_wrap is
  generic (
    CLK_FREQ_HZ : integer := 24930000;
    BAUD        : integer := 9600;
    DATA_BITS   : integer := 8;
    STOP_BITS   : integer := 1;
    PARITY      : integer := 0
  );
  port (
    clk_sys : in  std_logic;
    reset_i : in  std_logic;  -- active-high synchronous reset

    -- Serial pins
    RX : in  std_logic;
    TX : out std_logic;

    -- =========================
    -- RX FIFO (bytes received)
    -- =========================
    rxq_data  : out std_logic_vector(7 downto 0);  -- current FIFO output
    rxq_empty : out std_logic;
    rxq_full  : out std_logic;
    rxq_rd_en : in  std_logic;                     -- pulse 1 clk to pop

    -- =========================
    -- TX FIFO (bytes to send)
    -- =========================
    txq_data_in : in  std_logic_vector(7 downto 0); -- byte to enqueue for TX
    txq_wr_en   : in  std_logic;                    -- pulse 1 clk to push
    txq_empty   : out std_logic;
    txq_full    : out std_logic
  );
end entity;

architecture rtl of uart_rs232_fifo_wrap is

  ---------------------------------------------------------------------------
  -- FIFO depth
  -- Keep these comfortably larger than a single command/response packet so
  -- short host bursts do not overflow the software-facing queues.
  ---------------------------------------------------------------------------
  constant C_RX_FIFO_DEPTH : integer := 256;
  constant C_TX_FIFO_DEPTH : integer := 256;

  ---------------------------------------------------------------------------
  -- UART signals
  ---------------------------------------------------------------------------
  signal uart_rx_data        : std_logic_vector(7 downto 0);
  signal uart_rx_valid       : std_logic;
  signal uart_rx_ready       : std_logic;
  signal uart_rx_framing_err : std_logic;
  signal uart_rx_parity_err  : std_logic;
  signal uart_rx_overrun     : std_logic;

  signal uart_tx_data  : std_logic_vector(7 downto 0);
  signal uart_tx_valid : std_logic;
  signal uart_tx_ready : std_logic;

  ---------------------------------------------------------------------------
  -- RX FIFO signals (your FIFO)
  ---------------------------------------------------------------------------
  signal rx_fifo_q     : std_logic_vector(7 downto 0);
  signal rx_fifo_empty : std_logic;
  signal rx_fifo_full  : std_logic;
  signal rx_fifo_push  : std_logic;

  ---------------------------------------------------------------------------
  -- TX FIFO signals (your FIFO)
  ---------------------------------------------------------------------------
  signal tx_fifo_q     : std_logic_vector(7 downto 0);
  signal tx_fifo_empty : std_logic;
  signal tx_fifo_full  : std_logic;
  signal tx_fifo_pop   : std_logic;

begin

  ---------------------------------------------------------------------------
  -- Export FIFO status/data to top level
  ---------------------------------------------------------------------------
  rxq_data  <= rx_fifo_q;
  rxq_empty <= rx_fifo_empty;
  rxq_full  <= rx_fifo_full;

  txq_empty <= tx_fifo_empty;
  txq_full  <= tx_fifo_full;

  ---------------------------------------------------------------------------
  -- RX path: UART -> RX FIFO
  -- Only accept UART byte when FIFO not full.
  ---------------------------------------------------------------------------
  uart_rx_ready <= not rx_fifo_full;

  -- Push into RX FIFO when UART asserts rx_valid and we are ready
  rx_fifo_push <= uart_rx_valid and uart_rx_ready;

  ---------------------------------------------------------------------------
  -- TX path: TX FIFO -> UART (send ASAP)
  -- Present FIFO Q as uart_tx_data.
  -- Assert uart_tx_valid whenever TX FIFO not empty.
  -- Pop FIFO only when UART is ready to accept (handshake).
  ---------------------------------------------------------------------------
  uart_tx_data  <= tx_fifo_q;
  uart_tx_valid <= not tx_fifo_empty;

  tx_fifo_pop <= uart_tx_ready and (not tx_fifo_empty);

  ---------------------------------------------------------------------------
  -- Instantiate UART
  ---------------------------------------------------------------------------
  u_uart : entity work.uart_rs232
    generic map (
      CLK_FREQ_HZ => CLK_FREQ_HZ,
      BAUD        => BAUD,
      DATA_BITS   => DATA_BITS,
      STOP_BITS   => STOP_BITS,
      PARITY      => PARITY
    )
    port map (
      clk   => clk_sys,
      reset => reset_i,

      rx    => RX,
      tx    => TX,

      rx_data        => uart_rx_data,
      rx_valid       => uart_rx_valid,
      rx_ready       => uart_rx_ready,
      rx_framing_err => uart_rx_framing_err,
      rx_parity_err  => uart_rx_parity_err,
      rx_overrun     => uart_rx_overrun,

      tx_data  => uart_tx_data,
      tx_valid => uart_tx_valid,
      tx_ready => uart_tx_ready
    );

  ---------------------------------------------------------------------------
  -- Instantiate RX FIFO (your FIFO)
  ---------------------------------------------------------------------------
  u_rx_fifo : entity work.My_Own_FN_FIFO
    generic map (
      WIDTH => 8,
      DEPTH => C_RX_FIFO_DEPTH
    )
    port map (
      clk   => clk_sys,
      rst   => reset_i,

      push  => rx_fifo_push,
      din   => uart_rx_data,

      pop   => rxq_rd_en,
      dout  => rx_fifo_q,

      empty => rx_fifo_empty,
      full  => rx_fifo_full
    );

  ---------------------------------------------------------------------------
  -- Instantiate TX FIFO (your FIFO)
  ---------------------------------------------------------------------------
  u_tx_fifo : entity work.My_Own_FN_FIFO
    generic map (
      WIDTH => 8,
      DEPTH => C_TX_FIFO_DEPTH
    )
    port map (
      clk   => clk_sys,
      rst   => reset_i,

      push  => txq_wr_en,
      din   => txq_data_in,

      pop   => tx_fifo_pop,
      dout  => tx_fifo_q,

      empty => tx_fifo_empty,
      full  => tx_fifo_full
    );

end architecture;
