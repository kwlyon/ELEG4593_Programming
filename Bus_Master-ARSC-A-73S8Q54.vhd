-- ============================================================================
-- File: Bus_Master.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 04 March 2026
--
-- Description:
--   Bus master finite state machine implementing a packet-based serial
--   protocol for reading and writing the MachXO3D SPRAM via a UART interface.
--
--   This version replaces the earlier immediate byte-by-byte parser with:
--
--     1) A buffered packet receiver
--     2) Packet framing validation using LEN + CR
--     3) Packet checksum validation
--     4) A command execution engine
--     5) A dedicated TX packet formatter / sender FSM
--
--   Incoming packet format:
--
--     LEN | PAYLOAD... | CHKSUM | CR
--
--   where:
--     - LEN is the number of bytes following LEN up to and including CHKSUM
--     - LEN does NOT include itself
--     - LEN does NOT include the terminating CR
--     - CHKSUM is computed over PAYLOAD bytes only
--     - CR must arrive immediately after exactly LEN bytes
--
--   Request packet formats:
--
--     Read Request:
--       LEN | CMD | ADDR_H | ADDR_L | CHKSUM | CR
--       LEN = 0x04
--       CMD = 'R' (0x52)
--
--     Write Request:
--       LEN | CMD | ADDR_H | ADDR_L | DATA_H | DATA_L | CHKSUM | CR
--       LEN = 0x06
--       CMD = 'W' (0x57)
--
--   Response packet formats:
--
--     Read Response:
--       LEN | DATA_H | DATA_L | CHKSUM | CR
--       LEN = 0x03
--
--     Write ACK Response:
--       LEN | ACK | CHKSUM | CR
--       LEN = 0x02
--
--   Checksum:
--     - 8-bit XOR checksum
--     - Computed over payload bytes only
--     - LEN is NOT included
--     - CHKSUM is NOT included
--     - CR is NOT included
--     - RX checksum is accumulated incrementally as bytes are captured
--       rather than recomputed later from the receive buffer.  This avoids
--       missing the final payload byte due to signal update timing.
--     - TX checksum is accumulated incrementally while the TX FSM builds
--       a complete outgoing frame buffer.
--
--   Receiver behavior:
--     - First received byte is interpreted as LEN
--     - Exactly LEN bytes are collected into an internal receive buffer
--     - The next byte must be CR
--     - If CR arrives early, late, or not at all when expected, packet is discarded
--     - If checksum fails, packet is discarded
--     - Only valid packets are executed
--
--   Transmitter behavior:
--     - TX FSM accepts payload bytes only from RX FSM
--     - TX FSM builds a complete frame buffer:
--           LEN | PAYLOAD... | CHKSUM | CR
--     - Checksum is accumulated as payload bytes are inserted into the TX frame
--     - TX FSM then sends the completed frame buffer byte-by-byte into FIFO
--
--   Addressing:
--     - Protocol uses a 16-bit address field
--     - SPRAM is 1024 x 16 words, so only bits [9:0] are used
--     - Currently no test for valid memory range--it will try to write to FFFF
--
--   Debugging:
--     - LEDn displays the current RX FSM state
--
-- Revision History:
--
--   2026-03-04
--     - Initial implementation of UART command parser FSM.
--     - Added RX FIFO pop/capture mechanism using pop_pending.
--     - Implemented read command path and TX response logic.
--
--   2026-03-05
--     - Updated for custom FIFO in UART Wrapper.
--     - Added intermediate FSM states to safely capture address and
--       data bytes before using them in logic decisions.
--     - Implemented addr16_next staging register to prevent stale address.
--     - Corrected bug where the low data byte could be written
--       with a stale value due to signal update timing.
--     - Final fix: mem_din assembled directly as
--           wdata(15 downto 8) & rx_byte
--       ensuring correct word writes to SPRAM.
--
--   2026-03-09
--     - Reworked protocol to packet-based framing using LEN, CHKSUM, and CR.
--     - Added buffered packet receiver with explicit POP / CAPTURE states.
--     - Added checksum validation before command execution.
--     - Added dedicated TX packet FSM for formatted replies.
--     - Added reusable checksum function for both RX and TX packet handling.
--     - Rewritten in a more behavioral / readable style.
--     - Updated write path to use a two-cycle write sequence:
--         RX_WRITE_SETUP  -> present address/data
--         RX_WRITE_COMMIT -> pulse mem_en/mem_we
--       to improve SPRAM write robustness.
--
--   2026-03-10
--     - Changed RX checksum validation to a running XOR accumulated as
--       bytes are captured.
--     - This avoids post-buffer checksum timing issues where the final
--       payload byte could be missing from the computed checksum.
--
--   2026-03-11
--     - Reworked TX path to build a full outgoing frame buffer.
--     - TX checksum is now accumulated incrementally as payload bytes are
--       copied into the TX frame buffer.
--     - TX then transmits the finished frame buffer byte-by-byte into the
--       outgoing FIFO.
--
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Bus_Master is
  port (
    clk_sys : in  std_logic;
    reset_i : in  std_logic;

    LEDn : out std_logic_vector(7 downto 0);

    rxq_data  : in  std_logic_vector(7 downto 0);
    rxq_empty : in  std_logic;
    rxq_full  : in  std_logic;
    rxq_rd_en : out std_logic;

    txq_data_in : out std_logic_vector(7 downto 0);
    txq_wr_en   : out std_logic;
    txq_empty   : in  std_logic;
    txq_full    : in  std_logic;

    mem_addr : out std_logic_vector(9 downto 0);
    mem_din  : out std_logic_vector(15 downto 0);
    mem_dout : in  std_logic_vector(15 downto 0);
    mem_we   : out std_logic;
    mem_en   : out std_logic
  );
end entity;

architecture behavioral of Bus_Master is

  -----------------------------------------------------------------------------
  -- Protocol constants
  -----------------------------------------------------------------------------
  constant C_CMD_W : std_logic_vector(7 downto 0) := x"57"; -- 'W'
  constant C_CMD_R : std_logic_vector(7 downto 0) := x"52"; -- 'R'
  constant C_ACK   : std_logic_vector(7 downto 0) := x"06";
  constant C_CR    : std_logic_vector(7 downto 0) := x"0D";

  constant C_LEN_READ_REQ  : integer := 4; -- CMD ADDR_H ADDR_L CHKSUM
  constant C_LEN_WRITE_REQ : integer := 6; -- CMD ADDR_H ADDR_L DATA_H DATA_L CHKSUM
  constant C_LEN_READ_RSP  : integer := 3; -- DATA_H DATA_L CHKSUM
  constant C_LEN_ACK_RSP   : integer := 2; -- ACK CHKSUM

  constant C_MAX_PKT_BYTES   : integer := 16;
  constant C_MAX_FRAME_BYTES : integer := C_MAX_PKT_BYTES + 2; -- LEN + LEN-bytes + CR

  -----------------------------------------------------------------------------
  -- Byte array types used for RX/TX buffers
  -----------------------------------------------------------------------------
  type t_byte_array is array (0 to C_MAX_PKT_BYTES-1) of std_logic_vector(7 downto 0);
  type t_frame_array is array (0 to C_MAX_FRAME_BYTES-1) of std_logic_vector(7 downto 0);

  -----------------------------------------------------------------------------
  -- Checksum function
  -----------------------------------------------------------------------------
  function calc_checksum(
    buf   : t_byte_array;
    count : integer
  ) return std_logic_vector is
    variable chk : std_logic_vector(7 downto 0) := (others => '0');
  begin
    for i in 0 to count-1 loop
      chk := chk xor buf(i);
    end loop;
    return chk;
  end function;

  -----------------------------------------------------------------------------
  -- RX / command execution FSM
  -----------------------------------------------------------------------------
  type t_rx_state is (
    RX_IDLE,
    RX_LEN_POP,
    RX_LEN_CAP,
    RX_BYTE_POP,
    RX_BYTE_CAP,
    RX_CR_POP,
    RX_CR_CAP,
    RX_VALIDATE,
    RX_DECODE,
    RX_READ_REQ,
    RX_READ_WAIT,
    RX_WRITE_SETUP,
    RX_WRITE_COMMIT,
    RX_PREP_READ_RSP,
    RX_PREP_ACK_RSP,
    RX_CLEAR
  );

  -----------------------------------------------------------------------------
  -- TX packet sender FSM
  -----------------------------------------------------------------------------
  type t_tx_state is (
    TX_IDLE,
    TX_LATCH,
    TX_BUILD,
    TX_FINALIZE,
    TX_SEND
  );

  -----------------------------------------------------------------------------
  -- State registers
  -----------------------------------------------------------------------------
  signal rx_state : t_rx_state := RX_IDLE;
  signal tx_state : t_tx_state := TX_IDLE;

  -----------------------------------------------------------------------------
  -- Shared FIFO pop/capture mechanism for RX
  -----------------------------------------------------------------------------
  signal pop_pending : std_logic := '0';
  signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');

  -----------------------------------------------------------------------------
  -- RX packet buffer and bookkeeping
  -----------------------------------------------------------------------------
  signal rx_buf           : t_byte_array;
  signal rx_expected_len  : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal rx_index         : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal rx_checksum_rx   : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_checksum_calc : std_logic_vector(7 downto 0) := (others => '0');

  -----------------------------------------------------------------------------
  -- Decoded fields
  -----------------------------------------------------------------------------
  signal cmd       : std_logic_vector(7 downto 0)  := (others => '0');
  signal addr16    : std_logic_vector(15 downto 0) := (others => '0');
  signal addr_word : std_logic_vector(9 downto 0)  := (others => '0');
  signal wdata     : std_logic_vector(15 downto 0) := (others => '0');
  signal rdata     : std_logic_vector(15 downto 0) := (others => '0');

  -----------------------------------------------------------------------------
  -- TX payload interface from RX FSM
  -----------------------------------------------------------------------------
  signal tx_payload_buf : t_byte_array;
  signal tx_payload_len : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal tx_start       : std_logic := '0';
  signal tx_busy        : std_logic := '0';

  -----------------------------------------------------------------------------
  -- TX local build/send registers
  -----------------------------------------------------------------------------
  signal tx_payload_local   : t_byte_array;
  signal tx_payload_len_loc : integer range 0 to C_MAX_PKT_BYTES := 0;

  signal tx_frame_buf     : t_frame_array;
  signal tx_frame_len     : integer range 0 to C_MAX_FRAME_BYTES := 0;
  signal tx_build_index   : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal tx_send_index    : integer range 0 to C_MAX_FRAME_BYTES := 0;
  signal tx_checksum_run  : std_logic_vector(7 downto 0) := (others => '0');

  -----------------------------------------------------------------------------
  -- Debug LEDs
  -----------------------------------------------------------------------------
  signal led_buff : std_logic_vector(7 downto 0) := (others => '0');

begin

  LEDn <= led_buff;

  -----------------------------------------------------------------------------
  -- LED STATE DECODE
  -- Displays current RX FSM state
  -----------------------------------------------------------------------------
  p_led_decode : process(rx_state, reset_i)
  begin
    if reset_i = '1' then
      led_buff <= (others => '0');
    else
      case rx_state is
        when RX_IDLE          => led_buff <= "10000000";
        when RX_LEN_POP       => led_buff <= "01000000";
        when RX_LEN_CAP       => led_buff <= "00100000";
        when RX_BYTE_POP      => led_buff <= "00010000";
        when RX_BYTE_CAP      => led_buff <= "00001000";
        when RX_CR_POP        => led_buff <= "00000100";
        when RX_CR_CAP        => led_buff <= "00000010";
        when RX_VALIDATE      => led_buff <= "00000001";
        when RX_DECODE        => led_buff <= "00000011";
        when RX_READ_REQ      => led_buff <= "00000111";
        when RX_READ_WAIT     => led_buff <= "00001111";
        when RX_WRITE_SETUP   => led_buff <= "00011111";
        when RX_WRITE_COMMIT  => led_buff <= "00111111";
        when RX_PREP_READ_RSP => led_buff <= "01111111";
        when RX_PREP_ACK_RSP  => led_buff <= "01010101";
        when RX_CLEAR         => led_buff <= "11111111";
        when others           => led_buff <= (others => '0');
      end case;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- RX / EXECUTION FSM
  -----------------------------------------------------------------------------
  p_rx_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then

      -------------------------------------------------------------------------
      -- Default one-cycle strobes
      -------------------------------------------------------------------------
      rxq_rd_en <= '0';
      mem_en    <= '0';
      mem_we    <= '0';
      tx_start  <= '0';

      if reset_i = '1' then
        rx_state          <= RX_IDLE;
        pop_pending       <= '0';
        rx_byte           <= (others => '0');
        rx_expected_len   <= 0;
        rx_index          <= 0;
        rx_checksum_rx    <= (others => '0');
        rx_checksum_calc  <= (others => '0');
        cmd               <= (others => '0');
        addr16            <= (others => '0');
        addr_word         <= (others => '0');
        wdata             <= (others => '0');
        rdata             <= (others => '0');
        mem_addr          <= (others => '0');
        mem_din           <= (others => '0');
        tx_payload_len    <= 0;

        for i in 0 to C_MAX_PKT_BYTES-1 loop
          rx_buf(i)         <= (others => '0');
          tx_payload_buf(i) <= (others => '0');
        end loop;

      else

        -----------------------------------------------------------------------
        -- Capture popped FIFO data one cycle after asserting rxq_rd_en
        -----------------------------------------------------------------------
        if pop_pending = '1' then
          rx_byte     <= rxq_data;
          pop_pending <= '0';
        end if;

        case rx_state is

          ---------------------------------------------------------------------
          -- Wait for first byte of packet (LEN)
          ---------------------------------------------------------------------
          when RX_IDLE =>
            if rxq_empty = '0' then
              rx_state <= RX_LEN_POP;
            end if;

          ---------------------------------------------------------------------
          -- Pop LEN byte
          ---------------------------------------------------------------------
          when RX_LEN_POP =>
            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              rx_state    <= RX_LEN_CAP;
            end if;

          ---------------------------------------------------------------------
          -- Capture LEN and initialize receive bookkeeping
          ---------------------------------------------------------------------
          when RX_LEN_CAP =>
            if pop_pending = '0' then
              rx_expected_len  <= to_integer(unsigned(rx_byte));
              rx_index         <= 0;
              rx_checksum_calc <= (others => '0');
              rx_checksum_rx   <= (others => '0');

              if (to_integer(unsigned(rx_byte)) >= 2) and
                 (to_integer(unsigned(rx_byte)) <= C_MAX_PKT_BYTES) then
                rx_state <= RX_BYTE_POP;
              else
                rx_state <= RX_CLEAR;
              end if;
            end if;

          ---------------------------------------------------------------------
          -- Pop next packet byte
          ---------------------------------------------------------------------
          when RX_BYTE_POP =>
            if rx_index < rx_expected_len then
              if rxq_empty = '0' then
                rxq_rd_en   <= '1';
                pop_pending <= '1';
                rx_state    <= RX_BYTE_CAP;
              end if;
            else
              rx_state <= RX_CR_POP;
            end if;

          ---------------------------------------------------------------------
          -- Capture packet byte into buffer
          -- Incremental checksum:
          --   - bytes 0 to LEN-2 are payload bytes and are XORed into
          --     rx_checksum_calc as they arrive
          --   - byte LEN-1 is the received checksum byte and is stored in
          --     rx_checksum_rx
          ---------------------------------------------------------------------
          when RX_BYTE_CAP =>
            if pop_pending = '0' then
              if rx_byte = C_CR then
                rx_state <= RX_CLEAR;
              else
                rx_buf(rx_index) <= rx_byte;

                if rx_index < (rx_expected_len - 1) then
                  rx_checksum_calc <= rx_checksum_calc xor rx_byte;
                else
                  rx_checksum_rx <= rx_byte;
                end if;

                if (rx_index + 1) < rx_expected_len then
                  rx_index <= rx_index + 1;
                  rx_state <= RX_BYTE_POP;
                else
                  rx_index <= rx_index + 1;
                  rx_state <= RX_CR_POP;
                end if;
              end if;
            end if;

          ---------------------------------------------------------------------
          -- After exactly LEN bytes, next byte must be CR
          ---------------------------------------------------------------------
          when RX_CR_POP =>
            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              rx_state    <= RX_CR_CAP;
            end if;

          ---------------------------------------------------------------------
          -- Validate CR terminator
          ---------------------------------------------------------------------
          when RX_CR_CAP =>
            if pop_pending = '0' then
              if rx_byte = C_CR then
                rx_state <= RX_VALIDATE;
              else
                rx_state <= RX_CLEAR;
              end if;
            end if;

          ---------------------------------------------------------------------
          -- Validate checksum
          ---------------------------------------------------------------------
          when RX_VALIDATE =>
            if rx_checksum_rx = rx_checksum_calc then
              rx_state <= RX_DECODE;
            else
              rx_state <= RX_CLEAR;
            end if;

          ---------------------------------------------------------------------
          -- Decode validated packet
          ---------------------------------------------------------------------
          when RX_DECODE =>

            cmd    <= rx_buf(0);
            addr16 <= rx_buf(1) & rx_buf(2);

            addr_word <= rx_buf(1)(1 downto 0) & rx_buf(2);
            mem_addr  <= rx_buf(1)(1 downto 0) & rx_buf(2);

            if (rx_buf(0) = C_CMD_R) and (rx_expected_len = C_LEN_READ_REQ) then
              rx_state <= RX_READ_REQ;

            elsif (rx_buf(0) = C_CMD_W) and (rx_expected_len = C_LEN_WRITE_REQ) then
              wdata    <= rx_buf(3) & rx_buf(4);
              rx_state <= RX_WRITE_SETUP;

            else
              rx_state <= RX_CLEAR;
            end if;

          ---------------------------------------------------------------------
          -- Validated read request: assert memory enable
          ---------------------------------------------------------------------
          when RX_READ_REQ =>
            mem_addr <= addr_word;
            mem_en   <= '1';
            mem_we   <= '0';
            rx_state <= RX_READ_WAIT;

          ---------------------------------------------------------------------
          -- Read data appears after one cycle
          ---------------------------------------------------------------------
          when RX_READ_WAIT =>
            rdata    <= mem_dout;
            rx_state <= RX_PREP_READ_RSP;

          ---------------------------------------------------------------------
          -- Write setup: present address and data one full cycle before commit
          ---------------------------------------------------------------------
          when RX_WRITE_SETUP =>
            mem_addr <= addr_word;
            mem_din  <= wdata;
            rx_state <= RX_WRITE_COMMIT;

          ---------------------------------------------------------------------
          -- Write commit: pulse write enable after address/data are stable
          ---------------------------------------------------------------------
          when RX_WRITE_COMMIT =>
            mem_addr <= addr_word;
            mem_din  <= wdata;
            mem_en   <= '1';
            mem_we   <= '1';
            rx_state <= RX_PREP_ACK_RSP;

          ---------------------------------------------------------------------
          -- Prepare read response payload = DATA_H DATA_L
          ---------------------------------------------------------------------
          when RX_PREP_READ_RSP =>
            if tx_busy = '0' then
              tx_payload_buf(0) <= rdata(15 downto 8);
              tx_payload_buf(1) <= rdata(7 downto 0);
              tx_payload_len    <= 2;
              tx_start          <= '1';
              rx_state          <= RX_CLEAR;
            end if;

          ---------------------------------------------------------------------
          -- Prepare ACK response payload = ACK
          ---------------------------------------------------------------------
          when RX_PREP_ACK_RSP =>
            if tx_busy = '0' then
              tx_payload_buf(0) <= C_ACK;
              tx_payload_len    <= 1;
              tx_start          <= '1';
              rx_state          <= RX_CLEAR;
            end if;

          ---------------------------------------------------------------------
          -- Clear working registers and return idle
          ---------------------------------------------------------------------
          when RX_CLEAR =>

            rx_expected_len  <= 0;
            rx_index         <= 0;
            rx_checksum_rx   <= (others => '0');
            rx_checksum_calc <= (others => '0');
            cmd              <= (others => '0');
            addr16           <= (others => '0');
            addr_word        <= (others => '0');
            wdata            <= (others => '0');
            pop_pending      <= '0';
            rx_byte          <= (others => '0');

            for i in 0 to C_MAX_PKT_BYTES-1 loop
              rx_buf(i) <= (others => '0');
            end loop;

            rx_state <= RX_IDLE;

          when others =>
            rx_state <= RX_IDLE;

        end case;
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- TX PACKET FSM
  -----------------------------------------------------------------------------
  p_tx_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then

      -------------------------------------------------------------------------
      -- Default single-cycle strobe
      -------------------------------------------------------------------------
      txq_wr_en <= '0';

      if reset_i = '1' then
        tx_state          <= TX_IDLE;
        tx_busy           <= '0';
        tx_payload_len_loc <= 0;
        tx_frame_len      <= 0;
        tx_build_index    <= 0;
        tx_send_index     <= 0;
        tx_checksum_run   <= (others => '0');
        txq_data_in       <= (others => '0');

        for i in 0 to C_MAX_PKT_BYTES-1 loop
          tx_payload_local(i) <= (others => '0');
        end loop;

        for i in 0 to C_MAX_FRAME_BYTES-1 loop
          tx_frame_buf(i) <= (others => '0');
        end loop;

      else
        case tx_state is

          ---------------------------------------------------------------------
          -- Wait for tx_start pulse from RX FSM
          ---------------------------------------------------------------------
          when TX_IDLE =>
            tx_busy        <= '0';
            tx_build_index <= 0;
            tx_send_index  <= 0;

            if tx_start = '1' then
              tx_busy  <= '1';
              tx_state <= TX_LATCH;
            end if;

          ---------------------------------------------------------------------
          -- Latch payload request from RX side
          ---------------------------------------------------------------------
          when TX_LATCH =>
            tx_busy            <= '1';
            tx_payload_len_loc <= tx_payload_len;
            tx_checksum_run    <= (others => '0');
            tx_build_index     <= 0;

            -- Frame format:
            --   [0] = LEN = payload_len + 1 (checksum byte included)
            tx_frame_buf(0) <= std_logic_vector(to_unsigned(tx_payload_len + 1, 8));

            for i in 0 to C_MAX_PKT_BYTES-1 loop
              tx_payload_local(i) <= tx_payload_buf(i);
            end loop;

            tx_state <= TX_BUILD;

          ---------------------------------------------------------------------
          -- Build full outgoing frame incrementally:
          --   LEN | PAYLOAD... | CHKSUM | CR
          ---------------------------------------------------------------------
          when TX_BUILD =>
            tx_busy <= '1';

            if tx_build_index < tx_payload_len_loc then
              tx_frame_buf(tx_build_index + 1) <= tx_payload_local(tx_build_index);
              tx_checksum_run <= tx_checksum_run xor tx_payload_local(tx_build_index);
              tx_build_index  <= tx_build_index + 1;
            else
              tx_state <= TX_FINALIZE;
            end if;

          ---------------------------------------------------------------------
          -- Append checksum and CR after payload build is complete
          ---------------------------------------------------------------------
          when TX_FINALIZE =>
            tx_busy <= '1';

            tx_frame_buf(tx_payload_len_loc + 1) <= tx_checksum_run;
            tx_frame_buf(tx_payload_len_loc + 2) <= C_CR;

            -- Total transmitted bytes = LEN byte + payload bytes + checksum + CR
            tx_frame_len   <= tx_payload_len_loc + 3;
            tx_send_index  <= 0;
            tx_state       <= TX_SEND;

          ---------------------------------------------------------------------
          -- Send completed frame buffer byte-by-byte
          ---------------------------------------------------------------------
          when TX_SEND =>
            tx_busy <= '1';

            if tx_send_index < tx_frame_len then
              if txq_full = '0' then
                txq_data_in <= tx_frame_buf(tx_send_index);
                txq_wr_en   <= '1';
                tx_send_index <= tx_send_index + 1;
              end if;
            else
              tx_state <= TX_IDLE;
            end if;

          when others =>
            tx_state <= TX_IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture;