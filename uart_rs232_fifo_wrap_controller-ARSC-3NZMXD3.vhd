-- ============================================================================
-- File: uart_rs232_fifo_wrap_controller.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 16 March 2026
-- Last Updated: 16 March 2026
--
-- Description:
--   UART protocol controller wrapper built around uart_rs232_fifo_wrap.
--
--   This module is responsible for:
--     1) UART RX/TX buffering through uart_rs232_fifo_wrap
--     2) Packet parsing and framing validation
--     3) Packet checksum validation
--     4) Command execution sequencing
--     5) Formatted TX packet generation
--
--   This module no longer accesses SPRAM directly.
--   Instead, it behaves as a client of the dedicated Bus_Master SPRAM wrapper.
--
--   Bus interface:
--     - bus_req  : request access / transaction
--     - bus_ack  : transaction complete / data valid
--     - bus_we   : 0=read, 1=write
--     - bus_addr : SPRAM word address
--     - bus_wdata: data for writes
--     - bus_rdata: returned data for reads
--
-- Packet format:
--
--   Incoming:
--     LEN | PAYLOAD... | CHKSUM | CR
--
--   Read request payload:
--     CMD | ADDR_H | ADDR_L | COUNT
--     COUNT = number of sequential 16-bit SPRAM words to read
--     Example:
--       05 52 00 04 02 54 0D
--       LEN=05, CMD='R', ADDR=0x0004, COUNT=2, CHKSUM=54, CR=0D
--
--   Write request payload:
--     CMD | ADDR_H | ADDR_L | DATA0_H | DATA0_L | DATA1_H | DATA1_L | ...
--     Number of sequential writes is inferred from packet length
--     Example:
--       08 57 00 04 12 34 AB CD 1B 0D
--       LEN=08, CMD='W', ADDR=0x0004, two words: 0x1234, 0xABCD
--
--   Outgoing:
--     LEN | PAYLOAD... | CHKSUM | CR
--
--   Read response payload:
--     DATA0_H | DATA0_L | DATA1_H | DATA1_L | ...
--
--   Write response payload:
--     ACK
--
-- Notes:
--   - LEN counts PAYLOAD + CHKSUM
--   - CHKSUM is XOR over PAYLOAD bytes only
--   - CR is not included in LEN or CHKSUM
--   - bus_addr is 10 bits, so only ADDR_H(1 downto 0) is used
--   - C_MAX_PKT_BYTES holds our upper limit for packet length
--   - RX FIFO is show-ahead, so bytes must be captured before pop
--   - CR is only checked at the expected final terminator position
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_rs232_fifo_wrap_controller is
  generic (
    CLK_FREQ_HZ : integer := 24930000;
    BAUD        : integer := 9600;
    DATA_BITS   : integer := 8;
    STOP_BITS   : integer := 1;
    PARITY      : integer := 0
  );
  port (
    clk_sys : in  std_logic;
    reset_i : in  std_logic;

    -- Serial pins
    RX : in  std_logic;
    TX : out std_logic;

    -- Client bus to Bus_Master
    bus_req   : out std_logic;
    bus_ack   : in  std_logic;
    bus_we    : out std_logic;
    bus_addr  : out std_logic_vector(9 downto 0);
    bus_wdata : out std_logic_vector(15 downto 0);
    bus_rdata : in  std_logic_vector(15 downto 0);

    -- Debug LEDs
    LEDn : out std_logic_vector(7 downto 0)
  );
end entity;

architecture behavioral of uart_rs232_fifo_wrap_controller is

  ------------------------------------------------------------------------------
  -- Protocol constants
  ------------------------------------------------------------------------------
  constant C_CMD_W : std_logic_vector(7 downto 0) := x"57"; -- 'W'
  constant C_CMD_R : std_logic_vector(7 downto 0) := x"52"; -- 'R'
  constant C_ACK   : std_logic_vector(7 downto 0) := x"06";
  constant C_CR    : std_logic_vector(7 downto 0) := x"0D";

  constant C_MAX_PKT_BYTES    : integer := 256;
  constant C_MAX_WRITE_WORDS  : integer := (C_MAX_PKT_BYTES - 4) / 2;
  constant C_MAX_READ_WORDS   : integer := C_MAX_PKT_BYTES / 2;
  constant C_MAX_BURST_WORDS  : integer := C_MAX_WRITE_WORDS;

  -- Read request is: CMD | ADDR_H | ADDR_L | COUNT  => payload=4, LEN=5
  constant C_LEN_READ_REQ : integer := 5;

  ------------------------------------------------------------------------------
  -- Byte array type
  ------------------------------------------------------------------------------
  type t_byte_array is array (0 to C_MAX_PKT_BYTES-1) of std_logic_vector(7 downto 0);

  ------------------------------------------------------------------------------
  -- UART FIFO wrapper signals
  ------------------------------------------------------------------------------
  signal rxq_data   : std_logic_vector(7 downto 0);
  signal rxq_empty  : std_logic;
  signal rxq_full   : std_logic;
  signal rxq_rd_en  : std_logic;

  signal txq_data_in : std_logic_vector(7 downto 0);
  signal txq_wr_en   : std_logic;
  signal txq_empty   : std_logic;
  signal txq_full    : std_logic;

  ------------------------------------------------------------------------------
  -- RX FSM
  ------------------------------------------------------------------------------
  type t_rx_state is (
    RX_IDLE,
    RX_LEN_POP,
    RX_BYTE_POP,
    RX_CR_POP,
    RX_VALIDATE,
    RX_QUEUE_REQ,
    RX_CLEAR
  );

  ------------------------------------------------------------------------------
  -- EXEC FSM
  ------------------------------------------------------------------------------
  type t_ex_state is (
    EX_IDLE,
    EX_READ_REQ,
    EX_WRITE_REQ,
    EX_NEXT,
    EX_PREP_READ_RSP,
    EX_START_READ_RSP,
    EX_PREP_ACK_RSP,
    EX_START_ACK_RSP
  );

  ------------------------------------------------------------------------------
  -- TX FSM
  ------------------------------------------------------------------------------
  type t_tx_state is (
    TX_IDLE,
    TX_PREP,
    TX_SEND_LEN,
    TX_SEND_PAYLOAD,
    TX_SEND_CHKSUM,
    TX_SEND_CR
  );

  ------------------------------------------------------------------------------
  -- State registers
  ------------------------------------------------------------------------------
  signal rx_state : t_rx_state := RX_IDLE;
  signal ex_state : t_ex_state := EX_IDLE;
  signal tx_state : t_tx_state := TX_IDLE;

  ------------------------------------------------------------------------------
  -- RX packet bookkeeping
  ------------------------------------------------------------------------------
  signal rx_byte           : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_buf            : t_byte_array;
  signal rx_expected_len   : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal rx_index          : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal rx_checksum_rx    : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_checksum_calc  : std_logic_vector(7 downto 0) := (others => '0');

  ------------------------------------------------------------------------------
  -- Queued request interface
  ------------------------------------------------------------------------------
  signal req_valid       : std_logic := '0';
  signal req_take        : std_logic := '0';
  signal req_cmd         : std_logic_vector(7 downto 0) := (others => '0');
  signal req_count       : integer range 0 to C_MAX_BURST_WORDS := 0;
  signal req_addr        : std_logic_vector(9 downto 0) := (others => '0');
  signal req_payload_len : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal req_buf         : t_byte_array;

  ------------------------------------------------------------------------------
  -- Active execution registers
  ------------------------------------------------------------------------------
  signal ex_cmd        : std_logic_vector(7 downto 0) := (others => '0');
  signal ex_count      : integer range 0 to C_MAX_BURST_WORDS := 0;
  signal ex_base_addr  : std_logic_vector(9 downto 0) := (others => '0');
  signal ex_index      : integer range 0 to C_MAX_BURST_WORDS := 0;
  signal ex_buf        : t_byte_array;

  ------------------------------------------------------------------------------
  -- TX payload interface
  ------------------------------------------------------------------------------
  signal tx_payload_buf : t_byte_array;
  signal tx_payload_len : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal tx_checksum    : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_len_byte    : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_index       : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal tx_start       : std_logic := '0';
  signal tx_busy        : std_logic := '0';

  ------------------------------------------------------------------------------
  -- Debug LEDs
  ------------------------------------------------------------------------------
  signal led_buff : std_logic_vector(7 downto 0) := (others => '0');

begin

  LEDn <= led_buff;

  ------------------------------------------------------------------------------
  -- UART wrapper instantiation
  ------------------------------------------------------------------------------
  u_uart_wrap : entity work.uart_rs232_fifo_wrap
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
      TX => TX,

      rxq_data  => rxq_data,
      rxq_empty => rxq_empty,
      rxq_full  => rxq_full,
      rxq_rd_en => rxq_rd_en,

      txq_data_in => txq_data_in,
      txq_wr_en   => txq_wr_en,
      txq_empty   => txq_empty,
      txq_full    => txq_full
    );

  ------------------------------------------------------------------------------
  -- LED decode
  ------------------------------------------------------------------------------
  p_led_decode : process(rx_state, ex_state, reset_i)
  begin
    if reset_i = '1' then
      led_buff <= (others => '0');
    else
      case rx_state is
        when RX_IDLE      => led_buff <= "10000000";
        when RX_LEN_POP   => led_buff <= "01000000";
        when RX_BYTE_POP  => led_buff <= "00010000";
        when RX_CR_POP    => led_buff <= "00000100";
        when RX_VALIDATE  => led_buff <= "00000001";
        when RX_QUEUE_REQ => led_buff <= "00000011";
        when RX_CLEAR     => led_buff <= "11111111";
        when others =>
          case ex_state is
            when EX_IDLE           => led_buff <= "10011001";
            when EX_READ_REQ       => led_buff <= "00111100";
            when EX_WRITE_REQ      => led_buff <= "00001111";
            when EX_NEXT           => led_buff <= "10100101";
            when EX_PREP_READ_RSP  => led_buff <= "01111110";
            when EX_START_READ_RSP => led_buff <= "01101110";
            when EX_PREP_ACK_RSP   => led_buff <= "01010101";
            when EX_START_ACK_RSP  => led_buff <= "01000101";
            when others            => led_buff <= (others => '0');
          end case;
      end case;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- RX FSM
  -- NOTE: FIFO is show-ahead. Capture rxq_data before asserting pop.
  -- NOTE: Do not test for CR inside the payload stream. CR is only checked
  --       after exactly LEN bytes have been received.
  ------------------------------------------------------------------------------
  p_rx_fsm : process(clk_sys)
    variable v_payload_len : integer;
    variable v_count       : integer;
    variable v_ok          : boolean;
  begin
    if rising_edge(clk_sys) then

      rxq_rd_en <= '0';

      if reset_i = '1' then
        rx_state          <= RX_IDLE;
        rx_byte           <= (others => '0');
        rx_expected_len   <= 0;
        rx_index          <= 0;
        rx_checksum_rx    <= (others => '0');
        rx_checksum_calc  <= (others => '0');

        req_valid         <= '0';
        req_cmd           <= (others => '0');
        req_count         <= 0;
        req_addr          <= (others => '0');
        req_payload_len   <= 0;

        for i in 0 to C_MAX_PKT_BYTES-1 loop
          rx_buf(i)  <= (others => '0');
          req_buf(i) <= (others => '0');
        end loop;

      else

        if req_take = '1' then
          req_valid <= '0';
        end if;

        case rx_state is

          when RX_IDLE =>
            if rxq_empty = '0' then
              rx_state <= RX_LEN_POP;
            end if;

          when RX_LEN_POP =>
            if rxq_empty = '0' then
              rx_byte          <= rxq_data;
              rxq_rd_en        <= '1';
              rx_expected_len  <= to_integer(unsigned(rxq_data));
              rx_index         <= 0;
              rx_checksum_calc <= (others => '0');
              rx_checksum_rx   <= (others => '0');

              if (to_integer(unsigned(rxq_data)) >= 2) and
                 (to_integer(unsigned(rxq_data)) <= C_MAX_PKT_BYTES) then
                rx_state <= RX_BYTE_POP;
              else
                rx_state <= RX_CLEAR;
              end if;
            end if;

          when RX_BYTE_POP =>
            if rx_index < rx_expected_len then
              if rxq_empty = '0' then
                rx_byte   <= rxq_data;
                rxq_rd_en <= '1';

                rx_buf(rx_index) <= rxq_data;

                if rx_index < (rx_expected_len - 1) then
                  rx_checksum_calc <= rx_checksum_calc xor rxq_data;
                else
                  rx_checksum_rx <= rxq_data;
                end if;

                if (rx_index + 1) < rx_expected_len then
                  rx_index <= rx_index + 1;
                  rx_state <= RX_BYTE_POP;
                else
                  rx_index <= rx_index + 1;
                  rx_state <= RX_CR_POP;
                end if;
              end if;
            else
              rx_state <= RX_CR_POP;
            end if;

          when RX_CR_POP =>
            if rxq_empty = '0' then
              rx_byte   <= rxq_data;
              rxq_rd_en <= '1';

              if rxq_data = C_CR then
                rx_state <= RX_VALIDATE;
              else
                rx_state <= RX_CLEAR;
              end if;
            end if;

          when RX_VALIDATE =>
            if rx_checksum_rx = rx_checksum_calc then
              rx_state <= RX_QUEUE_REQ;
            else
              rx_state <= RX_CLEAR;
            end if;

          when RX_QUEUE_REQ =>
            if req_valid = '0' then

              -- rx_expected_len = payload bytes + checksum byte
              v_payload_len := rx_expected_len - 1;
              v_count       := 0;
              v_ok          := false;

              if rx_buf(0) = C_CMD_R then
                -- Read payload: CMD | ADDR_H | ADDR_L | COUNT
                if rx_expected_len = C_LEN_READ_REQ then
                  v_count := to_integer(unsigned(rx_buf(3)));
                  if (v_count >= 1) and
                     (v_count <= C_MAX_BURST_WORDS) and
                     ((2 * v_count) <= C_MAX_PKT_BYTES) then
                    v_ok := true;
                  end if;
                end if;

              elsif rx_buf(0) = C_CMD_W then
                -- Write payload: CMD | ADDR_H | ADDR_L | DATA_H | DATA_L | ...
                if v_payload_len >= 5 then
                  if ((v_payload_len - 3) mod 2) = 0 then
                    v_count := (v_payload_len - 3) / 2;
                    if (v_count >= 1) and
                       (v_count <= C_MAX_BURST_WORDS) then
                      v_ok := true;
                    end if;
                  end if;
                end if;
              end if;

              if v_ok then
                req_cmd         <= rx_buf(0);
                req_count       <= v_count;
                req_addr        <= rx_buf(1)(1 downto 0) & rx_buf(2);
                req_payload_len <= v_payload_len;
                req_valid       <= '1';

                for i in 0 to C_MAX_PKT_BYTES-1 loop
                  req_buf(i) <= rx_buf(i);
                end loop;

                rx_state <= RX_CLEAR;
              else
                rx_state <= RX_CLEAR;
              end if;
            end if;

          when RX_CLEAR =>
            rx_expected_len  <= 0;
            rx_index         <= 0;
            rx_checksum_rx   <= (others => '0');
            rx_checksum_calc <= (others => '0');
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

  ------------------------------------------------------------------------------
  -- EXEC FSM
  ------------------------------------------------------------------------------
  p_exec_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then

      bus_req   <= '0';
      bus_we    <= '0';
      bus_addr  <= (others => '0');
      bus_wdata <= (others => '0');

      tx_start <= '0';
      req_take <= '0';

      if reset_i = '1' then
        ex_state       <= EX_IDLE;
        ex_cmd         <= (others => '0');
        ex_count       <= 0;
        ex_base_addr   <= (others => '0');
        ex_index       <= 0;
        tx_payload_len <= 0;

        for i in 0 to C_MAX_PKT_BYTES-1 loop
          ex_buf(i)         <= (others => '0');
          tx_payload_buf(i) <= (others => '0');
        end loop;

      else
        case ex_state is

          when EX_IDLE =>
            ex_index <= 0;

            if (req_valid = '1') and (tx_busy = '0') then
              ex_cmd       <= req_cmd;
              ex_count     <= req_count;
              ex_base_addr <= req_addr;
              ex_index     <= 0;

              for i in 0 to C_MAX_PKT_BYTES-1 loop
                ex_buf(i) <= req_buf(i);
              end loop;

              req_take <= '1';

              if req_cmd = C_CMD_R then
                ex_state <= EX_READ_REQ;
              else
                ex_state <= EX_WRITE_REQ;
              end if;
            end if;

          when EX_READ_REQ =>
            bus_req  <= '1';
            bus_we   <= '0';
            bus_addr <= std_logic_vector(unsigned(ex_base_addr) + to_unsigned(ex_index, 10));

            if bus_ack = '1' then
              tx_payload_buf(2*ex_index)     <= bus_rdata(15 downto 8);
              tx_payload_buf(2*ex_index + 1) <= bus_rdata(7 downto 0);
              ex_state                       <= EX_NEXT;
            else
              ex_state <= EX_READ_REQ;
            end if;

          when EX_WRITE_REQ =>
            bus_req   <= '1';
            bus_we    <= '1';
            bus_addr  <= std_logic_vector(unsigned(ex_base_addr) + to_unsigned(ex_index, 10));
            bus_wdata <= ex_buf(3 + 2*ex_index) & ex_buf(4 + 2*ex_index);

            if bus_ack = '1' then
              ex_state <= EX_NEXT;
            else
              ex_state <= EX_WRITE_REQ;
            end if;

          when EX_NEXT =>
            if (ex_index + 1) < ex_count then
              ex_index <= ex_index + 1;

              if ex_cmd = C_CMD_R then
                ex_state <= EX_READ_REQ;
              else
                ex_state <= EX_WRITE_REQ;
              end if;
            else
              if ex_cmd = C_CMD_R then
                ex_state <= EX_PREP_READ_RSP;
              else
                ex_state <= EX_PREP_ACK_RSP;
              end if;
            end if;

          when EX_PREP_READ_RSP =>
            if tx_busy = '0' then
              tx_payload_len <= 2 * ex_count;
              ex_state       <= EX_START_READ_RSP;
            end if;

          when EX_START_READ_RSP =>
            if tx_busy = '0' then
              tx_start <= '1';
              ex_state <= EX_IDLE;
            end if;

          when EX_PREP_ACK_RSP =>
            if tx_busy = '0' then
              tx_payload_buf(0) <= C_ACK;
              tx_payload_len    <= 1;
              ex_state          <= EX_START_ACK_RSP;
            end if;

          when EX_START_ACK_RSP =>
            if tx_busy = '0' then
              tx_start <= '1';
              ex_state <= EX_IDLE;
            end if;

          when others =>
            ex_state <= EX_IDLE;
        end case;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- TX FSM
  ------------------------------------------------------------------------------
  p_tx_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then

      txq_wr_en <= '0';

      if reset_i = '1' then
        tx_state    <= TX_IDLE;
        tx_busy     <= '0';
        tx_index    <= 0;
        tx_checksum <= (others => '0');
        tx_len_byte <= (others => '0');
        txq_data_in <= (others => '0');

      else
        case tx_state is

          when TX_IDLE =>
            tx_busy  <= '0';
            tx_index <= 0;

            if tx_start = '1' then
              tx_busy  <= '1';
              tx_state <= TX_PREP;
            end if;

          when TX_PREP =>
            tx_busy     <= '1';
            tx_len_byte <= std_logic_vector(to_unsigned(tx_payload_len + 1, 8));
            tx_checksum <= (others => '0');
            tx_index    <= 0;
            tx_state    <= TX_SEND_LEN;

          when TX_SEND_LEN =>
            tx_busy <= '1';
            if txq_full = '0' then
              txq_data_in <= tx_len_byte;
              txq_wr_en   <= '1';
              tx_index    <= 0;
              tx_state    <= TX_SEND_PAYLOAD;
            end if;

          when TX_SEND_PAYLOAD =>
            tx_busy <= '1';

            if tx_index < tx_payload_len then
              if txq_full = '0' then
                txq_data_in <= tx_payload_buf(tx_index);
                txq_wr_en   <= '1';
                tx_checksum <= tx_checksum xor tx_payload_buf(tx_index);
                tx_index    <= tx_index + 1;
              end if;
            else
              tx_state <= TX_SEND_CHKSUM;
            end if;

          when TX_SEND_CHKSUM =>
            tx_busy <= '1';
            if txq_full = '0' then
              txq_data_in <= tx_checksum;
              txq_wr_en   <= '1';
              tx_state    <= TX_SEND_CR;
            end if;

          when TX_SEND_CR =>
            tx_busy <= '1';
            if txq_full = '0' then
              txq_data_in <= C_CR;
              txq_wr_en   <= '1';
              tx_state    <= TX_IDLE;
            end if;

          when others =>
            tx_state <= TX_IDLE;
        end case;
      end if;
    end if;
  end process;

end architecture;