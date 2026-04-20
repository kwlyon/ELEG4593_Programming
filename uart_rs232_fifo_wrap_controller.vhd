-- ============================================================================
-- File: uart_rs232_fifo_wrap_controller.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 16 March 2026
-- Last Updated: 14 April 2026
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
--     SOF | LEN | PAYLOAD... | CHKSUM
--
--   Read request payload:
--     OP_ID | COUNT | ADDR_H | ADDR_L
--     Example: 7E 04 0F 10 01 00 DF
--
--   Write request payload:
--     OP_ID | COUNT | ADDR_H | ADDR_L | DATA0_H | DATA0_L | ...
--     Example: 7E 12 0A 07 01 00 00 01 BF FF 60 00 20 00 40 00 80 00 FF FF F0
--
--   Outgoing:
--     SOF | LEN | PAYLOAD... | CHKSUM
--
--   Read response payload:
--     0A | COUNT | ADDR_H | ADDR_L | DATA0_H | DATA0_L | ...
--
--   Write response payload:
--     none
--
--   Error response payload:
--     00
--
-- Notes:
--   - SOF is 0x7E
--   - LEN counts OP_ID through the end of DATA
--   - CHKSUM is 0xFF - sum(PAYLOAD bytes)
--   - SOF and CHKSUM are not included in LEN
--
-- Revision History:
--
--   2026-03-16
--     - Initial version.
--     - Separated UART packet handling from Bus_Master.
--     - Added RX packet FSM, execution FSM, and TX packet FSM.
--     - Implemented packet parsing, checksum validation, request queueing,
--       execution sequencing, and formatted reply generation.
--
--   2026-03-18
--     - Refactored the Bus_Master client interface to use structured request
--       and response records from bus_pkg.vhd.
--     - Replaced flat bus_req / bus_ack / bus_we / bus_addr / bus_wdata /
--       bus_rdata ports with bus_req : t_bus_req and bus_rsp : t_bus_rsp.
--     - No intended functional change to UART packet handling or execution
--       state-machine behavior.
--
--   2026-03-20
--     - Added 0x7E start-of-frame delimiter support to both RX and TX packet
--       formats.
--     - Updated the RX FSM to require and validate SOF before LEN.
--     - Updated the TX FSM to prepend SOF to all transmitted packets.
--     - Added response packet support for read transactions.
--
--   2026-04-05
--     - Removed legacy debug LED output port and all associated internal LED
--       decode logic.
--     - No intended functional change to UART packet handling, bus access, or
--       TX/RX behavior.
--
--   2026-04-07
--     - Fixed RX parser so 0x0D is no longer treated as an illegal value
--       during payload/checksum reception.
--
--   2026-04-13
--     - Updated framing to:
--           7E | LEN | OP_ID | COUNT | ADDR_H | ADDR_L | DATA... | CHKSUM
--     - LEN now excludes checksum and counts OP_ID through DATA only.
--     - Read OP_ID is 0x0F and write/read-response OP_ID is 0x0A.
--     - Checksum is now computed as 0xFF - sum(payload bytes).
--     - Write requests no longer transmit an acknowledge packet.
--
--   2026-04-14
--     - Added framed UART error response packet support (7E 01 00 FF) for
--       packet checksum failure and invalid request conditions.
--     - Implemented single-owner handshake for error response pending flag:
--       RX FSM asserts/clears the pending flag, EXEC FSM only requests clear.
--     - This prevents LabVIEW from hanging on rejected packets while also
--       avoiding multi-driver control logic.
--
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.bus_pkg.all;

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
    bus_req : out t_bus_req;
    bus_rsp : in  t_bus_rsp
  );
end entity;

architecture behavioral of uart_rs232_fifo_wrap_controller is

  ------------------------------------------------------------------------------
  -- Protocol constants
  ------------------------------------------------------------------------------
  constant C_SOF     : std_logic_vector(7 downto 0) := x"7E";
  constant C_CMD_W   : std_logic_vector(7 downto 0) := x"0A";
  constant C_CMD_R   : std_logic_vector(7 downto 0) := x"0F";
  constant C_ERR_PAY : std_logic_vector(7 downto 0) := x"00";

  constant C_MAX_PKT_BYTES   : integer := 64;
  constant C_MAX_BURST_WORDS : integer := (C_MAX_PKT_BYTES - 5) / 2;

  constant C_LEN_READ_REQ : integer := 4;

  function f_checksum_next(
    sum_in  : std_logic_vector(7 downto 0);
    byte_in : std_logic_vector(7 downto 0)
  ) return std_logic_vector is
  begin
    return std_logic_vector(unsigned(sum_in) + unsigned(byte_in));
  end function;

  function f_checksum_final(
    sum_in : std_logic_vector(7 downto 0)
  ) return std_logic_vector is
  begin
    return std_logic_vector(to_unsigned(255, 8) - unsigned(sum_in));
  end function;

  ------------------------------------------------------------------------------
  -- Byte array type
  ------------------------------------------------------------------------------
  type t_byte_array is array (0 to C_MAX_PKT_BYTES-1) of std_logic_vector(7 downto 0);

  ------------------------------------------------------------------------------
  -- UART FIFO wrapper signals
  ------------------------------------------------------------------------------
  signal rxq_data    : std_logic_vector(7 downto 0);
  signal rxq_empty   : std_logic;
  signal rxq_full    : std_logic;
  signal rxq_rd_en   : std_logic;

  signal txq_data_in : std_logic_vector(7 downto 0);
  signal txq_wr_en   : std_logic;
  signal txq_empty   : std_logic;
  signal txq_full    : std_logic;

  ------------------------------------------------------------------------------
  -- RX FSM
  ------------------------------------------------------------------------------
  type t_rx_state is (
    RX_IDLE,
    RX_SOF_POP,
    RX_SOF_CAP,
    RX_LEN_POP,
    RX_LEN_CAP,
    RX_BYTE_POP,
    RX_BYTE_CAP,
    RX_CHKSUM_POP,
    RX_CHKSUM_CAP,
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
    EX_READ_WAIT,
    EX_WRITE_REQ,
    EX_WRITE_WAIT,
    EX_NEXT,
    EX_PREP_READ_RSP,
    EX_START_READ_RSP,
    EX_PREP_ERR_RSP,
    EX_START_ERR_RSP
  );

  ------------------------------------------------------------------------------
  -- TX FSM
  ------------------------------------------------------------------------------
  type t_tx_state is (
    TX_IDLE,
    TX_PREP,
    TX_SEND_SOF,
    TX_SEND_LEN,
    TX_SEND_PAYLOAD,
    TX_SEND_CHKSUM
  );

  ------------------------------------------------------------------------------
  -- State registers
  ------------------------------------------------------------------------------
  signal rx_state : t_rx_state := RX_IDLE;
  signal ex_state : t_ex_state := EX_IDLE;
  signal tx_state : t_tx_state := TX_IDLE;

  ------------------------------------------------------------------------------
  -- Shared RX pop/capture mechanism
  ------------------------------------------------------------------------------
  signal pop_pending : std_logic := '0';
  signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');

  ------------------------------------------------------------------------------
  -- RX packet bookkeeping
  ------------------------------------------------------------------------------
  signal rx_buf           : t_byte_array;
  signal rx_expected_len  : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal rx_index         : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal rx_checksum_rx   : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_checksum_calc : std_logic_vector(7 downto 0) := (others => '0');

  ------------------------------------------------------------------------------
  -- Queued request interface
  ------------------------------------------------------------------------------
  signal req_valid       : std_logic := '0';
  signal req_take        : std_logic := '0';
  signal req_cmd         : std_logic_vector(7 downto 0) := (others => '0');
  signal req_count       : integer range 0 to C_MAX_BURST_WORDS := 0;
  signal req_addr        : std_logic_vector(15 downto 0) := (others => '0');
  signal req_payload_len : integer range 0 to C_MAX_PKT_BYTES := 0;
  signal req_buf         : t_byte_array;

  ------------------------------------------------------------------------------
  -- Error response handshake
  ------------------------------------------------------------------------------
  signal err_rsp_pending : std_logic := '0';
  signal err_rsp_take    : std_logic := '0';

  ------------------------------------------------------------------------------
  -- Active execution registers
  ------------------------------------------------------------------------------
  signal ex_cmd        : std_logic_vector(7 downto 0) := (others => '0');
  signal ex_count      : integer range 0 to C_MAX_BURST_WORDS := 0;
  signal ex_base_addr  : std_logic_vector(15 downto 0) := (others => '0');
  signal ex_index      : integer range 0 to C_MAX_BURST_WORDS := 0;
  signal ex_curr_addr  : std_logic_vector(9 downto 0) := (others => '0');
  signal ex_curr_wdata : std_logic_vector(15 downto 0) := (others => '0');
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

begin

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
  -- RX FSM
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
        pop_pending       <= '0';
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

        err_rsp_pending   <= '0';

        for i in 0 to C_MAX_PKT_BYTES-1 loop
          rx_buf(i)  <= (others => '0');
          req_buf(i) <= (others => '0');
        end loop;

      else

        if pop_pending = '1' then
          rx_byte     <= rxq_data;
          pop_pending <= '0';
        end if;

        if req_take = '1' then
          req_valid <= '0';
        end if;

        if err_rsp_take = '1' then
          err_rsp_pending <= '0';
        end if;

        case rx_state is

          when RX_IDLE =>
            if rxq_empty = '0' then
              rx_state <= RX_SOF_POP;
            end if;

          when RX_SOF_POP =>
            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              rx_state    <= RX_SOF_CAP;
            end if;

          when RX_SOF_CAP =>
            if pop_pending = '0' then
              if rx_byte = C_SOF then
                rx_state <= RX_LEN_POP;
              else
                rx_state <= RX_IDLE;
              end if;
            end if;

          when RX_LEN_POP =>
            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              rx_state    <= RX_LEN_CAP;
            end if;

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
                err_rsp_pending <= '1';
                rx_state        <= RX_CLEAR;
              end if;
            end if;

          when RX_BYTE_POP =>
            if rx_index < rx_expected_len then
              if rxq_empty = '0' then
                rxq_rd_en   <= '1';
                pop_pending <= '1';
                rx_state    <= RX_BYTE_CAP;
              end if;
            else
              rx_state <= RX_CHKSUM_POP;
            end if;

          when RX_BYTE_CAP =>
            if pop_pending = '0' then
              rx_buf(rx_index) <= rx_byte;
              rx_checksum_calc <= f_checksum_next(rx_checksum_calc, rx_byte);

              if (rx_index + 1) < rx_expected_len then
                rx_index <= rx_index + 1;
                rx_state <= RX_BYTE_POP;
              else
                rx_index <= rx_index + 1;
                rx_state <= RX_CHKSUM_POP;
              end if;
            end if;

          when RX_CHKSUM_POP =>
            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              rx_state    <= RX_CHKSUM_CAP;
            end if;

          when RX_CHKSUM_CAP =>
            if pop_pending = '0' then
              rx_checksum_rx <= rx_byte;
              rx_state       <= RX_VALIDATE;
            end if;

          when RX_VALIDATE =>
            if rx_checksum_rx = f_checksum_final(rx_checksum_calc) then
              rx_state <= RX_QUEUE_REQ;
            else
              err_rsp_pending <= '1';
              rx_state        <= RX_CLEAR;
            end if;

          when RX_QUEUE_REQ =>
            if req_valid = '0' then

              v_payload_len := rx_expected_len;
              v_count       := 0;
              v_ok          := false;

              if rx_buf(0) = C_CMD_R then
                if rx_expected_len = C_LEN_READ_REQ then
                  v_count := to_integer(unsigned(rx_buf(1)));
                  if (v_count >= 1) and
                     (v_count <= C_MAX_BURST_WORDS) and
                     ((2 * v_count) <= C_MAX_PKT_BYTES) then
                    v_ok := true;
                  end if;
                end if;

              elsif rx_buf(0) = C_CMD_W then
                if v_payload_len >= 4 then
                  v_count := to_integer(unsigned(rx_buf(1)));
                  if (v_count >= 1) and
                     (v_count <= C_MAX_BURST_WORDS) and
                     (v_payload_len = (4 + 2*v_count)) then
                    v_ok := true;
                  end if;
                end if;
              end if;

              if v_ok then
                req_cmd         <= rx_buf(0);
                req_count       <= v_count;
                req_addr        <= rx_buf(2) & rx_buf(3);
                req_payload_len <= v_payload_len;
                req_valid       <= '1';

                for i in 0 to C_MAX_PKT_BYTES-1 loop
                  req_buf(i) <= rx_buf(i);
                end loop;

                rx_state <= RX_CLEAR;
              else
                err_rsp_pending <= '1';
                rx_state        <= RX_CLEAR;
              end if;
            end if;

          when RX_CLEAR =>
            rx_expected_len  <= 0;
            rx_index         <= 0;
            rx_checksum_rx   <= (others => '0');
            rx_checksum_calc <= (others => '0');
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

  ------------------------------------------------------------------------------
  -- EXEC FSM
  ------------------------------------------------------------------------------
  p_exec_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then

      bus_req.req   <= '0';
      bus_req.we    <= '0';
      bus_req.addr  <= (others => '0');
      bus_req.wdata <= (others => '0');

      tx_start    <= '0';
      req_take    <= '0';
      err_rsp_take <= '0';

      if reset_i = '1' then
        ex_state       <= EX_IDLE;
        ex_cmd         <= (others => '0');
        ex_count       <= 0;
        ex_base_addr   <= (others => '0');
        ex_index       <= 0;
        ex_curr_addr   <= (others => '0');
        ex_curr_wdata  <= (others => '0');
        tx_payload_len <= 0;

        for i in 0 to C_MAX_PKT_BYTES-1 loop
          ex_buf(i)         <= (others => '0');
          tx_payload_buf(i) <= (others => '0');
        end loop;

      else
        case ex_state is

          when EX_IDLE =>
            ex_index <= 0;

            if (err_rsp_pending = '1') and (tx_busy = '0') then
              ex_state <= EX_PREP_ERR_RSP;

            elsif (req_valid = '1') and (tx_busy = '0') then
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
            ex_curr_addr  <= std_logic_vector(unsigned(ex_base_addr(9 downto 0)) + to_unsigned(ex_index, 10));

            bus_req.req   <= '1';
            bus_req.we    <= '0';
            bus_req.addr  <= std_logic_vector(unsigned(ex_base_addr(9 downto 0)) + to_unsigned(ex_index, 10));

            if bus_rsp.ack = '1' then
              ex_state <= EX_READ_WAIT;
            else
              ex_state <= EX_READ_REQ;
            end if;

          when EX_READ_WAIT =>
            tx_payload_buf(4 + 2*ex_index) <= bus_rsp.rdata(15 downto 8);
            tx_payload_buf(5 + 2*ex_index) <= bus_rsp.rdata(7 downto 0);
            ex_state <= EX_NEXT;

          when EX_WRITE_REQ =>
            ex_curr_addr  <= std_logic_vector(unsigned(ex_base_addr(9 downto 0)) + to_unsigned(ex_index, 10));
            ex_curr_wdata <= ex_buf(4 + 2*ex_index) & ex_buf(5 + 2*ex_index);

            bus_req.req   <= '1';
            bus_req.we    <= '1';
            bus_req.addr  <= std_logic_vector(unsigned(ex_base_addr(9 downto 0)) + to_unsigned(ex_index, 10));
            bus_req.wdata <= ex_buf(4 + 2*ex_index) & ex_buf(5 + 2*ex_index);

            if bus_rsp.ack = '1' then
              ex_state <= EX_NEXT;
            else
              ex_state <= EX_WRITE_REQ;
            end if;

          when EX_WRITE_WAIT =>
            ex_state <= EX_NEXT;

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
                ex_state <= EX_IDLE;
              end if;
            end if;

          when EX_PREP_READ_RSP =>
            if tx_busy = '0' then
              tx_payload_buf(0) <= C_CMD_W;
              tx_payload_buf(1) <= std_logic_vector(to_unsigned(ex_count, 8));
              tx_payload_buf(2) <= ex_base_addr(15 downto 8);
              tx_payload_buf(3) <= ex_base_addr(7 downto 0);
              tx_payload_len    <= 4 + 2 * ex_count;
              ex_state          <= EX_START_READ_RSP;
            end if;

          when EX_START_READ_RSP =>
            if tx_busy = '0' then
              tx_start <= '1';
              ex_state <= EX_IDLE;
            end if;

          when EX_PREP_ERR_RSP =>
            if tx_busy = '0' then
              tx_payload_buf(0) <= C_ERR_PAY;
              tx_payload_len    <= 1;
              ex_state          <= EX_START_ERR_RSP;
            end if;

          when EX_START_ERR_RSP =>
            if tx_busy = '0' then
              tx_start     <= '1';
              err_rsp_take <= '1';
              ex_state     <= EX_IDLE;
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
            tx_len_byte <= std_logic_vector(to_unsigned(tx_payload_len, 8));
            tx_checksum <= (others => '0');
            tx_index    <= 0;
            tx_state    <= TX_SEND_SOF;

          when TX_SEND_SOF =>
            tx_busy <= '1';
            if txq_full = '0' then
              txq_data_in <= C_SOF;
              txq_wr_en   <= '1';
              tx_state    <= TX_SEND_LEN;
            end if;

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
                tx_checksum <= f_checksum_next(tx_checksum, tx_payload_buf(tx_index));
                tx_index    <= tx_index + 1;
              end if;
            else
              tx_state <= TX_SEND_CHKSUM;
            end if;

          when TX_SEND_CHKSUM =>
            tx_busy <= '1';
            if txq_full = '0' then
              txq_data_in <= f_checksum_final(tx_checksum);
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