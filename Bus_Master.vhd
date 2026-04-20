-- ============================================================================
-- File: Bus_Master.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 04 March 2026
-- Last Updated: 07 April 2026
--
-- Description:
--   Prioritized SPRAM wrapper / arbitration layer for the MachXO3D internal
--   SPRAM block.
--
--   Earlier revisions of this module implemented the complete UART command
--   protocol parser and memory execution engine.  That functionality has now
--   been moved into the UART protocol controller
--   (uart_rs232_fifo_wrap_controller.vhd).
--
--   The Bus_Master module is now responsible only for:
--
--     1) Arbitrating access to SPRAM between multiple requesters
--     2) Performing the actual SPRAM read and write transactions
--     3) Returning completion acknowledgements to the requesting client
--
--   Each client presents a simple transaction interface:
--
--       req    : request memory transaction
--       ack    : transaction complete
--       we     : write enable (0 = read, 1 = write)
--       addr   : SPRAM word address
--       wdata  : data to write
--       rdata  : returned read data
--
--   Arbitration policy:
--
--       Fixed priority arbitration.
--       Client 0 has highest priority.
--       Client 1 has middle priority.
--       Client 2 has lowest priority.
--
--   Transaction behavior:
--
--       Read:
--         - Client asserts req with we = 0 and address valid
--         - Bus_Master performs synchronous SPRAM read
--         - rdata returned and ack asserted
--
--       Write:
--         - Client asserts req with we = 1 and address/data valid
--         - Bus_Master performs a two-cycle write sequence
--         - ack asserted after commit cycle
--
--   Addressing:
--
--       SPRAM is 1024 x 16 words
--       Only address bits [9:0] are used.
--
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
--     - Implemented addr16_next staging register to prevent stale
--       address slicing during address construction.
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
--     - Added TX_PREP state in transmit FSM.
--     - Outgoing checksum and LEN are now computed one cycle after tx_start
--       so the response payload and payload length are stable.
--     - Fixes stale TX checksum issue that could omit the final payload byte
--       or use old payload contents.
--
--   2026-03-13
--     - Added COUNT field to both read and write command formats.
--     - Enabled sequential burst reads and writes to SPRAM.
--     - Separated packet reception (RX FSM) from memory execution (EXEC FSM).
--     - Introduced request queue handshake using req_valid / req_take.
--     - Added simple arbitration scaffold for future prioritized bus access.
--     - Changed TX checksum generation to a running XOR accumulated as
--       payload bytes are transmitted.
--     - This fixes the stale TX checksum issue where the final payload byte
--       was omitted from the returned checksum.
--
--   2026-03-16
--     - Major architectural refactor.
--     - UART protocol parsing and packet execution moved to
--         uart_rs232_fifo_wrap_controller.vhd.
--     - Bus_Master redesigned as a reusable SPRAM arbitration wrapper.
--     - Introduced simple req/ack client transaction interface.
--     - Added fixed-priority multi-client arbitration scaffold.
--
--   2026-03-18
--     - Refactored the flat client req/ack signal interface into structured
--       request and response records defined in bus_pkg.vhd.
--     - Updated client 0 and client 1 ports to use t_bus_req and t_bus_rsp.
--     - No intended functional change to arbitration or SPRAM timing.
--     - Added per-client request-consumed flags so each asserted req is
--       serviced only once until that client drops req low.
--     - Removed the separate re-arm process after discovering that the
--       request-consumed flags were being driven from two clocked processes.
--     - Moved both flag set and flag clear behavior into the main bus FSM
--       process so each flag is implemented as a single clean register.
--     - This prevents duplicate transactions from a held req without
--       stalling the main bus FSM and without creating multi-driver logic.
--
--   2026-03-20
--     - Removed the per-client req_seen flags.
--     - Updated the final read and write FSM states to hold until the active
--       client's req line returns low before allowing the bus FSM to return
--       to BUS_IDLE.
--     - This restores one-transaction-per-request behavior using explicit
--       request deassertion rather than request-consumed latches.
--     - Added dedicated post-ack wait states so ack is asserted for only a
--       single clock cycle, after which the FSM waits for the active client's
--       req line to return low before returning to BUS_IDLE.
--
--   2026-04-07
--     - Added a third client port for ADC access.
--     - Updated arbitration to fixed priority:
--         Client 0 = PWM (highest)
--         Client 1 = ADC (middle)
--         Client 2 = UART (lowest)
--     - No intended change to SPRAM transaction timing.
--
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.bus_pkg.all;

entity Bus_Master is
  port (
    clk_sys : in  std_logic;
    reset_i : in  std_logic;

    ----------------------------------------------------------------------------
    -- Client 0 PWM (highest priority)
    ----------------------------------------------------------------------------
    c0_req : in  t_bus_req;
    c0_rsp : out t_bus_rsp;

    ----------------------------------------------------------------------------
    -- Client 1 ADC (middle priority)
    ----------------------------------------------------------------------------
    c1_req : in  t_bus_req;
    c1_rsp : out t_bus_rsp;

    ----------------------------------------------------------------------------
    -- Client 2 RS232 (lowest priority)
    ----------------------------------------------------------------------------
    c2_req : in  t_bus_req;
    c2_rsp : out t_bus_rsp;

    ----------------------------------------------------------------------------
    -- SPRAM interface
    ----------------------------------------------------------------------------
    mem_addr : out std_logic_vector(9 downto 0);
    mem_din  : out std_logic_vector(15 downto 0);
    mem_dout : in  std_logic_vector(15 downto 0);
    mem_we   : out std_logic;
    mem_en   : out std_logic
  );
end entity;

architecture behavioral of Bus_Master is

  ------------------------------------------------------------------------------
  -- Arbiter / transaction FSM
  ------------------------------------------------------------------------------
  type t_bus_state is (
    BUS_IDLE,
    BUS_READ_SETUP,
    BUS_READ_RETURN,
    BUS_READ_WAIT_REQ_LOW,
    BUS_WRITE_SETUP,
    BUS_WRITE_COMMIT,
    BUS_WRITE_WAIT_REQ_LOW
  );

  type t_client_sel is (
    SEL_NONE,
    SEL_C0,
    SEL_C1,
    SEL_C2
  );

  signal bus_state      : t_bus_state  := BUS_IDLE;
  signal active_client  : t_client_sel := SEL_NONE;

  signal active_we      : std_logic := '0';
  signal active_addr    : std_logic_vector(9 downto 0)  := (others => '0');
  signal active_wdata   : std_logic_vector(15 downto 0) := (others => '0');

  signal c0_rsp_r       : t_bus_rsp := (
    ack   => '0',
    rdata => (others => '0')
  );

  signal c1_rsp_r       : t_bus_rsp := (
    ack   => '0',
    rdata => (others => '0')
  );

  signal c2_rsp_r       : t_bus_rsp := (
    ack   => '0',
    rdata => (others => '0')
  );

begin

  c0_rsp <= c0_rsp_r;
  c1_rsp <= c1_rsp_r;
  c2_rsp <= c2_rsp_r;

  ------------------------------------------------------------------------------
  -- Arbitration / SPRAM transaction engine
  ------------------------------------------------------------------------------
  p_bus_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then

      -- defaults every cycle
      c0_rsp_r.ack <= '0';
      c1_rsp_r.ack <= '0';
      c2_rsp_r.ack <= '0';

      mem_en <= '0';
      mem_we <= '0';

      if reset_i = '1' then
        bus_state        <= BUS_IDLE;
        active_client    <= SEL_NONE;
        active_we        <= '0';
        active_addr      <= (others => '0');
        active_wdata     <= (others => '0');

        c0_rsp_r.ack     <= '0';
        c0_rsp_r.rdata   <= (others => '0');
        c1_rsp_r.ack     <= '0';
        c1_rsp_r.rdata   <= (others => '0');
        c2_rsp_r.ack     <= '0';
        c2_rsp_r.rdata   <= (others => '0');

        mem_addr         <= (others => '0');
        mem_din          <= (others => '0');

      else

        case bus_state is

          ----------------------------------------------------------------------
          -- Choose next requester by fixed priority
          ----------------------------------------------------------------------
          when BUS_IDLE =>
            if c0_req.req = '1' then
              active_client <= SEL_C0;
              active_we     <= c0_req.we;
              active_addr   <= c0_req.addr;
              active_wdata  <= c0_req.wdata;

              if c0_req.we = '1' then
                bus_state <= BUS_WRITE_SETUP;
              else
                bus_state <= BUS_READ_SETUP;
              end if;

            elsif c1_req.req = '1' then
              active_client <= SEL_C1;
              active_we     <= c1_req.we;
              active_addr   <= c1_req.addr;
              active_wdata  <= c1_req.wdata;

              if c1_req.we = '1' then
                bus_state <= BUS_WRITE_SETUP;
              else
                bus_state <= BUS_READ_SETUP;
              end if;

            elsif c2_req.req = '1' then
              active_client <= SEL_C2;
              active_we     <= c2_req.we;
              active_addr   <= c2_req.addr;
              active_wdata  <= c2_req.wdata;

              if c2_req.we = '1' then
                bus_state <= BUS_WRITE_SETUP;
              else
                bus_state <= BUS_READ_SETUP;
              end if;

            else
              bus_state <= BUS_IDLE;
            end if;

          ----------------------------------------------------------------------
          -- Read: present address and enable SPRAM
          ----------------------------------------------------------------------
          when BUS_READ_SETUP =>
            mem_addr  <= active_addr;
            mem_en    <= '1';
            mem_we    <= '0';
            bus_state <= BUS_READ_RETURN;

          ----------------------------------------------------------------------
          -- Read: data now valid from synchronous SPRAM
          ----------------------------------------------------------------------
          when BUS_READ_RETURN =>
            if active_client = SEL_C0 then
              c0_rsp_r.rdata <= mem_dout;
              c0_rsp_r.ack   <= '1';
              bus_state      <= BUS_READ_WAIT_REQ_LOW;

            elsif active_client = SEL_C1 then
              c1_rsp_r.rdata <= mem_dout;
              c1_rsp_r.ack   <= '1';
              bus_state      <= BUS_READ_WAIT_REQ_LOW;

            elsif active_client = SEL_C2 then
              c2_rsp_r.rdata <= mem_dout;
              c2_rsp_r.ack   <= '1';
              bus_state      <= BUS_READ_WAIT_REQ_LOW;

            else
              active_client <= SEL_NONE;
              bus_state     <= BUS_IDLE;
            end if;

          ----------------------------------------------------------------------
          -- Read complete: wait for requester to release req before re-arming
          ----------------------------------------------------------------------
          when BUS_READ_WAIT_REQ_LOW =>
            if active_client = SEL_C0 then
              if c0_req.req = '0' then
                active_client <= SEL_NONE;
                bus_state     <= BUS_IDLE;
              else
                bus_state     <= BUS_READ_WAIT_REQ_LOW;
              end if;

            elsif active_client = SEL_C1 then
              if c1_req.req = '0' then
                active_client <= SEL_NONE;
                bus_state     <= BUS_IDLE;
              else
                bus_state     <= BUS_READ_WAIT_REQ_LOW;
              end if;

            elsif active_client = SEL_C2 then
              if c2_req.req = '0' then
                active_client <= SEL_NONE;
                bus_state     <= BUS_IDLE;
              else
                bus_state     <= BUS_READ_WAIT_REQ_LOW;
              end if;

            else
              active_client <= SEL_NONE;
              bus_state     <= BUS_IDLE;
            end if;

          ----------------------------------------------------------------------
          -- Write: setup address/data for one full cycle
          ----------------------------------------------------------------------
          when BUS_WRITE_SETUP =>
            mem_addr  <= active_addr;
            mem_din   <= active_wdata;
            mem_we    <= '0';
            mem_en    <= '0';
            bus_state <= BUS_WRITE_COMMIT;

          ----------------------------------------------------------------------
          -- Write: commit write pulse and issue single-cycle ack
          ----------------------------------------------------------------------
          when BUS_WRITE_COMMIT =>
            mem_addr <= active_addr;
            mem_din  <= active_wdata;
            mem_en   <= '1';
            mem_we   <= '1';

            if active_client = SEL_C0 then
              c0_rsp_r.ack <= '1';
              bus_state    <= BUS_WRITE_WAIT_REQ_LOW;

            elsif active_client = SEL_C1 then
              c1_rsp_r.ack <= '1';
              bus_state    <= BUS_WRITE_WAIT_REQ_LOW;

            elsif active_client = SEL_C2 then
              c2_rsp_r.ack <= '1';
              bus_state    <= BUS_WRITE_WAIT_REQ_LOW;

            else
              active_client <= SEL_NONE;
              bus_state     <= BUS_IDLE;
            end if;

          ----------------------------------------------------------------------
          -- Write complete: wait for requester to release req before re-arming
          ----------------------------------------------------------------------
          when BUS_WRITE_WAIT_REQ_LOW =>
            if active_client = SEL_C0 then
              if c0_req.req = '0' then
                active_client <= SEL_NONE;
                bus_state     <= BUS_IDLE;
              else
                bus_state     <= BUS_WRITE_WAIT_REQ_LOW;
              end if;

            elsif active_client = SEL_C1 then
              if c1_req.req = '0' then
                active_client <= SEL_NONE;
                bus_state     <= BUS_IDLE;
              else
                bus_state     <= BUS_WRITE_WAIT_REQ_LOW;
              end if;

            elsif active_client = SEL_C2 then
              if c2_req.req = '0' then
                active_client <= SEL_NONE;
                bus_state     <= BUS_IDLE;
              else
                bus_state     <= BUS_WRITE_WAIT_REQ_LOW;
              end if;

            else
              active_client <= SEL_NONE;
              bus_state     <= BUS_IDLE;
            end if;

          when others =>
            bus_state <= BUS_IDLE;
        end case;
      end if;
    end if;
  end process;

end architecture;