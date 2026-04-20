-- ============================================================================
-- File: adc_spram_client.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 24 March 2026
-- Last Updated: 19 April 2026
--
-- Description:
--   Memory-mapped ADC client wrapper for the shared SPRAM / Bus_Master
--   architecture.
--
--   This module periodically starts a conversion on the AD7928 SPI ADC core,
--   captures each valid 12-bit sample, and writes that sample into a selected
--   SPRAM register through the Bus_Master client interface.
--
--   The intent is to make ADC feedback available as a memory-mapped register
--   so that other modules, such as a PID controller or UART interface, may
--   read the latest sampled value without directly interacting with the ADC
--   timing or SPI protocol.
--
--   Current default mapping:
--
--       SPRAM address ADC_BASE_ADDR + channel_number
--           Latest sample for each channel
--
--   The 12-bit ADC result is zero-extended into the 16-bit SPRAM data word:
--
--       bus_req.wdata <= "0000" & adc_result
--
--   Bus behavior:
--     - Write-only client of Bus_Master
--     - Starts a burst ADC sweep once per programmable sweep period
--     - Waits for adc data_valid
--     - Latches most recent valid sample
--     - Issues one write request to the selected SPRAM address
--     - Waits for ack
--     - Deasserts req cleanly before returning to idle
--
--   ADC behavior:
--     - Instantiates ad7928_spi_master from AD7928_Core.vhd
--     - Uses programmable start-to-start period between complete sweeps
--     - Cycles through all eight AD7928 channels in a burst
--     - Uses straight-binary coding and 0..2*REFIN range by default
--
-- Notes:
--   - The AD7928 returns conversion data from the previous frame, so the
--     first completed frame primes the pipeline and does not produce a valid
--     stored sample. This is already handled by ad7928_spi_master via the
--     data_valid_o output.
--   - If a new ADC sample arrives while a previous bus write is still pending,
--     the wrapper retains only the most recent sample.  For the expected
--     sample and bus rates in this project, this is acceptable.
--
-- Revision History:
--
--   2026-03-24
--     - Initial version.
--     - Added periodic AD7928 conversion control.
--     - Added sample capture and Bus_Master write interface.
--     - Added zero-extension of 12-bit ADC sample to 16-bit SPRAM word.
--
--   2026-04-17
--     - Updated from fixed single-channel sampling to round-robin sampling of
--       all eight AD7928 channels.
--     - Added sequential SPRAM storage so each returned channel sample is
--       written to ADC_BASE_ADDR + channel_id.
--     - Updated the scheduler so one complete 8-channel sweep is acquired and
--       written to SPRAM once per SAMPLE_PERIOD_CLKS start-to-start interval.
--
--   2026-04-19
--     - Updated the AD7928 control configuration to drive RANGE=0 so the ADC
--       command format matches the class reference slide's 0V to 2*REFIN mode.
--
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.bus_pkg.all;

entity adc_spram_client is
  generic (
    CLK_FREQ_HZ        : integer := 24930000;
    ADC_CLK_DIV        : positive := 5;
    SAMPLE_PERIOD_CLKS : integer := 24930;
    ADC_BASE_ADDR      : integer := 16#0200#
  );
  port (
    clk_sys : in  std_logic;
    reset_i : in  std_logic;

    -- Client bus to Bus_Master
    bus_req : out t_bus_req;
    bus_rsp : in  t_bus_rsp;

    -- AD7928 pins
    adc_cs_n_o : out std_logic;
    adc_sclk_o : out std_logic;
    adc_din_o  : out std_logic;
    adc_dout_i : in  std_logic
  );
end entity;

architecture rtl of adc_spram_client is

  ------------------------------------------------------------------------------
  -- Bus request register
  ------------------------------------------------------------------------------
  signal bus_req_r : t_bus_req := (
    req   => '0',
    we    => '0',
    addr  => (others => '0'),
    wdata => (others => '0')
  );

  ------------------------------------------------------------------------------
  -- ADC core interface
  ------------------------------------------------------------------------------
  signal adc_start       : std_logic := '0';
  signal adc_busy        : std_logic := '0';
  signal adc_data_valid  : std_logic := '0';
  signal adc_primed      : std_logic := '0';
  signal adc_result      : std_logic_vector(11 downto 0) := (others => '0');
  signal adc_result_chan : std_logic_vector(2 downto 0)  := (others => '0');
  signal adc_raw_word    : std_logic_vector(15 downto 0) := (others => '0');
  signal adc_next_chan   : std_logic_vector(2 downto 0)  := (others => '0');
  signal sweep_start_pulse : std_logic := '0';
  signal discard_next_sample : std_logic := '0';

  ------------------------------------------------------------------------------
  -- Sweep period timer
  ------------------------------------------------------------------------------
  signal sample_count : integer range 0 to SAMPLE_PERIOD_CLKS-1 := SAMPLE_PERIOD_CLKS-1;

  ------------------------------------------------------------------------------
  -- Sweep scheduler
  ------------------------------------------------------------------------------
  type t_sched_state is (
    SCH_WAIT_PERIOD,
    SCH_START_FRAME,
    SCH_WAIT_BUSY_HIGH,
    SCH_WAIT_BUSY_LOW
  );

  signal sched_state      : t_sched_state := SCH_WAIT_PERIOD;
  signal sweep_frame_idx  : integer range 0 to 8 := 0;

  ------------------------------------------------------------------------------
  -- Sample storage
  ------------------------------------------------------------------------------
  signal sample_reg      : std_logic_vector(11 downto 0) := (others => '0');
  signal sample_addr_reg : std_logic_vector(9 downto 0)  := (others => '0');
  signal sample_pending  : std_logic := '0';

  ------------------------------------------------------------------------------
  -- Simple bus-write FSM
  ------------------------------------------------------------------------------
  type t_state is (
    ST_IDLE,
    ST_REQ_WRITE,
    ST_WAIT_ACK
  );

  signal state : t_state := ST_IDLE;

  ------------------------------------------------------------------------------
  -- Constants
  ------------------------------------------------------------------------------
  constant C_ADC_BASE_ADDR : unsigned(9 downto 0) :=
    to_unsigned(ADC_BASE_ADDR, 10);

begin

  bus_req <= bus_req_r;

  ------------------------------------------------------------------------------
  -- AD7928 SPI master
  ------------------------------------------------------------------------------
  u_adc : entity work.ad7928_spi_master
    generic map (
      CLK_DIV => ADC_CLK_DIV
    )
    port map (
      clk           => clk_sys,
      reset_i       => reset_i,

      start_i       => adc_start,
      channel_i     => adc_next_chan,
      range_i       => '0',   -- 0..2*REFIN
      coding_i      => '1',   -- straight binary

      busy_o        => adc_busy,
      data_valid_o  => adc_data_valid,
      primed_o      => adc_primed,

      result_o      => adc_result,
      result_chan_o => adc_result_chan,
      raw_word_o    => adc_raw_word,

      adc_cs_n_o    => adc_cs_n_o,
      adc_sclk_o    => adc_sclk_o,
      adc_din_o     => adc_din_o,
      adc_dout_i    => adc_dout_i
    );

  ------------------------------------------------------------------------------
  -- ADC sweep scheduler
  -- Wait for the programmed sweep period boundary, then issue one complete burst of
  -- channel requests: 0,1,2,3,4,5,6,7 and one final flush frame requesting 0.
  -- Because the AD7928 result corresponds to the previous frame, the first
  -- returned sample of each burst is discarded and the next 8 valid samples
  -- refresh SPRAM addresses ADC_BASE_ADDR + 0 through ADC_BASE_ADDR + 7.
  ------------------------------------------------------------------------------
  p_sample_timer : process(clk_sys)
    variable v_next_cmd : integer range 0 to 7;
  begin
    if rising_edge(clk_sys) then
      adc_start         <= '0';
      sweep_start_pulse <= '0';

      if reset_i = '1' then
        sample_count    <= SAMPLE_PERIOD_CLKS-1;
        adc_start       <= '0';
        adc_next_chan   <= (others => '0');
        sched_state     <= SCH_WAIT_PERIOD;
        sweep_frame_idx <= 0;
      else
        if sample_count < (SAMPLE_PERIOD_CLKS - 1) then
          sample_count <= sample_count + 1;
        end if;

        case sched_state is

          when SCH_WAIT_PERIOD =>
            if (sample_count = SAMPLE_PERIOD_CLKS - 1) and
               (adc_busy = '0') and
               (sample_pending = '0') and
               (state = ST_IDLE) then
              sweep_frame_idx <= 0;
              adc_next_chan   <= (others => '0');
              sched_state     <= SCH_START_FRAME;
            end if;

          when SCH_START_FRAME =>
            if (adc_busy = '0') and (sample_pending = '0') then
              adc_start <= '1';

              if sweep_frame_idx = 0 then
                sweep_start_pulse <= '1';
                sample_count      <= 0;
              end if;

              sched_state <= SCH_WAIT_BUSY_HIGH;
            end if;

          when SCH_WAIT_BUSY_HIGH =>
            if adc_busy = '1' then
              sched_state <= SCH_WAIT_BUSY_LOW;
            end if;

          when SCH_WAIT_BUSY_LOW =>
            if adc_busy = '0' then
              if sweep_frame_idx = 8 then
                sched_state <= SCH_WAIT_PERIOD;
              else
                sweep_frame_idx <= sweep_frame_idx + 1;

                if sweep_frame_idx = 7 then
                  v_next_cmd := 0;
                else
                  v_next_cmd := sweep_frame_idx + 1;
                end if;

                adc_next_chan <= std_logic_vector(to_unsigned(v_next_cmd, adc_next_chan'length));
                sched_state   <= SCH_START_FRAME;
              end if;
            end if;

          when others =>
            sched_state <= SCH_WAIT_PERIOD;

        end case;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Capture the most recent valid ADC sample
  ------------------------------------------------------------------------------
  p_sample_capture : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        sample_reg      <= (others => '0');
        sample_addr_reg <= (others => '0');
        sample_pending  <= '0';
        discard_next_sample <= '0';
      else
        if sweep_start_pulse = '1' then
          discard_next_sample <= '1';
        end if;

        if adc_data_valid = '1' then
          if discard_next_sample = '1' then
            discard_next_sample <= '0';
          else
            sample_reg      <= adc_result;
            sample_addr_reg <= std_logic_vector(
              C_ADC_BASE_ADDR + resize(unsigned(adc_result_chan), sample_addr_reg'length)
            );
            sample_pending  <= '1';
          end if;
        end if;

        if (state = ST_WAIT_ACK) and (bus_rsp.ack = '1') then
          sample_pending <= '0';
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Bus write client FSM
  ------------------------------------------------------------------------------
  p_bus_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        state           <= ST_IDLE;

        bus_req_r.req   <= '0';
        bus_req_r.we    <= '1';
        bus_req_r.addr  <= std_logic_vector(C_ADC_BASE_ADDR);
        bus_req_r.wdata <= (others => '0');

      else
        case state is

          ----------------------------------------------------------------------
          -- Wait for a captured sample to be available
          ----------------------------------------------------------------------
          when ST_IDLE =>
            bus_req_r.req   <= '0';
            bus_req_r.we    <= '1';
            bus_req_r.addr  <= sample_addr_reg;
            bus_req_r.wdata <= "0000" & sample_reg;

            if sample_pending = '1' then
              state <= ST_REQ_WRITE;
            else
              state <= ST_IDLE;
            end if;

          ----------------------------------------------------------------------
          -- Assert one write request
          ----------------------------------------------------------------------
          when ST_REQ_WRITE =>
            bus_req_r.req   <= '1';
            bus_req_r.we    <= '1';
            bus_req_r.addr  <= sample_addr_reg;
            bus_req_r.wdata <= "0000" & sample_reg;

            state <= ST_WAIT_ACK;

          ----------------------------------------------------------------------
          -- Hold req until Bus_Master acknowledges the write
          ----------------------------------------------------------------------
          when ST_WAIT_ACK =>
            bus_req_r.req   <= '1';
            bus_req_r.we    <= '1';
            bus_req_r.addr  <= sample_addr_reg;
            bus_req_r.wdata <= "0000" & sample_reg;

            if bus_rsp.ack = '1' then
              bus_req_r.req <= '0';
              state         <= ST_IDLE;
            else
              state <= ST_WAIT_ACK;
            end if;

          when others =>
            state <= ST_IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture;
