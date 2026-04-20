-- ============================================================================
-- File: pwm_spram_client.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 24 March 2026
-- Last Updated: 07 April 2026
--
-- Description:
--   Memory-mapped multi-channel PWM client wrapper for the shared SPRAM /
--   Bus_Master architecture.
--
--   *** UPDATED 2026-04-07 ***
--   - Reduced PWM channels from 8 to 4 (PWM_0 through PWM_3 only).
--   - Removed PWM_4 through PWM_7 from all logic, registers, and outputs.
--
--   (All other behavior unchanged)
--
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.bus_pkg.all;

entity pwm_spram_client is
  generic (
    CLK_FREQ_HZ        : integer := 24930000;
    PWM_WIDTH          : integer := 12;
    PWM_DIVIDER        : integer := 1;
    REG_BASE_ADDR      : integer := 16#0100#;
    POLL_CYCLES        : integer := 25000;
    BLINK_TICK_CYCLES  : integer := 24930
  );
  port (
    clk_sys : in  std_logic;
    reset_i : in  std_logic;

    bus_req : out t_bus_req;
    bus_rsp : in  t_bus_rsp;

    PWM_0   : out std_logic;
    PWM_1   : out std_logic;
    PWM_2   : out std_logic;
    PWM_3   : out std_logic
  );
end entity;

architecture rtl of pwm_spram_client is

  ------------------------------------------------------------------------------
  -- Register map offsets
  ------------------------------------------------------------------------------
  constant C_ADDR_EN      : integer := REG_BASE_ADDR + 0;
  constant C_ADDR_FREQ    : integer := REG_BASE_ADDR + 1;
  constant C_ADDR_PW      : integer := REG_BASE_ADDR + 2;
  constant C_ADDR_PWM0_DC : integer := REG_BASE_ADDR + 3;
  constant C_NUM_REGS     : integer := 7;

  ------------------------------------------------------------------------------
  -- Types
  ------------------------------------------------------------------------------
  type t_state is (
    ST_POLL,
    ST_REQ,
    ST_WAIT_ACK,
    ST_NEXT_REG
  );

  type t_duty_array is array (0 to 3) of unsigned(PWM_WIDTH-1 downto 0);
  type t_pwm_logic_array is array (0 to 3) of std_logic;

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
  -- FSM / polling
  ------------------------------------------------------------------------------
  signal state      : t_state := ST_POLL;
  signal poll_count : integer range 0 to POLL_CYCLES-1 := POLL_CYCLES-1;
  signal reg_index  : integer range 0 to C_NUM_REGS-1 := 0;
  signal curr_addr  : std_logic_vector(9 downto 0) := (others => '0');

  ------------------------------------------------------------------------------
  -- Local shadow registers
  ------------------------------------------------------------------------------
  signal reg_enable       : std_logic_vector(7 downto 0) := (others => '0');
  signal reg_blink_period : unsigned(15 downto 0) := (others => '0');
  signal reg_pulse_width  : unsigned(15 downto 0) := (others => '0');
  signal pwm_duty         : t_duty_array := (others => (others => '0'));

  ------------------------------------------------------------------------------
  -- Blink timing
  ------------------------------------------------------------------------------
  signal blink_counter        : unsigned(15 downto 0) := (others => '0');
  signal blink_gate           : std_logic := '1';
  signal blink_prescale_count : integer range 0 to BLINK_TICK_CYCLES-1 := 0;
  signal blink_tick           : std_logic := '0';

  ------------------------------------------------------------------------------
  -- Per-channel enables / outputs
  ------------------------------------------------------------------------------
  signal pwm_enable : std_logic_vector(3 downto 0) := (others => '0');
  signal pwm_raw    : t_pwm_logic_array := (others => '0');
  signal pwm_gated  : t_pwm_logic_array := (others => '0');

begin

  bus_req <= bus_req_r;

  curr_addr <= std_logic_vector(to_unsigned(REG_BASE_ADDR + reg_index, 10));

  ------------------------------------------------------------------------------
  -- Polling FSM
  ------------------------------------------------------------------------------
  p_client_fsm : process(clk_sys)
    variable v_rdata_u : unsigned(15 downto 0);
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        state      <= ST_POLL;
        poll_count <= POLL_CYCLES-1;
        reg_index  <= 0;

        bus_req_r.req <= '0';
        bus_req_r.we  <= '0';

      else
        case state is

          when ST_POLL =>
            bus_req_r.req <= '0';

            reg_index <= 0;

            if poll_count = POLL_CYCLES-1 then
              poll_count <= 0;
              state      <= ST_REQ;
            else
              poll_count <= poll_count + 1;
            end if;

          when ST_REQ =>
            bus_req_r.req  <= '1';
            bus_req_r.we   <= '0';
            bus_req_r.addr <= curr_addr;

            state <= ST_WAIT_ACK;

          when ST_WAIT_ACK =>
            if bus_rsp.ack = '1' then
              v_rdata_u := unsigned(bus_rsp.rdata);

              case reg_index is
                when 0 => reg_enable <= bus_rsp.rdata(7 downto 0);
                when 1 => reg_blink_period <= v_rdata_u;
                when 2 => reg_pulse_width  <= v_rdata_u;
                when 3 => pwm_duty(0) <= resize(v_rdata_u, PWM_WIDTH);
                when 4 => pwm_duty(1) <= resize(v_rdata_u, PWM_WIDTH);
                when 5 => pwm_duty(2) <= resize(v_rdata_u, PWM_WIDTH);
                when 6 => pwm_duty(3) <= resize(v_rdata_u, PWM_WIDTH);
                when others => null;
              end case;

              bus_req_r.req <= '0';
              state         <= ST_NEXT_REG;
            end if;

          when ST_NEXT_REG =>
            if reg_index < (C_NUM_REGS - 1) then
              reg_index <= reg_index + 1;
              state     <= ST_REQ;
            else
              reg_index <= 0;
              state     <= ST_POLL;
            end if;

        end case;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Blink prescaler
  ------------------------------------------------------------------------------
  p_blink_prescaler : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        blink_prescale_count <= 0;
        blink_tick           <= '0';
      else
        if blink_prescale_count = BLINK_TICK_CYCLES - 1 then
          blink_prescale_count <= 0;
          blink_tick           <= '1';
        else
          blink_prescale_count <= blink_prescale_count + 1;
          blink_tick           <= '0';
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Blink logic
  ------------------------------------------------------------------------------
  p_blink : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        blink_counter <= (others => '0');
        blink_gate    <= '1';

      else
        if reg_blink_period = 0 then
          blink_gate <= '1';

        elsif blink_tick = '1' then
          if blink_counter >= (reg_blink_period - 1) then
            blink_counter <= (others => '0');
          else
            blink_counter <= blink_counter + 1;
          end if;

          if reg_pulse_width = 0 then
            blink_gate <= '0';
          elsif reg_pulse_width >= reg_blink_period then
            blink_gate <= '1';
          elsif blink_counter < reg_pulse_width then
            blink_gate <= '1';
          else
            blink_gate <= '0';
          end if;
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Enable mapping
  ------------------------------------------------------------------------------
  pwm_enable(0) <= reg_enable(0);
  pwm_enable(1) <= reg_enable(1);
  pwm_enable(2) <= reg_enable(2);
  pwm_enable(3) <= reg_enable(3);

  ------------------------------------------------------------------------------
  -- PWM generators
  ------------------------------------------------------------------------------
  gen_pwm : for i in 0 to 3 generate
    u_pwm : entity work.pwm_generic
      generic map (
        COUNTER_WIDTH => PWM_WIDTH,
        CLK_DIVIDER   => PWM_DIVIDER,
        ACTIVE_HIGH   => true
      )
      port map (
        clk      => clk_sys,
        reset_i  => reset_i,
        enable_i => pwm_enable(i),
        duty_i   => pwm_duty(i),
        pwm_o    => pwm_raw(i)
      );
  end generate;

  ------------------------------------------------------------------------------
  -- Blink gating
  ------------------------------------------------------------------------------
  gen_gate : for i in 0 to 3 generate
    pwm_gated(i) <= pwm_raw(i) when blink_gate = '1' else '0';
  end generate;

  ------------------------------------------------------------------------------
  -- Outputs
  ------------------------------------------------------------------------------
  PWM_0 <= pwm_gated(0);
  PWM_1 <= pwm_gated(1);
  PWM_2 <= pwm_gated(2);
  PWM_3 <= pwm_gated(3);

end architecture;