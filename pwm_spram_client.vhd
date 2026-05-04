-- ============================================================================
-- File: pwm_spram_client.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 24 March 2026
-- Last Updated: 03 May 2026
--
-- Description:
--   Memory-mapped multi-channel PWM client wrapper for the shared SPRAM /
--   Bus_Master architecture.
--
--   Current register usage:
--     0x0100 : global enable in bit 0 for PWM_0..PWM_3
--     0x0101 : PWM frequency control, not blink period
--              0 => fastest PWM
--              65535 => slowest PWM in the bounded configured range
--     0x0103 : PWM_0 duty
--     0x0104 : PWM_1 duty
--     0x0105 : PWM_2 duty
--     0x0106 : PWM_3 duty
--
--   The legacy blink-period register at 0x0101 now controls PWM switching
--   frequency over a bounded range. 0x0102 remains unused. There is no blink
--   envelope in this module.
--
--   Enable behavior:
--     reg_enable(0) = '1' enables all four PWM outputs
--     reg_enable(0) = '0' disables all four PWM outputs
--
-- Revision History:
--   2026-04-07
--     - Reduced PWM channels from 8 to 4 (PWM_0 through PWM_3 only).
--     - Removed PWM_4 through PWM_7 from all logic, registers, and outputs.
--
--   2026-04-22
--     - Removed blink behavior from the LED PWM client.
--     - At that revision, 0x0101 and 0x0102 were ignored here.
--     - PWM outputs are driven directly from the duty registers at 0x0103+.
--     - Changed 0x0100 bit 0 to act as a global enable for all four LEDs.
--
--   2026-05-03
--     - Reassigned 0x0101 from ignored legacy blink-period storage to a live
--       bounded 16-bit PWM frequency control.
--     - Replaced fixed-divider pwm_generic instances with local PWM engines
--       that share the runtime frequency control.
--     - Duty registers are scaled from the 16-bit LabVIEW words into the
--       configured PWM width by taking the most significant bits.
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
    PWM_WIDTH          : integer := 16;
    PWM_DIVIDER        : integer := 1;
    PWM_FREQ_CTRL_MAX_DIV : integer := 6;
    REG_BASE_ADDR      : integer := 16#0100#;
    POLL_CYCLES        : integer := 25000
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
    ST_READ_CAPTURE,
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
  signal read_data_r : std_logic_vector(15 downto 0) := (others => '0');

  ------------------------------------------------------------------------------
  -- Local shadow registers
  ------------------------------------------------------------------------------
  signal reg_enable       : std_logic_vector(7 downto 0) := (others => '0');
  signal reg_freq_ctrl    : std_logic_vector(15 downto 0) := (others => '0');
  signal pwm_freq_div_r   : unsigned(15 downto 0) := to_unsigned(PWM_DIVIDER, 16);
  signal pwm_duty         : t_duty_array := (others => (others => '0'));

  ------------------------------------------------------------------------------
  -- Per-channel enables / outputs
  ------------------------------------------------------------------------------
  signal pwm_enable : std_logic_vector(3 downto 0) := (others => '0');
  signal pwm_raw    : t_pwm_logic_array := (others => '0');
  signal pwm_count  : unsigned(PWM_WIDTH-1 downto 0) := (others => '0');
  signal pwm_duty_latched : t_duty_array := (others => (others => '0'));
  signal pwm_div_count : unsigned(15 downto 0) := (others => '0');

  function f_scale_duty(
    value_in : std_logic_vector(15 downto 0)
  ) return unsigned is
    variable result_v : unsigned(PWM_WIDTH-1 downto 0);
  begin
    if PWM_WIDTH <= 16 then
      result_v := unsigned(value_in(15 downto 16-PWM_WIDTH));
    else
      result_v := resize(unsigned(value_in), PWM_WIDTH);
    end if;

    return result_v;
  end function;

  function f_freq_ctrl_to_div(
    value_in : std_logic_vector(15 downto 0)
  ) return unsigned is
    variable ctrl_v : integer;
    variable div_v  : integer;
  begin
    ctrl_v := to_integer(unsigned(value_in));

    if PWM_FREQ_CTRL_MAX_DIV <= 1 then
      div_v := 1;
    else
      div_v := 1 + ((ctrl_v * (PWM_FREQ_CTRL_MAX_DIV - 1) + 32767) / 65535);
    end if;

    return to_unsigned(div_v, 16);
  end function;

begin

  bus_req <= bus_req_r;

  curr_addr <= std_logic_vector(to_unsigned(REG_BASE_ADDR + reg_index, 10));

  ------------------------------------------------------------------------------
  -- Polling FSM
  ------------------------------------------------------------------------------
  p_client_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        state      <= ST_POLL;
        poll_count <= POLL_CYCLES-1;
        reg_index  <= 0;
        read_data_r <= (others => '0');

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
              read_data_r <= bus_rsp.rdata;
              bus_req_r.req <= '0';
              state         <= ST_READ_CAPTURE;
            end if;

          when ST_READ_CAPTURE =>
            case reg_index is
              when 0 => reg_enable <= read_data_r(7 downto 0);
              when 1 =>
                reg_freq_ctrl <= read_data_r;
                pwm_freq_div_r <= f_freq_ctrl_to_div(read_data_r);
              when 2 => null;
              when 3 => pwm_duty(0) <= f_scale_duty(read_data_r);
              when 4 => pwm_duty(1) <= f_scale_duty(read_data_r);
              when 5 => pwm_duty(2) <= f_scale_duty(read_data_r);
              when 6 => pwm_duty(3) <= f_scale_duty(read_data_r);
              when others => null;
            end case;

            state <= ST_NEXT_REG;

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
  -- Enable mapping
  ------------------------------------------------------------------------------
  pwm_enable(0) <= reg_enable(0);
  pwm_enable(1) <= reg_enable(0);
  pwm_enable(2) <= reg_enable(0);
  pwm_enable(3) <= reg_enable(0);

  ------------------------------------------------------------------------------
  -- Runtime-divider PWM generators
  ------------------------------------------------------------------------------
  p_pwm : process(clk_sys)
    variable divider_v : unsigned(15 downto 0);
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        pwm_count        <= (others => '0');
        pwm_duty_latched <= (others => (others => '0'));
        pwm_div_count    <= (others => '0');
        pwm_raw          <= (others => '0');
      else
        divider_v := pwm_freq_div_r;

        if divider_v = 0 then
          divider_v := to_unsigned(1, divider_v'length);
        end if;

        if pwm_div_count >= (divider_v - 1) then
          pwm_div_count <= (others => '0');

          if pwm_count = 0 then
            pwm_duty_latched <= pwm_duty;
          end if;

          for i in 0 to 3 loop
            if (pwm_enable(i) = '1') and (pwm_count < pwm_duty_latched(i)) then
              pwm_raw(i) <= '1';
            else
              pwm_raw(i) <= '0';
            end if;
          end loop;

          pwm_count <= pwm_count + 1;
        else
          pwm_div_count <= pwm_div_count + 1;
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- Outputs
  ------------------------------------------------------------------------------
  PWM_0 <= pwm_raw(0);
  PWM_1 <= pwm_raw(1);
  PWM_2 <= pwm_raw(2);
  PWM_3 <= pwm_raw(3);

end architecture;
