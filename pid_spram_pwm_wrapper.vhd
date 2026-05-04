-- ============================================================================
-- File: pid_spram_pwm_wrapper.vhd
--
-- Creator: Kevin Lyon / Codex
-- Date Created: 22 April 2026
-- Last Updated: 03 May 2026
--
-- Description:
--   Memory-mapped PID + PWM wrapper for the shared SPRAM / Bus_Master
--   architecture.
--
--   This module reads PID configuration and live process values from SPRAM,
--   executes one step of My_PID at a fixed hardware update rate matched to the
--   ADC sweep period, and converts the resulting signed PID output into the
--   duty-cycle input of a dedicated runtime-divider PWM generator.
--
-- Register map used by this wrapper:
--
--   0x0100 : PWM enable
--            Bit 0 is treated as the overall PWM enable. When it is zero,
--            this wrapper forces the buck PWM duty low and clears/holds the
--            PID integrator so toggling the LabVIEW enable acts as a manual
--            integral reset.
--   0x0101 : PID PWM frequency control
--            0 => fastest PWM
--            65535 => slowest PWM in the bounded configured range
--
--            PWM frequency is:
--              f_pwm = clk_sys / (effective_divider * 2^PWM_COUNTER_WIDTH)
--
--   0x0103 : PID setpoint
--            Shared intentionally with the PWM_0 duty register
--   0x0201 : PID feedback input
--   0x0300 : Kp
--   0x0301 : Ki
--   0x0302 : Kd
--   0x0310 : Debug scaled setpoint (12-bit ADC-domain value, zero-extended)
--   0x0311 : Debug feedback
--   0x0312 : Debug error
--   0x0313 : Debug PID output (low 16 bits after saturation to 16-bit signed)
--   0x0314 : Debug PWM duty (zero-extended)
--   0x0315+ : Debug PID accumulator image in 16-bit words, most-significant
--             word first. The number of words depends on PID_ACCUM_WIDTH.
--
-- Fixed-point interpretation of Kp / Ki / Kd:
--
--   The coefficient registers are treated as unsigned fixed-point values with
--   PID_COEFF_FRAC_BITS fractional bits:
--
--       real_gain = register_value / 2^PID_COEFF_FRAC_BITS
--
--   Default example with PID_COEFF_FRAC_BITS = 10:
--
--       0x0400 = 1.0
--       0x0200 = 0.5
--       0x0100 = 0.25
--
-- Startup defaults written by this wrapper into SPRAM once after reset:
--
--       Kp = 0x0050
--       Ki = 0x0005
--       Kd = 0x0000  = 0.0
--
-- Integral-term scaling:
--
--   The instantiated My_PID core also applies INTEGRATOR_SHIFT, which shifts
--   the stored accumulator right before the I-term multiply. Larger values
--   make the integral contribution build more gently.
--
-- Notes:
--   * error = scaled_setpoint - feedback
--   * The 16-bit setpoint register is scaled to the ADC domain by dividing by
--     16 before subtraction, so:
--         0x0000 => target 0
--         0xFFFF => target 4095 (full-scale 12-bit ADC)
--   * The feedback and setpoint words are interpreted as unsigned 16-bit
--     magnitudes and expanded to signed math for subtraction.
--   * Negative PID output is clipped to zero before driving PWM duty.
--   * Positive PID output is saturated to the available PWM range.
--   * On startup, this wrapper writes conservative default PID gains into
--     SPRAM before beginning normal PID updates.
--   * The wrapper re-reads the PWM enable, PWM divider, Kp, Ki, Kd, feedback, and
--     setpoint before each PID update.
--   * If the live Ki register is read as zero, this wrapper pulses the PID
--     core's accumulator-clear input before executing the next PID step.
--   * If the live PWM enable bit at 0x0100 is zero, this wrapper skips the
--     PID execute pulse, forces duty to zero, and repeatedly clears the
--     integrator so disabling PWM from LabVIEW can be used as a manual reset.
--   * The update scheduler is free-running and intentionally decoupled from
--     the PWM frequency. It is hardwired by PID_UPDATE_CLKS so the PID can be
--     matched to the ADC sweep/update rate.
--   * Register 0x0101 was changed on 03 May 2026 from a PID update divider
--     into a bounded 16-bit runtime PWM frequency control so the LabVIEW
--     LED_BlinkPRD control can adjust buck switching frequency without
--     changing PID sample rate.
--   * My_PID is internally pipelined, so this wrapper waits the required
--     extra clocks before latching the updated PID output into the PWM duty
--     register. One additional wrapper wait state is included so the PID
--     output register has completed its clocked update before duty_i is
--     sampled here.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.bus_pkg.all;

entity pid_spram_pwm_wrapper is
  generic (
    PID_ERROR_WIDTH      : positive := 17;
    PID_COEFF_WIDTH      : positive := 16;
    PID_COEFF_FRAC_BITS  : natural  := 10;
    PID_INTEGRATOR_SHIFT : natural  := 0;
    PID_ACCUM_WIDTH      : positive := 32;
    PID_OUTPUT_WIDTH     : positive := 24;
    PWM_COUNTER_WIDTH    : positive := 12;
    PWM_DIVIDER          : positive := 1;
    PWM_FREQ_CTRL_MAX_DIV : positive := 6;
    PID_UPDATE_CLKS      : positive := 65536;
    PWM_DIV_ADDR         : integer  := 16#0101#;
    SETPOINT_ADDR        : integer  := 16#0103#;
    FEEDBACK_ADDR        : integer  := 16#0201#;
    KP_ADDR              : integer  := 16#0300#;
    KI_ADDR              : integer  := 16#0301#;
    KD_ADDR              : integer  := 16#0302#
  );
  port (
    clk_sys   : in  std_logic;
    reset_i   : in  std_logic;
    bus_req   : out t_bus_req;
    bus_rsp   : in  t_bus_rsp;
    pid_pwm_o : out std_logic
  );
end entity;

architecture rtl of pid_spram_pwm_wrapper is

  constant C_NUM_READS       : integer := 7;
  constant C_DBG_FIXED_WORDS : integer := 5;
  constant C_DBG_ACCUM_WORDS : integer := (PID_ACCUM_WIDTH + 15) / 16;
  constant C_DBG_WORDS_TOTAL : integer := C_DBG_FIXED_WORDS + C_DBG_ACCUM_WORDS;
  constant C_KP_INIT         : std_logic_vector(15 downto 0) := x"0050";
  constant C_KI_INIT         : std_logic_vector(15 downto 0) := x"0005";
  constant C_KD_INIT         : std_logic_vector(15 downto 0) := x"0000";

  type t_state is (
    ST_INIT_WRITE,
    ST_INIT_WAIT_ACK,
    ST_INIT_GAP,
    ST_IDLE,
    ST_REQ_READ,
    ST_WAIT_ACK,
    ST_READ_CAPTURE,
    ST_REQ_GAP,
    ST_HOLD_DISABLED,
    ST_CLEAR_ACCUM,
    ST_EXEC_PULSE,
    ST_WAIT_PID_1,
    ST_WAIT_PID_2,
    ST_WAIT_PID_3,
    ST_LATCH_PWM,
    ST_DBG_WRITE,
    ST_DBG_WAIT_ACK,
    ST_DBG_GAP
  );

  signal state : t_state := ST_IDLE;

  signal bus_req_r : t_bus_req := (
    req   => '0',
    we    => '0',
    addr  => (others => '0'),
    wdata => (others => '0')
  );

  signal read_index_r      : integer range 0 to C_NUM_READS-1 := 0;
  signal read_data_r       : std_logic_vector(15 downto 0) := (others => '0');
  signal update_period_count_r : integer range 0 to PID_UPDATE_CLKS-1 := 0;
  signal update_tick_r         : std_logic := '0';

  signal update_pending_r : std_logic := '0';
  signal pwm_enable_r     : std_logic := '0';
  signal pwm_divider_r    : unsigned(15 downto 0) := to_unsigned(PWM_DIVIDER, 16);

  signal setpoint_r : std_logic_vector(15 downto 0) := (others => '0');
  signal feedback_r : std_logic_vector(15 downto 0) := (others => '0');
  signal kp_r       : unsigned(PID_COEFF_WIDTH-1 downto 0) := (others => '0');
  signal ki_r       : unsigned(PID_COEFF_WIDTH-1 downto 0) := (others => '0');
  signal kd_r       : unsigned(PID_COEFF_WIDTH-1 downto 0) := (others => '0');
  signal setpoint_adc_s : unsigned(11 downto 0);

  signal pid_error_s   : signed(PID_ERROR_WIDTH-1 downto 0);
  signal pid_execute_r : std_logic := '0';
  signal pid_clear_accum_r : std_logic := '0';
  signal pid_output_s  : signed(PID_OUTPUT_WIDTH-1 downto 0);
  signal pwm_duty_r    : unsigned(PWM_COUNTER_WIDTH-1 downto 0) := (others => '0');
  signal pwm_count_r   : unsigned(PWM_COUNTER_WIDTH-1 downto 0) := (others => '0');
  signal pwm_duty_latched_r : unsigned(PWM_COUNTER_WIDTH-1 downto 0) := (others => '0');
  signal pwm_div_count_r : unsigned(15 downto 0) := (others => '0');
  signal pid_pwm_r     : std_logic := '0';
  signal init_index_r  : integer range 0 to 2 := 0;
  signal dbg_index_r   : integer range 0 to C_DBG_WORDS_TOTAL-1 := 0;
  signal dbg_setpoint_r : std_logic_vector(15 downto 0) := (others => '0');
  signal dbg_feedback_r : std_logic_vector(15 downto 0) := (others => '0');
  signal dbg_error_r    : std_logic_vector(15 downto 0) := (others => '0');
  signal dbg_pid_r      : std_logic_vector(15 downto 0) := (others => '0');
  signal dbg_pwm_r      : std_logic_vector(15 downto 0) := (others => '0');
  signal pid_accum_s    : signed(PID_ACCUM_WIDTH-1 downto 0);

  function f_read_addr(
    idx : integer
  ) return std_logic_vector is
  begin
    case idx is
      when 0 =>
        return std_logic_vector(to_unsigned(16#0100#, 10));
      when 1 =>
        return std_logic_vector(to_unsigned(PWM_DIV_ADDR, 10));
      when 2 =>
        return std_logic_vector(to_unsigned(KP_ADDR, 10));
      when 3 =>
        return std_logic_vector(to_unsigned(KI_ADDR, 10));
      when 4 =>
        return std_logic_vector(to_unsigned(KD_ADDR, 10));
      when 5 =>
        return std_logic_vector(to_unsigned(FEEDBACK_ADDR, 10));
      when others =>
        return std_logic_vector(to_unsigned(SETPOINT_ADDR, 10));
    end case;
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

  function f_sat_to_pwm(
    value_in    : signed;
    target_size : positive
  ) return unsigned is
    variable result_v : unsigned(target_size-1 downto 0);
    variable fits_v   : boolean;
  begin
    if value_in(value_in'high) = '1' then
      result_v := (others => '0');
    elsif value_in'length <= target_size then
      result_v := resize(unsigned(value_in), target_size);
    else
      fits_v := true;

      for idx in value_in'high downto target_size loop
        if value_in(idx) = '1' then
          fits_v := false;
        end if;
      end loop;

      if fits_v then
        result_v := unsigned(value_in(target_size-1 downto 0));
      else
        result_v := (others => '1');
      end if;
    end if;

    return result_v;
  end function;

  function f_init_addr(
    idx : integer
  ) return std_logic_vector is
  begin
    case idx is
      when 0 =>
        return std_logic_vector(to_unsigned(KP_ADDR, 10));
      when 1 =>
        return std_logic_vector(to_unsigned(KI_ADDR, 10));
      when others =>
        return std_logic_vector(to_unsigned(KD_ADDR, 10));
    end case;
  end function;

  function f_init_data(
    idx : integer
  ) return std_logic_vector is
  begin
    case idx is
      when 0 =>
        return C_KP_INIT;
      when 1 =>
        return C_KI_INIT;
      when others =>
        return C_KD_INIT;
    end case;
  end function;

  function f_dbg_addr(
    idx : integer
  ) return std_logic_vector is
  begin
    case idx is
      when 0 =>
        return std_logic_vector(to_unsigned(16#0310#, 10));
      when 1 =>
        return std_logic_vector(to_unsigned(16#0311#, 10));
      when 2 =>
        return std_logic_vector(to_unsigned(16#0312#, 10));
      when 3 =>
        return std_logic_vector(to_unsigned(16#0313#, 10));
      when 4 =>
        return std_logic_vector(to_unsigned(16#0314#, 10));
      when others =>
        return std_logic_vector(to_unsigned(16#0315# + (idx - C_DBG_FIXED_WORDS), 10));
    end case;
  end function;

  function f_dbg_data(
    idx       : integer;
    setpoint  : std_logic_vector(15 downto 0);
    feedback  : std_logic_vector(15 downto 0);
    err_word  : std_logic_vector(15 downto 0);
    pid_word  : std_logic_vector(15 downto 0);
    pwm_word  : std_logic_vector(15 downto 0);
    accum_word : signed
  ) return std_logic_vector is
    variable word_idx_v  : integer;
    variable accum_ext_v : signed(C_DBG_ACCUM_WORDS*16-1 downto 0);
  begin
    case idx is
      when 0 =>
        return setpoint;
      when 1 =>
        return feedback;
      when 2 =>
        return err_word;
      when 3 =>
        return pid_word;
      when 4 =>
        return pwm_word;
      when others =>
        word_idx_v  := (C_DBG_ACCUM_WORDS - 1) - (idx - C_DBG_FIXED_WORDS);
        accum_ext_v := resize(accum_word, C_DBG_ACCUM_WORDS*16);
        return std_logic_vector(accum_ext_v((word_idx_v*16)+15 downto word_idx_v*16));
    end case;
  end function;

begin

  bus_req <= bus_req_r;

  setpoint_adc_s <= unsigned(setpoint_r(15 downto 4));

  pid_error_s <= resize(signed('0' & std_logic_vector(setpoint_adc_s)), PID_ERROR_WIDTH)
               - resize(signed('0' & feedback_r), PID_ERROR_WIDTH);

  u_pid : entity work.My_PID
    generic map (
      ERROR_WIDTH  => PID_ERROR_WIDTH,
      COEFF_WIDTH  => PID_COEFF_WIDTH,
      FRAC_BITS    => PID_COEFF_FRAC_BITS,
      INTEGRATOR_SHIFT => PID_INTEGRATOR_SHIFT,
      ACCUM_WIDTH  => PID_ACCUM_WIDTH,
      OUTPUT_WIDTH => PID_OUTPUT_WIDTH
    )
    port map (
      clk           => clk_sys,
      reset_i       => reset_i,
      clear_accum_i => pid_clear_accum_r,
      execute_i     => pid_execute_r,
      error_i       => pid_error_s,
      kp_i          => kp_r,
      ki_i          => ki_r,
      kd_i          => kd_r,
      accum_o       => pid_accum_s,
      pid_o         => pid_output_s
    );

  pid_pwm_o <= pid_pwm_r;

  p_pid_pwm : process(clk_sys)
    variable divider_v : unsigned(15 downto 0);
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        pwm_count_r        <= (others => '0');
        pwm_duty_latched_r <= (others => '0');
        pwm_div_count_r    <= (others => '0');
        pid_pwm_r          <= '0';
      else
        divider_v := pwm_divider_r;

        if divider_v = 0 then
          divider_v := to_unsigned(1, divider_v'length);
        end if;

        if pwm_div_count_r >= (divider_v - 1) then
          pwm_div_count_r <= (others => '0');

          if pwm_count_r = 0 then
            pwm_duty_latched_r <= pwm_duty_r;
          end if;

          if pwm_count_r < pwm_duty_latched_r then
            pid_pwm_r <= '1';
          else
            pid_pwm_r <= '0';
          end if;

          pwm_count_r <= pwm_count_r + 1;
        else
          pwm_div_count_r <= pwm_div_count_r + 1;
        end if;
      end if;
    end if;
  end process;

  p_update_timer : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        update_period_count_r <= 0;
        update_tick_r         <= '0';
      else
        update_tick_r <= '0';

        if update_period_count_r = PID_UPDATE_CLKS - 1 then
          update_period_count_r <= 0;
          update_tick_r         <= '1';
        else
          update_period_count_r <= update_period_count_r + 1;
        end if;
      end if;
    end if;
  end process;

  p_ctrl : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      bus_req_r.req   <= '0';
      bus_req_r.we    <= '0';
      bus_req_r.addr  <= (others => '0');
      bus_req_r.wdata <= (others => '0');
      pid_execute_r   <= '0';
      pid_clear_accum_r <= '0';

      if reset_i = '1' then
        state            <= ST_INIT_WRITE;
        read_index_r     <= 0;
        read_data_r      <= (others => '0');
        init_index_r     <= 0;
        update_pending_r <= '0';
        pwm_enable_r     <= '0';
        pwm_divider_r    <= to_unsigned(PWM_DIVIDER, pwm_divider_r'length);
        setpoint_r       <= (others => '0');
        feedback_r       <= (others => '0');
        kp_r             <= (others => '0');
        ki_r             <= (others => '0');
        kd_r             <= (others => '0');
        pwm_duty_r       <= (others => '0');
        dbg_index_r      <= 0;
        dbg_setpoint_r   <= (others => '0');
        dbg_feedback_r   <= (others => '0');
        dbg_error_r      <= (others => '0');
        dbg_pid_r        <= (others => '0');
        dbg_pwm_r        <= (others => '0');
      else
        if (update_tick_r = '1') and (update_pending_r = '0') then
          update_pending_r <= '1';
        end if;

        case state is
          when ST_INIT_WRITE =>
            bus_req_r.req   <= '1';
            bus_req_r.we    <= '1';
            bus_req_r.addr  <= f_init_addr(init_index_r);
            bus_req_r.wdata <= f_init_data(init_index_r);
            state           <= ST_INIT_WAIT_ACK;

          when ST_INIT_WAIT_ACK =>
            bus_req_r.req   <= '1';
            bus_req_r.we    <= '1';
            bus_req_r.addr  <= f_init_addr(init_index_r);
            bus_req_r.wdata <= f_init_data(init_index_r);

            if bus_rsp.ack = '1' then
              case init_index_r is
                when 0 =>
                  kp_r <= unsigned(C_KP_INIT);
                when 1 =>
                  ki_r <= unsigned(C_KI_INIT);
                when others =>
                  kd_r <= unsigned(C_KD_INIT);
              end case;

              state <= ST_INIT_GAP;
            end if;

          when ST_INIT_GAP =>
            if init_index_r = 2 then
              state <= ST_IDLE;
            else
              init_index_r <= init_index_r + 1;
              state        <= ST_INIT_WRITE;
            end if;

          when ST_IDLE =>
            if update_pending_r = '1' then
              update_pending_r <= '0';
              read_index_r     <= 0;
              state            <= ST_REQ_READ;
            end if;

          when ST_REQ_READ =>
            bus_req_r.req  <= '1';
            bus_req_r.we   <= '0';
            bus_req_r.addr <= f_read_addr(read_index_r);
            state          <= ST_WAIT_ACK;

          when ST_WAIT_ACK =>
            bus_req_r.req  <= '1';
            bus_req_r.we   <= '0';
            bus_req_r.addr <= f_read_addr(read_index_r);

            if bus_rsp.ack = '1' then
              read_data_r <= bus_rsp.rdata;
              state <= ST_READ_CAPTURE;
            end if;

          when ST_READ_CAPTURE =>
            case read_index_r is
              when 0 =>
                pwm_enable_r <= read_data_r(0);
              when 1 =>
                pwm_divider_r <= f_freq_ctrl_to_div(read_data_r);
              when 2 =>
                kp_r <= unsigned(read_data_r);
              when 3 =>
                ki_r <= unsigned(read_data_r);
              when 4 =>
                kd_r <= unsigned(read_data_r);
              when 5 =>
                feedback_r <= read_data_r;
              when others =>
                setpoint_r <= read_data_r;
            end case;

            state <= ST_REQ_GAP;

          when ST_REQ_GAP =>
            if read_index_r = C_NUM_READS - 1 then
              if pwm_enable_r = '0' then
                state <= ST_HOLD_DISABLED;
              elsif ki_r = 0 then
                state <= ST_CLEAR_ACCUM;
              else
                state <= ST_EXEC_PULSE;
              end if;
            else
              read_index_r <= read_index_r + 1;
              state        <= ST_REQ_READ;
            end if;

          when ST_HOLD_DISABLED =>
            pid_clear_accum_r <= '1';
            pwm_duty_r        <= (others => '0');
            dbg_index_r       <= 0;
            dbg_setpoint_r    <= std_logic_vector(resize(setpoint_adc_s, 16));
            dbg_feedback_r    <= feedback_r;
            dbg_error_r       <= std_logic_vector(resize(pid_error_s, 16));
            dbg_pid_r         <= (others => '0');
            dbg_pwm_r         <= (others => '0');
            state             <= ST_DBG_WRITE;

          when ST_CLEAR_ACCUM =>
            pid_clear_accum_r <= '1';
            state             <= ST_EXEC_PULSE;

          when ST_EXEC_PULSE =>
            pid_execute_r <= '1';
            state <= ST_WAIT_PID_1;

          when ST_WAIT_PID_1 =>
            state <= ST_WAIT_PID_2;

          when ST_WAIT_PID_2 =>
            state <= ST_WAIT_PID_3;

          when ST_WAIT_PID_3 =>
            state <= ST_LATCH_PWM;

          when ST_LATCH_PWM =>
            pwm_duty_r <= f_sat_to_pwm(pid_output_s, PWM_COUNTER_WIDTH);
            dbg_index_r    <= 0;
            dbg_setpoint_r <= std_logic_vector(resize(setpoint_adc_s, 16));
            dbg_feedback_r <= feedback_r;
            dbg_error_r    <= std_logic_vector(resize(pid_error_s, 16));
            dbg_pid_r      <= std_logic_vector(resize(pid_output_s, 16));
            dbg_pwm_r      <= std_logic_vector(resize(f_sat_to_pwm(pid_output_s, PWM_COUNTER_WIDTH), 16));
            state          <= ST_DBG_WRITE;

          when ST_DBG_WRITE =>
            bus_req_r.req   <= '1';
            bus_req_r.we    <= '1';
            bus_req_r.addr  <= f_dbg_addr(dbg_index_r);
            bus_req_r.wdata <= f_dbg_data(
              dbg_index_r,
              dbg_setpoint_r,
              dbg_feedback_r,
              dbg_error_r,
              dbg_pid_r,
              dbg_pwm_r,
              pid_accum_s
            );
            state <= ST_DBG_WAIT_ACK;

          when ST_DBG_WAIT_ACK =>
            bus_req_r.req   <= '1';
            bus_req_r.we    <= '1';
            bus_req_r.addr  <= f_dbg_addr(dbg_index_r);
            bus_req_r.wdata <= f_dbg_data(
              dbg_index_r,
              dbg_setpoint_r,
              dbg_feedback_r,
              dbg_error_r,
              dbg_pid_r,
              dbg_pwm_r,
              pid_accum_s
            );

            if bus_rsp.ack = '1' then
              state <= ST_DBG_GAP;
            end if;

          when ST_DBG_GAP =>
            if dbg_index_r = C_DBG_WORDS_TOTAL - 1 then
              state <= ST_IDLE;
            else
              dbg_index_r <= dbg_index_r + 1;
              state       <= ST_DBG_WRITE;
            end if;

          when others =>
            state <= ST_IDLE;
        end case;
      end if;
    end if;
  end process;

end architecture rtl;
