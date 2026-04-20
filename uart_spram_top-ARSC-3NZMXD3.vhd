library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_spram_top is
  generic (
    -- Clocking (same idea as HW7)
    OSC_FREQ    : string  := "8.31";      -- MHz (internal osc setting)
    CLK_FREQ_HZ : integer := 24930000;    -- MUST match PLL output clock

    -- UART
    BAUD        : integer := 9600;
    DATA_BITS   : integer := 8;
    STOP_BITS   : integer := 1;
    PARITY      : integer := 0            -- 0=None, 1=Even, 2=Odd
  );
  port (
    RX : in  std_logic;
    TX : out std_logic
  );
end entity;

architecture rtl of uart_spram_top is

  -----------------------------------------------------------------------------
  -- Clocking: internal osc to PLL to clk_sys
  -----------------------------------------------------------------------------
  signal osc_8m31 : std_logic;
  signal clk_sys  : std_logic;
  signal pll_lock : std_logic;

  -----------------------------------------------------------------------------
  -- Reset: asserted until PLL locks plus delay counter
  -----------------------------------------------------------------------------
  signal reset_i : std_logic := '1';
  signal rst_cnt : unsigned(19 downto 0) := (others => '0'); -- about 20 ms delay

  -----------------------------------------------------------------------------
  -- UART signals (matches your uart_rs232)
  -----------------------------------------------------------------------------
  signal rx_data        : std_logic_vector(7 downto 0);
  signal rx_valid       : std_logic;
  signal rx_ready       : std_logic;

  signal rx_framing_err : std_logic;
  signal rx_parity_err  : std_logic;
  signal rx_overrun     : std_logic;

  signal tx_data        : std_logic_vector(7 downto 0) := (others => '0');
  signal tx_valid       : std_logic := '0';
  signal tx_ready       : std_logic;

  -----------------------------------------------------------------------------
  -- SPRAM interface signals (1024 deep x 16-bit)
  -----------------------------------------------------------------------------
  signal mem_addr  : std_logic_vector(9 downto 0)  := (others => '0'); -- 0 to 1023
  signal mem_din   : std_logic_vector(15 downto 0) := (others => '0');
  signal mem_dout  : std_logic_vector(15 downto 0);
  signal mem_we    : std_logic := '0';
  signal mem_en    : std_logic := '1';  -- if your SPRAM has a chip-enable

  -----------------------------------------------------------------------------
  -- TX FIFO (byte wide, depth 4)
  -----------------------------------------------------------------------------
  constant TXF_DEPTH : natural := 4;

  type t_txf_mem is array (0 to TXF_DEPTH-1) of std_logic_vector(7 downto 0);
  signal txf_mem   : t_txf_mem := (others => (others => '0'));

  signal txf_wptr  : unsigned(1 downto 0) := (others => '0'); -- 0 to 3
  signal txf_rptr  : unsigned(1 downto 0) := (others => '0'); -- 0 to 3
  signal txf_cnt   : unsigned(2 downto 0) := (others => '0'); -- 0 to 4

  signal txf_full  : std_logic;
  signal txf_empty : std_logic;

  -- Push interface from command FSM into TX FIFO
  signal txf_push      : std_logic := '0';
  signal txf_push_data : std_logic_vector(7 downto 0) := (others => '0');

  -----------------------------------------------------------------------------
  -- "Bus master" / command parser
  -----------------------------------------------------------------------------
  type t_cmd_state is (
    CMD_IDLE,
    CMD_GET_CMD,
	
    CMD_GET_AHI, --Two states for address high and low
	CMD_GET_ALO,
	
    CMD_GET_DHI, --Two states for data high and low
	CMD_GET_DLO,
	
    CMD_DO_WRITE,
    CMD_DO_WRITE_WAIT,   -- hold WE for a full cycle
	
    CMD_DO_READ_ISSUE,
    CMD_DO_READ_WAIT,
	
    CMD_SEND_ACK,        -- enqueue ACK  **Why is this not happening?!
    CMD_SEND_RHI,        -- enqueue read-hi
    CMD_SEND_RLO         -- enqueue read-lo
  );
  signal cmd_state : t_cmd_state := CMD_IDLE;

  signal cmd_byte  : std_logic_vector(7 downto 0) := (others => '0');
  signal addr_hi   : std_logic_vector(7 downto 0) := (others => '0');
  signal addr_lo   : std_logic_vector(7 downto 0) := (others => '0');
  signal data_hi   : std_logic_vector(7 downto 0) := (others => '0');
  signal data_lo   : std_logic_vector(7 downto 0) := (others => '0');

begin

  -----------------------------------------------------------------------------
  -- FIFO flags
  -----------------------------------------------------------------------------
  txf_full  <= '1' when txf_cnt = to_unsigned(TXF_DEPTH, txf_cnt'length) else '0';
  txf_empty <= '1' when txf_cnt = to_unsigned(0,         txf_cnt'length) else '0';

  -----------------------------------------------------------------------------
  -- Internal oscillator module (Internal_clk.vhd): entity clock
  -----------------------------------------------------------------------------
  u_osc : entity work.clock
    generic map (
      OSC_FREQ => OSC_FREQ
    )
    port map (
      clk => osc_8m31
    );

  -----------------------------------------------------------------------------
  -- PLL generated by IPExpress (from HW7): pll_24m93
  -----------------------------------------------------------------------------
  u_pll : entity work.pll_24m93
    port map (
      CLKI  => osc_8m31,
      CLKOP => clk_sys,
      LOCK  => pll_lock
    );

  -----------------------------------------------------------------------------
  -- Reset generator: hold reset until PLL locks plus delay counter
  -----------------------------------------------------------------------------
  p_reset : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if pll_lock = '0' then
        reset_i <= '1';
        rst_cnt <= (others => '0');
      else
        if rst_cnt(rst_cnt'high) = '0' then
          rst_cnt <= rst_cnt + 1;
          reset_i <= '1';
        else
          reset_i <= '0';
        end if;
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- UART instance (UART_rs232.vhd)
  -----------------------------------------------------------------------------
  u_uart : entity work.uart_rs232
    generic map (
      CLK_FREQ_HZ => CLK_FREQ_HZ,
      BAUD        => BAUD,
      DATA_BITS   => DATA_BITS,
      STOP_BITS   => STOP_BITS,
      PARITY      => PARITY
    )
    port map (
      clk            => clk_sys,
      reset          => reset_i,

      rx             => RX,
      tx             => TX,

      rx_data        => rx_data,
      rx_valid       => rx_valid,
      rx_ready       => rx_ready,
      rx_framing_err => rx_framing_err,
      rx_parity_err  => rx_parity_err,
      rx_overrun     => rx_overrun,

      tx_data        => tx_data,
      tx_valid       => tx_valid,
      tx_ready       => tx_ready
    );

  -----------------------------------------------------------------------------
  -- SPRAM instance (SPRAM.vhd generated by SCUBA)
  -----------------------------------------------------------------------------
  u_spram : entity work.SPRAM
    port map (
      Address => mem_addr,
      Data    => mem_din,
      Clock   => clk_sys,
      WE      => mem_we,
      ClockEn => mem_en,
      Q       => mem_dout
    );

  -----------------------------------------------------------------------------
  -- RX ready logic:
  -- 1) do not accept new bytes if TX FIFO is full
  -- 2) do not accept bytes while we are trying to enqueue response bytes
  -----------------------------------------------------------------------------
  rx_ready <= '1' when (txf_full = '0' and
                        cmd_state /= CMD_SEND_ACK and
                        cmd_state /= CMD_SEND_RHI and
                        cmd_state /= CMD_SEND_RLO)
             else '0';

  -----------------------------------------------------------------------------
  -- TX FIFO + TX engine (single owner of tx_data and tx_valid)
  -- Pops one byte when tx_ready and FIFO not empty
  -- Pushes one byte when txf_push asserted by command FSM
  -----------------------------------------------------------------------------
  p_txfifo : process(clk_sys)
    variable do_pop  : boolean;
    variable do_push : boolean;
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        tx_valid <= '0';
        tx_data  <= (others => '0');

        txf_wptr <= (others => '0');
        txf_rptr <= (others => '0');
        txf_cnt  <= (others => '0');

      else
        tx_valid <= '0';  -- default: pulse only when we actually pop

        do_pop  := (tx_ready = '1') and (txf_empty = '0');
        do_push := (txf_push  = '1') and (txf_full  = '0');

        -- POP (send)
        if do_pop then
          tx_data  <= txf_mem(to_integer(txf_rptr));
          tx_valid <= '1';
          txf_rptr <= txf_rptr + 1;
        end if;

        -- PUSH (enqueue)
        if do_push then
          txf_mem(to_integer(txf_wptr)) <= txf_push_data;
          txf_wptr <= txf_wptr + 1;
        end if;

        -- Update count (handle simultaneous push/pop)
        if do_push and (not do_pop) then
          txf_cnt <= txf_cnt + 1;
        elsif do_pop and (not do_push) then
          txf_cnt <= txf_cnt - 1;
        else
          -- both or neither: count unchanged
          txf_cnt <= txf_cnt;
        end if;

      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  -- Command / Bus-master FSM
  -- Protocol:
  --   Write: 57 addr_hi addr_lo data_hi data_lo  returns 06 (ACK)
  --   Read : 52 addr_hi addr_lo                  returns data_hi data_lo
  -----------------------------------------------------------------------------
  p_cmd : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        cmd_state <= CMD_IDLE;

        mem_addr  <= (others => '0');
        mem_din   <= (others => '0');
        mem_we    <= '0';
        mem_en    <= '1';

        cmd_byte  <= (others => '0');
        addr_hi   <= (others => '0');
        addr_lo   <= (others => '0');
        data_hi   <= (others => '0');
        data_lo   <= (others => '0');

        txf_push      <= '0';
        txf_push_data <= (others => '0');

      else
        -- defaults
        mem_we   <= '0';
        txf_push <= '0';

        case cmd_state is   --Begin State Machine....

          when CMD_IDLE =>
            if rx_valid = '1' and rx_ready = '1' then
              cmd_byte  <= rx_data;
              cmd_state <= CMD_GET_CMD;
            end if;

          when CMD_GET_CMD =>
            if cmd_byte = x"57" then           -- W
              cmd_state <= CMD_GET_AHI;
            elsif cmd_byte = x"52" then        -- R
              cmd_state <= CMD_GET_AHI;
            else
              cmd_state <= CMD_IDLE;
            end if;

          when CMD_GET_AHI =>
            if rx_valid = '1' and rx_ready = '1' then
              addr_hi   <= rx_data;
              cmd_state <= CMD_GET_ALO;
            end if;

          when CMD_GET_ALO =>
            if rx_valid = '1' and rx_ready = '1' then
              addr_lo  <= rx_data;

              -- 10-bit address from two bytes: addr_hi[1:0] plus addr_lo[7:0]
              mem_addr <= addr_hi(1 downto 0) & rx_data;

              if cmd_byte = x"57" then         -- W
                cmd_state <= CMD_GET_DHI;
              else                              -- R
                cmd_state <= CMD_DO_READ_ISSUE;
              end if;
            end if;

          when CMD_GET_DHI =>
            if rx_valid = '1' and rx_ready = '1' then
              data_hi   <= rx_data;
              cmd_state <= CMD_GET_DLO;
            end if;

          when CMD_GET_DLO =>
            if rx_valid = '1' and rx_ready = '1' then
              data_lo   <= rx_data;
              cmd_state <= CMD_DO_WRITE;
            end if;

          when CMD_DO_WRITE =>
            -- Assert WE for a full cycle; write commits on next rising edge
            mem_din   <= data_hi & data_lo;
            mem_we    <= '1';
            cmd_state <= CMD_DO_WRITE_WAIT;

          when CMD_DO_WRITE_WAIT =>
            -- Now queue ACK
            cmd_state <= CMD_SEND_ACK;

          when CMD_DO_READ_ISSUE =>
            -- Unregistered output: mem_dout should be valid now
            cmd_state <= CMD_SEND_RHI;

          when CMD_DO_READ_WAIT =>
            -- Not used for unregistered; keep in case output is later registered
            cmd_state <= CMD_SEND_RHI;

          -------------------------------------------------------------------
          -- "SEND" states now enqueue into TX FIFO
          -------------------------------------------------------------------
          when CMD_SEND_ACK =>
            if txf_full = '0' then
              txf_push      <= '1';
              txf_push_data <= x"06"; -- ACK
              cmd_state     <= CMD_IDLE;
            end if;

          when CMD_SEND_RHI =>
            if txf_full = '0' then
              txf_push      <= '1';
              txf_push_data <= mem_dout(15 downto 8);
              cmd_state     <= CMD_SEND_RLO;
            end if;

          when CMD_SEND_RLO =>
            if txf_full = '0' then
              txf_push      <= '1';
              txf_push_data <= mem_dout(7 downto 0);
              cmd_state     <= CMD_IDLE;
            end if;

          when others =>
            cmd_state <= CMD_IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture;