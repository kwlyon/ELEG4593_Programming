--============================================================
-- File: Bus_Master.vhd
--============================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Bus_Master is
  port (
    clk_sys  : in  std_logic;
    reset_i  : in  std_logic;

    -- UART RX interface (from uart_rs232)
    rx_data  : in  std_logic_vector(7 downto 0);
    rx_valid : in  std_logic;
    rx_ready : out std_logic;

    -- TX FIFO interface (to top-level FIFO)
    txf_full      : in  std_logic;
    txf_push      : out std_logic;
    txf_push_data : out std_logic_vector(7 downto 0);

    -- SPRAM interface
    mem_addr : out std_logic_vector(9 downto 0);
    mem_din  : out std_logic_vector(15 downto 0);
    mem_dout : in  std_logic_vector(15 downto 0);
    mem_we   : out std_logic;
    mem_en   : out std_logic
  );
end entity;

architecture rtl of Bus_Master is

  -----------------------------------------------------------------------------
  -- Internal versions of OUT ports (avoids "reading from out port" errors)
  -----------------------------------------------------------------------------
  signal rx_ready_i     : std_logic := '0';
  signal txf_push_i     : std_logic := '0';
  signal txf_push_data_i: std_logic_vector(7 downto 0) := (others => '0');

  signal mem_addr_i : std_logic_vector(9 downto 0)  := (others => '0');
  signal mem_din_i  : std_logic_vector(15 downto 0) := (others => '0');
  signal mem_we_i   : std_logic := '0';
  signal mem_en_i   : std_logic := '1';

  -----------------------------------------------------------------------------
  -- "Bus master" / command parser states
  -----------------------------------------------------------------------------
  type t_cmd_state is (
    CMD_IDLE,
    CMD_GET_CMD,
    CMD_GET_AHI, CMD_GET_ALO,
    CMD_GET_DHI, CMD_GET_DLO,
    CMD_DO_WRITE,
    CMD_DO_WRITE_WAIT,
    CMD_DO_READ_ISSUE,
    CMD_DO_READ_WAIT,
    CMD_SEND_ACK,
    CMD_SEND_RHI,
    CMD_SEND_RLO
  );
  signal cmd_state : t_cmd_state := CMD_IDLE;

  signal cmd_byte  : std_logic_vector(7 downto 0) := (others => '0');
  signal addr_hi   : std_logic_vector(7 downto 0) := (others => '0');
  signal addr_lo   : std_logic_vector(7 downto 0) := (others => '0');
  signal data_hi   : std_logic_vector(7 downto 0) := (others => '0');
  signal data_lo   : std_logic_vector(7 downto 0) := (others => '0');

begin

  -----------------------------------------------------------------------------
  -- Drive outputs from internal signals
  -----------------------------------------------------------------------------
  rx_ready     <= rx_ready_i;
  txf_push     <= txf_push_i;
  txf_push_data<= txf_push_data_i;

  mem_addr <= mem_addr_i;
  mem_din  <= mem_din_i;
  mem_we   <= mem_we_i;
  mem_en   <= mem_en_i;

  -----------------------------------------------------------------------------
  -- RX ready logic
  -----------------------------------------------------------------------------
  rx_ready_i <= '1' when (txf_full = '0' and
                          cmd_state /= CMD_SEND_ACK and
                          cmd_state /= CMD_SEND_RHI and
                          cmd_state /= CMD_SEND_RLO)
               else '0';

  -----------------------------------------------------------------------------
  -- Command / Bus-master FSM
  -----------------------------------------------------------------------------
  p_cmd : process(clk_sys)
  begin
    if rising_edge(clk_sys) then
      if reset_i = '1' then
        cmd_state <= CMD_IDLE;

        mem_addr_i <= (others => '0');
        mem_din_i  <= (others => '0');
        mem_we_i   <= '0';
        mem_en_i   <= '1';

        cmd_byte   <= (others => '0');
        addr_hi    <= (others => '0');
        addr_lo    <= (others => '0');
        data_hi    <= (others => '0');
        data_lo    <= (others => '0');

        txf_push_i      <= '0';
        txf_push_data_i <= (others => '0');

      else
        -- defaults
        mem_we_i   <= '0';
        mem_en_i   <= '1';
        txf_push_i <= '0';

        case cmd_state is

          when CMD_IDLE =>
            if rx_valid = '1' and rx_ready_i = '1' then
              cmd_byte  <= rx_data;
              cmd_state <= CMD_GET_CMD;
            end if;

          when CMD_GET_CMD =>
            if cmd_byte = x"57" then           -- 'W'
              cmd_state <= CMD_GET_AHI;
            elsif cmd_byte = x"52" then        -- 'R'
              cmd_state <= CMD_GET_AHI;
            else
              cmd_state <= CMD_IDLE;
            end if;

          when CMD_GET_AHI =>
            if rx_valid = '1' and rx_ready_i = '1' then
              addr_hi   <= rx_data;
              cmd_state <= CMD_GET_ALO;
            end if;

          when CMD_GET_ALO =>
            if rx_valid = '1' and rx_ready_i = '1' then
              addr_lo <= rx_data;

              -- 10-bit address: addr_hi[1:0] & addr_lo[7:0]
              mem_addr_i <= addr_hi(1 downto 0) & rx_data;

              if cmd_byte = x"57" then
                cmd_state <= CMD_GET_DHI;
              else
                cmd_state <= CMD_DO_READ_ISSUE;
              end if;
            end if;

          when CMD_GET_DHI =>
            if rx_valid = '1' and rx_ready_i = '1' then
              data_hi   <= rx_data;
              cmd_state <= CMD_GET_DLO;
            end if;

          when CMD_GET_DLO =>
            if rx_valid = '1' and rx_ready_i = '1' then
              data_lo   <= rx_data;
              cmd_state <= CMD_DO_WRITE;
            end if;

          when CMD_DO_WRITE =>
            mem_din_i  <= data_hi & data_lo;
            mem_we_i   <= '1';
            cmd_state  <= CMD_DO_WRITE_WAIT;

          when CMD_DO_WRITE_WAIT =>
            cmd_state <= CMD_SEND_ACK;

          when CMD_DO_READ_ISSUE =>
            cmd_state <= CMD_SEND_RHI;

          when CMD_DO_READ_WAIT =>
            cmd_state <= CMD_SEND_RHI;

          when CMD_SEND_ACK =>
            if txf_full = '0' then
              txf_push_i      <= '1';
              txf_push_data_i <= x"06";
              cmd_state       <= CMD_IDLE;
            end if;

          when CMD_SEND_RHI =>
            if txf_full = '0' then
              txf_push_i      <= '1';
              txf_push_data_i <= mem_dout(15 downto 8);
              cmd_state       <= CMD_SEND_RLO;
            end if;

          when CMD_SEND_RLO =>
            if txf_full = '0' then
              txf_push_i      <= '1';
              txf_push_data_i <= mem_dout(7 downto 0);
              cmd_state       <= CMD_IDLE;
            end if;

          when others =>
            cmd_state <= CMD_IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture;