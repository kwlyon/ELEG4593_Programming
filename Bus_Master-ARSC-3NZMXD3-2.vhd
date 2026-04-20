library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Bus_Master is
  port (
    clk_sys : in  std_logic;
    reset_i : in  std_logic;  -- active-high synchronous reset
	
	-- =========================
    -- Debug Signals LEDs
    -- =========================
	LEDn : out std_logic_vector(7 downto 0);

    -- =========================
    -- From UART FIFO wrapper
    -- =========================
    rxq_data  : in  std_logic_vector(7 downto 0);
    rxq_empty : in  std_logic;
    rxq_full  : in  std_logic;
    rxq_rd_en : out std_logic;  -- pulse 1 clk to pop

    txq_data_in : out std_logic_vector(7 downto 0);
    txq_wr_en   : out std_logic; -- pulse 1 clk to push
    txq_empty   : in  std_logic;
    txq_full    : in  std_logic;

    -- =========================
    -- SPRAM interface
    -- =========================
    mem_addr : out std_logic_vector(9 downto 0);
    mem_din  : out std_logic_vector(15 downto 0);
    mem_dout : in  std_logic_vector(15 downto 0);
    mem_we   : out std_logic;
    mem_en   : out std_logic
  );
end entity;

architecture rtl of Bus_Master is

  -- Protocol constants
  constant C_CMD_W : std_logic_vector(7 downto 0) := x"57"; -- 'W'
  constant C_CMD_R : std_logic_vector(7 downto 0) := x"52"; -- 'R'
  constant C_ACK   : std_logic_vector(7 downto 0) := x"06"; -- ACK

  type t_state is (
    IDLE,

    -- command + address
    GET_CMD,
    GET_ADD_H,
    GET_ADD_L,

    -- read path
    READ_START,
    READ_WAIT,
    SEND_R_H,
    SEND_R_L,

    -- write path
    GET_W_H,
    GET_W_L,
    WRITE_DO,
    SEND_ACK
  );

  signal state : t_state := IDLE;

  signal cmd    : std_logic_vector(7 downto 0)  := (others => '0');
  signal addr16 : std_logic_vector(15 downto 0) := (others => '0');
  signal wdata  : std_logic_vector(15 downto 0) := (others => '0');
  signal rdata  : std_logic_vector(15 downto 0) := (others => '0');
  
  --LED Debug signals
  	-- LED Debug signals
	signal led_buff : std_logic_vector(7 downto 0) := (others => '0');

  -- FIFO pop/capture helper:
  -- We assert rxq_rd_en for 1 cycle; on the next cycle we capture rxq_data into rx_byte.
  signal pop_pending : std_logic := '0';
  signal rx_byte     : std_logic_vector(7 downto 0) := (others => '0');

begin

  ---------------------------------------------------------------------------
  -- Outputs defaulted combinationally via registered process below
  ---------------------------------------------------------------------------
	  -- Keep LEDs OFF unless explicitly changed later
	  LEDn <= led_buff;
  p_fsm : process(clk_sys)
  begin
    if rising_edge(clk_sys) then

      -- Defaults each cycle (single-cycle strobes)
      rxq_rd_en   <= '0';
      txq_wr_en   <= '0';
      mem_we      <= '0';
      mem_en      <= '0';

      if reset_i = '1' then
        state       <= IDLE;
        cmd         <= (others => '0');
        addr16      <= (others => '0');
        wdata       <= (others => '0');
        rdata       <= (others => '0');
        mem_addr    <= (others => '0');
        mem_din     <= (others => '0');
        txq_data_in <= (others => '0');
        pop_pending <= '0';
        rx_byte     <= (others => '0');

      else
        ---------------------------------------------------------------------
        -- Capture byte after a pop (safe for non-show-ahead FIFOs)
        ---------------------------------------------------------------------
        if pop_pending = '1' then
          rx_byte     <= rxq_data;
		  led_buff <= rxq_data;  --LED DEbug
          pop_pending <= '0';
        end if;

        case state is

          -------------------------------------------------------------------
          -- IDLE: wait until RX FIFO has a byte, then pop it  **It is clear the FIFO is clocking in recieved data when next data is recieved...
          -------------------------------------------------------------------
          when IDLE =>
		  led_buff <= (7 => '1', others => '0'); --LED Reset at Idle

            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              state       <= GET_CMD;
			  led_buff(7) <= '0';
            end if;

          -------------------------------------------------------------------
          -- GET_CMD: validate command
          -------------------------------------------------------------------
          when GET_CMD =>
            if pop_pending = '0' then
              if (rx_byte = C_CMD_W) or (rx_byte = C_CMD_R) then
                cmd   <= rx_byte;
				led_buff(7 downto 0) <= rx_byte;
                state <= GET_ADD_H;
              else
                state <= IDLE; -- bad command, ignore
              end if;
            end if;

          -------------------------------------------------------------------
          -- GET_ADD_H: pop high address byte
          -------------------------------------------------------------------
          when GET_ADD_H =>
            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              state       <= GET_ADD_L;
            end if;

          -------------------------------------------------------------------
          -- GET_ADD_L: latch high byte, pop low byte
          -------------------------------------------------------------------
          when GET_ADD_L =>
            if pop_pending = '0' then
              addr16(15 downto 8) <= rx_byte;

              if rxq_empty = '0' then
                rxq_rd_en   <= '1';
                pop_pending <= '1';

                -- Next state will latch low byte and branch on cmd
                state <= READ_START;  -- used as "ADDR_LOW_CAPTURE + branch"
              end if;
            end if;

          -------------------------------------------------------------------
          -- READ_START: latch low address byte and branch
          -------------------------------------------------------------------
          when READ_START =>
            if pop_pending = '0' then
              addr16(7 downto 0) <= rx_byte;

              -- map protocol address to SPRAM word address (0..1023)
              mem_addr <= addr16(9 downto 0);

              if cmd = C_CMD_R then
                -- start read
                mem_en <= '1';
                mem_we <= '0';
                state  <= READ_WAIT;
              else
                -- command is write: get write data high byte
                state <= GET_W_H;
              end if;
            end if;

          -------------------------------------------------------------------
          -- READ_WAIT: latch SPRAM output (safe 1-cycle latency)
          -------------------------------------------------------------------
          when READ_WAIT =>
            rdata <= mem_dout;
            state <= SEND_R_H;

          -------------------------------------------------------------------
          -- SEND_R_H: push read high byte
          -------------------------------------------------------------------
          when SEND_R_H =>
            if txq_full = '0' then
              txq_data_in <= rdata(15 downto 8);
              txq_wr_en   <= '1';
              state       <= SEND_R_L;
            end if;

          -------------------------------------------------------------------
          -- SEND_R_L: push read low byte, return to idle
          -------------------------------------------------------------------
          when SEND_R_L =>
            if txq_full = '0' then
              txq_data_in <= rdata(7 downto 0);
              txq_wr_en   <= '1';
              state       <= IDLE;
            end if;

          -------------------------------------------------------------------
          -- GET_W_H: pop write data high byte
          -------------------------------------------------------------------
          when GET_W_H =>
            if rxq_empty = '0' then
              rxq_rd_en   <= '1';
              pop_pending <= '1';
              state       <= GET_W_L;
            end if;

          -------------------------------------------------------------------
          -- GET_W_L: latch high byte, pop low byte  However we DO get here immediately...we must be stuck here.
          -------------------------------------------------------------------
          when GET_W_L =>

            if pop_pending = '0' then
              wdata(15 downto 8) <= rx_byte;
			--  led_buff(7 downto 0) <= rx_byte;  --TEST WORk

              if rxq_empty = '0' then  --WE ARE TRAPPED WITH rxq_empty != 0 here until next byte sent..any next byte...why?			  
                rxq_rd_en   <= '1';
                pop_pending <= '1';
                state       <= WRITE_DO;
              end if;
            end if;

          -------------------------------------------------------------------
          -- WRITE_DO: latch low byte and perform SPRAM write   **We never get here 57 00 00 00 80
          -------------------------------------------------------------------
          when WRITE_DO =>
            if pop_pending = '0' then
              wdata(7 downto 0) <= rx_byte;
			--  led_buff(7 downto 0) <= rx_byte;  --TEST WORk

              -- perform write
              mem_addr <= addr16(9 downto 0);
              mem_din  <= wdata;      -- NOTE: wdata low byte updated this cycle
              mem_en   <= '1';
              mem_we   <= '1';

              state <= SEND_ACK;
            end if;

          -------------------------------------------------------------------
          -- SEND_ACK: queue ACK byte then return idle
          -------------------------------------------------------------------
          when SEND_ACK =>
            if txq_full = '0' then
              txq_data_in <= C_ACK;
              txq_wr_en   <= '1';
              state       <= IDLE;
            end if;

          when others =>
            state <= IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture;