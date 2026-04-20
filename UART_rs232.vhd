-- ============================================================================
--  Module      : uart_rs232
--  Author      : Kevin Lyon
--  Description : UART (RS-232 style) transmitter and
--                receiver with configurable baud rate,
--                data width, stop bits, and parity.
--
--  Features:
--    - Parameterizable baud rate via CLK_FREQ_HZ and BAUD generics
--    - Configurable DATA_BITS (typically 5-8)
--    - Configurable STOP_BITS (1 or 2)
--    - Optional parity:
--          0 = None
--          1 = Even
--          2 = Odd
--    - Independent TX and RX state machines
--    - Single-cycle handshake interface
--    - Framing, parity, and overrun error detection
--
--  Clocking:
--    - Single clock domain
--    - Synchronous active-high reset
--    - Baud timing derived from integer clock division
--
--  TX Interface:
--    tx_data   : Data to transmit
--    tx_valid  : Assert to request transmission
--    tx_ready  : High when transmitter is idle
--
--  RX Interface:
--    rx_data        : Received data word
--    rx_valid       : Asserted when data is available
--    rx_ready       : Assert to acknowledge data
--    rx_framing_err : Stop bit error detected
--    rx_parity_err  : Parity error detected
--    rx_overrun     : New data arrived before previous was read
--
--  Revision History:
--    2-27-2026  - Initial version
--    2-28-2026  - Dropped shift register primitive in favor of indexed
--                 register for TX/RX
--    4-14-2026  - Added 2-flop synchronizer on asynchronous RX input.
--                 - Reworked RX timing to use 16x oversampling.
--                 - Intended to reduce intermittent UART packet parse
--                   failures caused by sampling-position sensitivity.
--
--    **I need to add RX and TX buffer registers**
--    **But they are not needed for this homework**
--    **RX amd TX FIFO added in wrapper**
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_rs232 is
  generic (
    CLK_FREQ_HZ : integer := 24930000;
    BAUD        : integer := 9600;

    DATA_BITS   : integer := 8;
    STOP_BITS   : integer := 1;

    -- PARITY: 0=None, 1=Even, 2=Odd
    PARITY      : integer := 0
  );
  port (
    clk   : in  std_logic;
    reset : in  std_logic; -- active-high synchronous reset

    -- Serial pins
    rx    : in  std_logic;
    tx    : out std_logic;

    -- Receive interface
    rx_data        : out std_logic_vector(7 downto 0);
    rx_valid       : out std_logic;
    rx_ready       : in  std_logic;
    rx_framing_err : out std_logic;
    rx_parity_err  : out std_logic;
    rx_overrun     : out std_logic;

    -- Transmit interface
    tx_data  : in  std_logic_vector(7 downto 0);
    tx_valid : in  std_logic;
    tx_ready : out std_logic
  );
end entity;

architecture behavioral of uart_rs232 is

  constant BAUD_DIV : integer := (CLK_FREQ_HZ / BAUD);
  constant HALF_DIV : integer := (CLK_FREQ_HZ / BAUD) / 2;
  constant RX_OVERSAMPLE : integer := 16;
  constant RX_TICK_DIV   : integer := CLK_FREQ_HZ / (BAUD * RX_OVERSAMPLE);
  constant RX_HALF_BIT_TICKS : integer := RX_OVERSAMPLE / 2;

  function parity_xor(d : std_logic_vector(7 downto 0); nbits : integer) return std_logic is
    variable p : std_logic := '0';
  begin
    for i in 0 to nbits-1 loop
      p := p xor d(i);
    end loop;
    return p;
  end function;

  -- =========================
  -- TX
  -- =========================
  type t_tx_state is (
		TX_IDLE, 
		TX_START, 
		TX_SEND, 
		TX_PAR, 
		TX_STOP
	);
  signal tx_state     : t_tx_state := TX_IDLE;
  signal tx_bit_timer : integer range 0 to BAUD_DIV := 0;
  signal tx_bit_idx   : integer range 0 to 7 := 0;

  signal tx_shift     : std_logic_vector(7 downto 0) := (others => '0'); 
  signal tx_par_bit   : std_logic := '0';
  signal tx_stop_cnt  : integer range 0 to 2 := 0;

  signal tx_line      : std_logic := '1';

  -- =========================
  -- RX
  -- =========================
  type t_rx_state is (
		RX_IDLE, 
		RX_START, 
		RX_RECV, 
		RX_PAR, 
		RX_STOP
	);
  signal rx_state     : t_rx_state := RX_IDLE;
  signal rx_bit_timer : integer range 0 to RX_OVERSAMPLE := 0;
  signal rx_bit_idx   : integer range 0 to 7 := 0;
  signal rx_tick_cnt  : integer range 0 to RX_TICK_DIV := 0;

  signal rx_shift     : std_logic_vector(7 downto 0) := (others => '0');
  signal rx_par_bit   : std_logic := '0';
  signal rx_stop_cnt  : integer range 0 to 2 := 0;

  signal rx_meta      : std_logic := '1';
  signal rx_sync      : std_logic := '1';
  signal rx_sync_prev : std_logic := '1';

  signal rx_valid_r       : std_logic := '0';
  signal rx_framing_err_r : std_logic := '0';
  signal rx_parity_err_r  : std_logic := '0';
  signal rx_overrun_r     : std_logic := '0';

begin
  tx <= tx_line;

  rx_data        <= rx_shift;
  rx_valid       <= rx_valid_r;
  rx_framing_err <= rx_framing_err_r;
  rx_parity_err  <= rx_parity_err_r;
  rx_overrun     <= rx_overrun_r;

  tx_ready <= '1' when tx_state = TX_IDLE else '0';  --Signal ready to recieve when idle

  -- ==========================================================================
  -- TX process
  -- ==========================================================================
  p_tx : process(clk)
  begin
    if rising_edge(clk) then
      if reset = '1' then            --Let's deal with reset first
        tx_state     <= TX_IDLE;
        tx_bit_timer <= 0;
        tx_bit_idx   <= 0;
        tx_shift     <= (others => '0');
        tx_par_bit   <= '0';
        tx_stop_cnt  <= 0;
        tx_line      <= '1';
      else
        case tx_state is              --Now the meat of our TX state machine
          when TX_IDLE =>
            tx_line <= '1';
            tx_bit_timer <= 0;
            tx_bit_idx   <= 0;
            tx_stop_cnt  <= 0;

            if tx_valid = '1' then   --Data on tx_data is ready to go
              tx_shift <= tx_data;   --Load the outgoing shift register

              if PARITY = 0 then
                tx_par_bit <= '0';
              elsif PARITY = 1 then -- even
                tx_par_bit <= parity_xor(tx_data, DATA_BITS);  --Calculate parity bit
              else -- odd
                tx_par_bit <= not parity_xor(tx_data, DATA_BITS);  --Calculate negated parity bit
              end if;

              tx_state <= TX_START;    --Locked and Loaded...let's light this candle
              tx_line  <= '0';         --Drop TX to signal start condition
              tx_bit_timer <= BAUD_DIV - 1;
            end if;

          when TX_START =>
            if tx_bit_timer = 0 then			--Start bit complete
              tx_state <= TX_SEND;      		--Next advance state machine to send data
              tx_line  <= tx_shift(0); 			--Present LSB to TX 
              tx_bit_idx <= 0;           		--Reset our data bit counter index
              tx_bit_timer <= BAUD_DIV - 1;  	--Reset the timer for first bit
            else
              tx_bit_timer <= tx_bit_timer - 1;  --Count down to 0
            end if;

          when TX_SEND =>
            if tx_bit_timer = 0 then
              if tx_bit_idx = DATA_BITS - 1 then   --Time to send the stop bit
                if PARITY = 0 then        			--For no Parity 
                  tx_state <= TX_STOP;     			--Next advance state machine to send stop bits
                  tx_line  <= '1';         		--Raise TX line back up...I always forget this
                  tx_stop_cnt  <= STOP_BITS;
                  tx_bit_timer <= BAUD_DIV - 1;  	--Reset the timer for stop bit
                else
                  tx_state <= TX_PAR;				--If Even or Odd Par
                  tx_line  <= tx_par_bit;			--Present Bit on TX
                  tx_bit_timer <= BAUD_DIV - 1;		--Reset the bit timer
                end if;
              else
                tx_bit_idx <= tx_bit_idx + 1;				--If we have more data bits left to send...
                tx_line    <= tx_shift(tx_bit_idx + 1);	--Send next byte **Note we are not shifting 
															--anymore as I want to preserve this data for 
															--Parity Calculation so "shift" is now just an 
															--indexted reg.  I should rename it...
				tx_bit_timer <= BAUD_DIV - 1;             	--Reset the bit timer
              end if;
            else
              tx_bit_timer <= tx_bit_timer - 1;  	--Count down to 0
            end if;

          when TX_PAR =>                   			--Let's send that parity bit
            if tx_bit_timer = 0 then
              tx_state <= TX_STOP;					--Next clock will advance state machine to stop bit transmission
              tx_line  <= '1';
              tx_stop_cnt  <= STOP_BITS;			--Let's go ahead and set the number of stop bits
              tx_bit_timer <= BAUD_DIV - 1;			--Reset the bit timer
            else
              tx_bit_timer <= tx_bit_timer - 1;		--Count down to 0
            end if;

          when TX_STOP =>							--Send stop bits
            if tx_bit_timer = 0 then
              if tx_stop_cnt <= 1 then
                tx_state <= TX_IDLE;               --Back home we go...
                tx_line  <= '1';
              else
                tx_stop_cnt  <= tx_stop_cnt - 1;
                tx_line      <= '1';
                tx_bit_timer <= BAUD_DIV - 1;
              end if;
            else
              tx_bit_timer <= tx_bit_timer - 1;
            end if;
        end case;
      end if;
    end if;
  end process;

  -- ==========================================================================
  -- RX process
  -- ==========================================================================
  p_rx : process(clk)                       	--Entirely parallel recieve circuit
    variable pcalc : std_logic;					--Variable to accumulate parity
    variable rx_tick : std_logic;
  begin
    if rising_edge(clk) then
      rx_tick := '0';

      if reset = '1' then                      --Again let's deal with reset first
        rx_state          <= RX_IDLE;
        rx_bit_timer      <= 0;
        rx_bit_idx        <= 0;					--We need an idex anyways so
        rx_tick_cnt       <= 0;
        rx_shift          <= (others => '0');  --no longer using the shift primative
        rx_par_bit        <= '0';
        rx_stop_cnt       <= 0;
        rx_meta           <= '1';
        rx_sync           <= '1';
        rx_sync_prev      <= '1';

        rx_valid_r        <= '0';
        rx_framing_err_r  <= '0';
        rx_parity_err_r   <= '0';
        rx_overrun_r      <= '0';

      else
        rx_meta      <= rx;
        rx_sync_prev <= rx_sync;
        rx_sync      <= rx_meta;

        if rx_tick_cnt = RX_TICK_DIV - 1 then
          rx_tick_cnt <= 0;
          rx_tick := '1';
        else
          rx_tick_cnt <= rx_tick_cnt + 1;
        end if;

        -- hold valid until consumed
        if rx_valid_r = '1' and rx_ready = '1' then
          rx_valid_r <= '0';
          rx_framing_err_r <= '0';
          rx_parity_err_r  <= '0';
        end if;

        case rx_state is
          when RX_IDLE =>
            rx_bit_timer <= 0;
            rx_bit_idx   <= 0;
            rx_stop_cnt  <= 0;

            if (rx_sync_prev = '1') and (rx_sync = '0') then
              rx_state     <= RX_START;
              rx_bit_timer <= RX_HALF_BIT_TICKS;
            end if;

          when RX_START =>
            if rx_tick = '1' then
              if rx_bit_timer = 0 then
                if rx_sync = '0' then
                  rx_state     <= RX_RECV;		--Sucessful start condition recieved
                  rx_bit_idx   <= 0;
                  rx_bit_timer <= RX_OVERSAMPLE - 1;
                else
                  rx_state <= RX_IDLE;            --Start contidtion not satisfied
                end if;						 	 --remain in IDLE
              else
                rx_bit_timer <= rx_bit_timer - 1;
              end if;
            end if;

          when RX_RECV =>
            if rx_tick = '1' then
              if rx_bit_timer = 0 then
                rx_shift(rx_bit_idx) <= rx_sync;			--Read and place that bit

                if rx_bit_idx = DATA_BITS - 1 then	--If we read the last bit...
                  if PARITY = 0 then					--Again no parity first
                    rx_state     <= RX_STOP;
                    rx_stop_cnt  <= STOP_BITS;
                    rx_bit_timer <= RX_OVERSAMPLE - 1;
                  else
                    rx_state     <= RX_PAR;			--Otherwise on to read parity bit
                    rx_bit_timer <= RX_OVERSAMPLE - 1;
                  end if;
                else
                  rx_bit_idx   <= rx_bit_idx + 1;		--Increment bit index
                  rx_bit_timer <= RX_OVERSAMPLE - 1;
                end if;
              else
                rx_bit_timer <= rx_bit_timer - 1;
              end if;
            end if;

          when RX_PAR =>
            if rx_tick = '1' then
              if rx_bit_timer = 0 then
                rx_par_bit <= rx_sync;						--read in parity bit

                pcalc := parity_xor(rx_shift, DATA_BITS);	--calculate parity recieved
                if PARITY = 1 then
                  if rx_sync /= pcalc then
                    rx_parity_err_r <= '1';		--Assert Parity Fail
                  end if;
                else
                  if rx_sync /= (not pcalc) then
                    rx_parity_err_r <= '1';		--Assert Parity Fail
                  end if;
                end if;

                rx_state     <= RX_STOP;			--Check for the Stop Bit
                rx_stop_cnt  <= STOP_BITS;
                rx_bit_timer <= RX_OVERSAMPLE - 1;
              else
                rx_bit_timer <= rx_bit_timer - 1;
              end if;
            end if;

          when RX_STOP =>						--Send that stop bit
            if rx_tick = '1' then
              if rx_bit_timer = 0 then
                if rx_sync /= '1' then
                  rx_framing_err_r <= '1';
                end if;

                if rx_stop_cnt <= 1 then
                  if rx_valid_r = '0' then
                    rx_valid_r <= '1';
                  else
                    rx_overrun_r <= '1';
                  end if;

                  rx_state <= RX_IDLE;				--Send us back home...
                else
                  rx_stop_cnt  <= rx_stop_cnt - 1;
                  rx_bit_timer <= RX_OVERSAMPLE - 1;
                end if;
              else
                rx_bit_timer <= rx_bit_timer - 1;
              end if;
            end if;
        end case;
      end if;
    end if;
  end process;

end architecture;
