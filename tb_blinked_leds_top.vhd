-------------------------------------------------------------------------------
-- Testbench: tb_blinked_leds_top
--
-- Updated:
--   - Reduced to PWM_0 through PWM_3 only
--   - Added required SPI and LED_4..LED_7 top-level connections
--   - Removed active blink behavior for now
--   - Restored blink/on-time writes as commented-out lines
--   - Enable written LAST so all PWM channels start together
--
-- NOTE:
--   This testbench currently runs the PWM channels continuously.
--   If blink behavior is wanted again, uncomment the writes to:
--     0x0101 = blink period
--     0x0102 = blink on-time
-------------------------------------------------------------------------------

-- UART PACKETS (CURRENT ACTIVE VALUES)
--
-- PWM_0 (500  = 0x01F4)
--   7E 07 57 01 01 03 01 F4 A5 0D
--
-- PWM_1 (1000 = 0x03E8)
--   7E 07 57 01 01 04 03 E8 B8 0D
--
-- PWM_2 (1500 = 0x05DC)
--   7E 07 57 01 01 05 05 DC 8A 0D
--
-- PWM_3 (2000 = 0x07D0)
--   7E 07 57 01 01 06 07 D0 9D 0D
--
-- Enable PWM_0 through PWM_3 (0x0100 <- 0x000F)
--   7E 07 57 01 01 00 00 0F 59 0D
--
-- OPTIONAL BLINK SETTINGS (CURRENTLY COMMENTED OUT)
--
-- Blink period (0x0101 <- 0x03E8) = 1000 ms
--   7E 07 57 01 01 01 03 E8 BC 0D
--
-- Blink ON time (0x0102 <- 0x01F4) = 500 ms
--   7E 07 57 01 01 02 01 F4 A4 0D
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_blinked_leds_top is
end entity;

architecture sim of tb_blinked_leds_top is

  -----------------------------------------------------------------------------
  -- UART timing
  -----------------------------------------------------------------------------
  constant BAUD       : integer := 9600;
  constant BIT_PERIOD : time    := 1 sec / BAUD;

  -----------------------------------------------------------------------------
  -- DUT I/O
  -----------------------------------------------------------------------------
  signal RX       : std_logic := '1';
  signal TX       : std_logic;

  signal SPI_Dout : std_logic := '0';
  signal SPI_clk  : std_logic;
  signal SPI_Din  : std_logic;
  signal SPI_CSn  : std_logic;

  signal PWM_0    : std_logic;
  signal PWM_1    : std_logic;
  signal PWM_2    : std_logic;
  signal PWM_3    : std_logic;
  signal DSP_G1   : std_logic;
  signal DSP_G2   : std_logic;

  signal LED_4    : std_logic;
  signal LED_5    : std_logic;
  signal LED_6    : std_logic;
  signal LED_7    : std_logic;

  -----------------------------------------------------------------------------
  -- Protocol constants
  -----------------------------------------------------------------------------
  constant C_SOF        : std_logic_vector(7 downto 0) := x"7E";
  constant C_LEN_WRITE1 : std_logic_vector(7 downto 0) := x"07";
  constant C_CMD_W      : std_logic_vector(7 downto 0) := x"57";
  constant C_COUNT_1    : std_logic_vector(7 downto 0) := x"01";
  constant C_CR         : std_logic_vector(7 downto 0) := x"0D";

  -----------------------------------------------------------------------------
  -- UART byte transmit helper
  -----------------------------------------------------------------------------
  procedure uart_send_byte (
    signal serial_line : out std_logic;
    constant data_byte : in  std_logic_vector(7 downto 0)
  ) is
  begin
    serial_line <= '0';
    wait for BIT_PERIOD;

    for i in 0 to 7 loop
      serial_line <= data_byte(i);
      wait for BIT_PERIOD;
    end loop;

    serial_line <= '1';
    wait for BIT_PERIOD;
  end procedure;

  -----------------------------------------------------------------------------
  -- UART write helper
  -----------------------------------------------------------------------------
  procedure uart_write_word (
    signal serial_line : out std_logic;
    constant addr_word : in  std_logic_vector(15 downto 0);
    constant data_word : in  std_logic_vector(15 downto 0)
  ) is
    variable v_addr_hi : std_logic_vector(7 downto 0);
    variable v_addr_lo : std_logic_vector(7 downto 0);
    variable v_data_hi : std_logic_vector(7 downto 0);
    variable v_data_lo : std_logic_vector(7 downto 0);
    variable v_chk     : std_logic_vector(7 downto 0);
  begin
    v_addr_hi := addr_word(15 downto 8);
    v_addr_lo := addr_word(7 downto 0);
    v_data_hi := data_word(15 downto 8);
    v_data_lo := data_word(7 downto 0);

    v_chk := C_CMD_W xor C_COUNT_1 xor v_addr_hi xor v_addr_lo xor
             v_data_hi xor v_data_lo;

    uart_send_byte(serial_line, C_SOF);
    uart_send_byte(serial_line, C_LEN_WRITE1);
    uart_send_byte(serial_line, C_CMD_W);
    uart_send_byte(serial_line, C_COUNT_1);
    uart_send_byte(serial_line, v_addr_hi);
    uart_send_byte(serial_line, v_addr_lo);
    uart_send_byte(serial_line, v_data_hi);
    uart_send_byte(serial_line, v_data_lo);
    uart_send_byte(serial_line, v_chk);
    uart_send_byte(serial_line, C_CR);
  end procedure;

begin

  -----------------------------------------------------------------------------
  -- DUT
  -----------------------------------------------------------------------------
  DUT : entity work.uart_spram_top
    port map (
      RX       => RX,
      TX       => TX,

      SPI_Dout => SPI_Dout,
      SPI_clk  => SPI_clk,
      SPI_Din  => SPI_Din,
      SPI_CSn  => SPI_CSn,

      PWM_0    => PWM_0,
      PWM_1    => PWM_1,
      PWM_2    => PWM_2,
      PWM_3    => PWM_3,
      DSP_G1   => DSP_G1,
      DSP_G2   => DSP_G2,

      LED_4    => LED_4,
      LED_5    => LED_5,
      LED_6    => LED_6,
      LED_7    => LED_7
    );

  -----------------------------------------------------------------------------
  -- Stimulus
  -----------------------------------------------------------------------------
  p_stimulus : process
  begin

    wait for 2 us;

    ---------------------------------------------------------------------------
    -- Write PWM duty cycles first
    ---------------------------------------------------------------------------
    uart_write_word(RX, x"0103", x"01F4");  -- PWM_0 =  500
    uart_write_word(RX, x"0104", x"03E8");  -- PWM_1 = 1000
    uart_write_word(RX, x"0105", x"05DC");  -- PWM_2 = 1500
    uart_write_word(RX, x"0106", x"07D0");  -- PWM_3 = 2000

    ---------------------------------------------------------------------------
    -- Optional blink settings
    -- Uncomment these if you want to restore blink behavior later
    ---------------------------------------------------------------------------
    -- uart_write_word(RX, x"0101", x"03E8");  -- Blink period = 1000 ms
    -- uart_write_word(RX, x"0102", x"01F4");  -- Blink ON time = 500 ms

    ---------------------------------------------------------------------------
    -- Enable all four PWM channels LAST so they start together
    ---------------------------------------------------------------------------
    uart_write_word(RX, x"0100", x"000F");  -- Enable PWM_0 through PWM_3

    ---------------------------------------------------------------------------
    -- Run long enough to observe PWM behavior
    ---------------------------------------------------------------------------
    wait for 50 ms;

    wait;

  end process;

end architecture;
