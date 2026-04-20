-------------------------------------------------------------------------------
-- Testbench: tb_uart_spram_top
--
-- Purpose:
--   Stimulate PWM_0 through PWM_3 via UART writes to the SPRAM-backed
--   control registers used by pwm_spram_client.
--
--   This testbench uses the real top-level UART path. It writes:
--
--       0x0100  Enable register
--       0x0101  Blink period register
--       0x0102  Blink pulse-width register
--       0x0103  PWM_0 duty-cycle register
--       0x0104  PWM_1 duty-cycle register
--       0x0105  PWM_2 duty-cycle register
--       0x0106  PWM_3 duty-cycle register
--
--   Sequence:
--
--       Write enable register for PWM_0–PWM_3
--       Disable blinking
--       Write distinct duty-cycle registers for PWM_0–PWM_3
--       Wait long enough for the PWM client polling cycle to update outputs
--
--       Modify PWM_0–PWM_3 duty-cycle registers
--       Wait long enough for the updated SPRAM values to be polled in
--
--   Observe:
--
--       PWM_0–PWM_3 change duty cycle accordingly without blink modulation.
--
-- Notes:
--
--   - Only PWM_0–PWM_3 are explicitly exercised here to match the
--     assignment scope, though the DUT exposes PWM_0–PWM_7.
--
--   - Blinking is disabled by writing 0x0000 to the blink period register.
--
--   - Because the PWM client polls SPRAM periodically, post-write waits
--     must be much longer than a few PWM cycles.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_uart_spram_top is
end entity;

architecture sim of tb_uart_spram_top is

  -----------------------------------------------------------------------------
  -- UART timing
  -----------------------------------------------------------------------------
  constant BAUD       : integer := 9600;
  constant BIT_PERIOD : time    := 1 sec / BAUD;

  -----------------------------------------------------------------------------
  -- DUT I/O
  -----------------------------------------------------------------------------
  signal RX    : std_logic := '1';
  signal TX    : std_logic;

  signal PWM_0 : std_logic;
  signal PWM_1 : std_logic;
  signal PWM_2 : std_logic;
  signal PWM_3 : std_logic;
  signal PWM_4 : std_logic;
  signal PWM_5 : std_logic;
  signal PWM_6 : std_logic;
  signal PWM_7 : std_logic;

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
    -- Start bit
    serial_line <= '0';
    wait for BIT_PERIOD;

    -- Data bits (LSB first)
    for i in 0 to 7 loop
      serial_line <= data_byte(i);
      wait for BIT_PERIOD;
    end loop;

    -- Stop bit
    serial_line <= '1';
    wait for BIT_PERIOD;
  end procedure;

  -----------------------------------------------------------------------------
  -- UART write helper: write one 16-bit word to one 16-bit address
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

    v_chk := C_CMD_W xor C_COUNT_1 xor v_addr_hi xor v_addr_lo xor v_data_hi xor v_data_lo;

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
      RX    => RX,
      TX    => TX,
      PWM_0 => PWM_0,
      PWM_1 => PWM_1,
      PWM_2 => PWM_2,
      PWM_3 => PWM_3,
      PWM_4 => PWM_4,
      PWM_5 => PWM_5,
      PWM_6 => PWM_6,
      PWM_7 => PWM_7
    );

  -----------------------------------------------------------------------------
  -- Stimulus
  -----------------------------------------------------------------------------
  p_stimulus : process
  begin

    ---------------------------------------------------------------------------
    -- Allow DUT to come out of reset
    ---------------------------------------------------------------------------
    wait for 2 us;

    ---------------------------------------------------------------------------
    -- Initial register writes
    --
    -- 0x0100 = 0x000F  Enable PWM_0..PWM_3
    -- 0x0101 = 0x0000  Blink disabled
    -- 0x0102 = 0x0000  Pulse width unused when blink disabledf
    -- 0x0103 = 0x1000  PWM_0 duty
    -- 0x0104 = 0x4000  PWM_1 duty
    -- 0x0105 = 0x8000  PWM_2 duty
    -- 0x0106 = 0xF000  PWM_3 duty
    ---------------------------------------------------------------------------
    uart_write_word(RX, x"0100", x"000F");
    uart_write_word(RX, x"0101", x"0000");
    uart_write_word(RX, x"0102", x"0000");
    uart_write_word(RX, x"0103", x"1000");
    uart_write_word(RX, x"0104", x"4000");
    uart_write_word(RX, x"0105", x"8000");
    uart_write_word(RX, x"0106", x"F000");

    ---------------------------------------------------------------------------
    -- Wait long enough for pwm_spram_client to poll the updated SPRAM block
    ---------------------------------------------------------------------------
    wait for 40 ms;

    ---------------------------------------------------------------------------
    -- Mid-run update: reverse brightness ordering
    --
    -- 0x0103 = 0xF000
    -- 0x0104 = 0x8000
    -- 0x0105 = 0x4000
    -- 0x0106 = 0x1000
    ---------------------------------------------------------------------------
    uart_write_word(RX, x"0103", x"F000");
    uart_write_word(RX, x"0104", x"8000");
    uart_write_word(RX, x"0105", x"4000");
    uart_write_word(RX, x"0106", x"1000");

    ---------------------------------------------------------------------------
    -- Wait again for repoll / update
    ---------------------------------------------------------------------------
    wait for 40 ms;

    ---------------------------------------------------------------------------
    -- End simulation
    ---------------------------------------------------------------------------
    wait;

  end process;

end architecture;