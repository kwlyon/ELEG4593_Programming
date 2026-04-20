-------------------------------------------------------------------------------
-- Testbench: tb_uart_spram_top
--
-- Purpose:
--   Simple UART stimulus testbench for uart_spram_top.
--
--   After allowing the DUT to exit reset, two UART packets are transmitted
--   on RX to exercise the full system path:
--
--       UART -> FIFO -> packet parser -> Bus_Master -> SPRAM
--
--   Command sequence:
--
--       Write three sequential 16-bit words starting at address 0x0000
--           Data words:
--               0x4269
--               0x0607
--               0xABCD
--
--           Packet:
--               0A 57 00 00 42 69 06 07 AB CD DC 0D
--
--       Read three sequential 16-bit words starting at address 0x0000
--           Packet:
--               05 52 00 00 03 51 0D
--
--   Packet format:
--
--       LEN | PAYLOAD | CHKSUM | CR
--
--       LEN    = number of PAYLOAD bytes + checksum byte
--       CHKSUM = XOR of all PAYLOAD bytes
--       CR     = 0x0D terminator
--
--   Expected TX responses:
--
--       Write command:
--           02 06 06 0D
--
--       Read command:
--           07 42 69 06 07 AB CD 5B 0D
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_uart_spram_top is
end entity;

architecture sim of tb_uart_spram_top is

  -------------------------------------------------------------------------------
  -- UART timing
  -------------------------------------------------------------------------------
  constant BAUD       : integer := 9600;
  constant BIT_PERIOD : time    := 1 sec / BAUD;

  -------------------------------------------------------------------------------
  -- DUT I/O
  -------------------------------------------------------------------------------
  signal RX   : std_logic := '1';  -- UART idle high
  signal TX   : std_logic;
  signal LEDn : std_logic_vector(7 downto 0);

  -------------------------------------------------------------------------------
  -- UART byte transmit helper
  -- Sends one 8N1 serial byte on the RX line, LSB first
  -------------------------------------------------------------------------------
  procedure uart_send_byte (
    signal serial_line : out std_logic;
    constant data_byte : in  std_logic_vector(7 downto 0)
  ) is
  begin
    -- Start bit
    serial_line <= '0';
    wait for BIT_PERIOD;

    -- Data bits
    for i in 0 to 7 loop
      serial_line <= data_byte(i);
      wait for BIT_PERIOD;
    end loop;

    -- Stop bit
    serial_line <= '1';
    wait for BIT_PERIOD;
  end procedure;

begin

  -------------------------------------------------------------------------------
  -- DUT
  -------------------------------------------------------------------------------
  DUT : entity work.uart_spram_top
    port map (
      RX   => RX,
      TX   => TX,
      LEDn => LEDn
    );

  -------------------------------------------------------------------------------
  -- Stimulus
  -------------------------------------------------------------------------------
  p_stimulus : process
  begin

    ---------------------------------------------------------------------------
    -- Allow DUT to come out of reset
    -- With PLL lock + reset_delay this should only need a short wait, but
    -- leave some margin here for simulation startup.
    ---------------------------------------------------------------------------
    wait for 2 us;

    ---------------------------------------------------------------------------
    -- WRITE packet: write three sequential words starting at address 0x0000
    --
    -- Data:
    --   addr 0x0000 <- 0x4269
    --   addr 0x0001 <- 0x0607
    --   addr 0x0002 <- 0xABCD
    --
    -- Packet = 0A 57 00 00 42 69 06 07 AB CD DC 0D
    ---------------------------------------------------------------------------
    uart_send_byte(RX, x"0A");
	uart_send_byte(RX, x"57");
	uart_send_byte(RX, x"00");
	uart_send_byte(RX, x"00");
	uart_send_byte(RX, x"42");
	uart_send_byte(RX, x"69");
	uart_send_byte(RX, x"06");
	uart_send_byte(RX, x"07");
	uart_send_byte(RX, x"AB");
	uart_send_byte(RX, x"CD");
	uart_send_byte(RX, x"1B");
	uart_send_byte(RX, x"0D");

    ---------------------------------------------------------------------------
    -- Wait long enough for write command to complete and ACK to transmit
    ---------------------------------------------------------------------------
    wait for 5 ms;

    ---------------------------------------------------------------------------
    -- READ packet: read 3 sequential words from address 0x0000
    -- Packet = 05 52 00 00 03 51 0D
    ---------------------------------------------------------------------------
    uart_send_byte(RX, x"05");
    uart_send_byte(RX, x"52");
    uart_send_byte(RX, x"00");
    uart_send_byte(RX, x"00");
    uart_send_byte(RX, x"03");
    uart_send_byte(RX, x"51");
    uart_send_byte(RX, x"0D");

    ---------------------------------------------------------------------------
    -- Allow time for returned read packet on TX
    ---------------------------------------------------------------------------
    wait for 20 ms;

    wait;
  end process;

end architecture;