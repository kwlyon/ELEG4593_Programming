-- ============================================================================
-- File: tb_uart_spram_top_adc.vhd
--
-- Creator: Kevin Lyon / ChatGPT
-- Date Created: 07 April 2026
--
-- Description:
--   Testbench for uart_spram_top using the simulated AD7928 ADC model.
--
--   This testbench:
--     1) Instantiates the full current top-level design
--     2) Connects the SPI pins to ad7928_model
--     3) Waits for ADC sampling to begin
--     4) Sends a UART read request for 8 words starting at SPRAM address 0x0200
--     5) Verifies the returned packet contains all 8 ADC channel samples
--
--   Expected ADC behavior with the provided ad7928_model:
--     - Channels 0 through 7 return x"111" through x"888"
--     - adc_spram_client writes zero-extended samples to SPRAM addresses
--       0x0200 through 0x0207
--     - A UART burst read from 0x0200 should therefore return:
--         0x0111, 0x0222, 0x0333, 0x0444,
--         0x0555, 0x0666, 0x0777, 0x0888
--
-- Notes:
--
--   - UART framing is 9600 baud, 8-N-1.
--   - Packet format:
--         SOF | LEN | PAYLOAD... | CHKSUM
--   - Read request payload:
--         0F | COUNT | ADDR_H | ADDR_L
--   - Read response payload:
--         0A | COUNT | ADDR_H | ADDR_L | DATA_H | DATA_L
--
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_uart_spram_top_adc is
end entity;

architecture sim of tb_uart_spram_top_adc is

  ---------------------------------------------------------------------------
  -- UART timing
  ---------------------------------------------------------------------------
  constant C_BAUD       : integer := 9600;
  constant C_BIT_PERIOD : time    := 1 sec / C_BAUD;

  ---------------------------------------------------------------------------
  -- DUT I/O
  ---------------------------------------------------------------------------
  signal RX       : std_logic := '1';
  signal TX       : std_logic;

  signal SPI_Dout : std_logic := 'Z';
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

  ---------------------------------------------------------------------------
  -- Simple observability
  ---------------------------------------------------------------------------
  signal spi_frame_count : integer := 0;

begin

  ---------------------------------------------------------------------------
  -- DUT
  ---------------------------------------------------------------------------
  u_dut : entity work.uart_spram_top
    generic map (
      OSC_FREQ           => "8.31",
      CLK_FREQ_HZ        => 24930000,
      BAUD               => 9600,
      DATA_BITS          => 8,
      STOP_BITS          => 1,
      PARITY             => 0,
      RESET_DELAY_CYCLES => 16
    )
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

  ---------------------------------------------------------------------------
  -- Simulated ADC model
  ---------------------------------------------------------------------------
  u_adc_model : entity work.ad7928_model
    port map (
      cs_n_i => SPI_CSn,
      sclk_i => SPI_clk,
      din_i  => SPI_Din,
      dout_o => SPI_Dout
    );

  ---------------------------------------------------------------------------
  -- Count ADC SPI frames
  ---------------------------------------------------------------------------
  p_count_spi_frames : process
  begin
    wait until falling_edge(SPI_CSn);
    spi_frame_count <= spi_frame_count + 1;
  end process;

  ---------------------------------------------------------------------------
  -- Main stimulus / checks
  ---------------------------------------------------------------------------
  p_stimulus : process

    -------------------------------------------------------------------------
    -- Send one UART byte, 8-N-1, LSB first
    -------------------------------------------------------------------------
    procedure uart_send_byte(
      signal ser_line : out std_logic;
      constant data_b : in  std_logic_vector(7 downto 0)
    ) is
    begin
      -- start bit
      ser_line <= '0';
      wait for C_BIT_PERIOD;

      -- data bits, LSB first
      for i in 0 to 7 loop
        ser_line <= data_b(i);
        wait for C_BIT_PERIOD;
      end loop;

      -- stop bit
      ser_line <= '1';
      wait for C_BIT_PERIOD;
    end procedure;

    -------------------------------------------------------------------------
    -- Receive and check one UART byte, 8-N-1, LSB first
    -------------------------------------------------------------------------
    procedure uart_expect_byte(
      signal ser_line   : in  std_logic;
      constant expected : in  std_logic_vector(7 downto 0);
      constant name_str : in  string
    ) is
      variable rx_b : std_logic_vector(7 downto 0);
    begin
      -- wait for start bit
      wait until ser_line = '0';

      -- sample in middle of first data bit
      wait for C_BIT_PERIOD + (C_BIT_PERIOD / 2);

      for i in 0 to 7 loop
        rx_b(i) := ser_line;
        if i < 7 then
          wait for C_BIT_PERIOD;
        end if;
      end loop;

      assert rx_b = expected
        report "UART byte mismatch at " & name_str
        severity error;

      -- move through stop bit
      wait for C_BIT_PERIOD;
    end procedure;

  begin
    -------------------------------------------------------------------------
    -- Idle
    -------------------------------------------------------------------------
    RX <= '1';

    -------------------------------------------------------------------------
    -- Let the design come out of reset and gather a full set of channel data.
    --
    -- The adc_spram_client cycles channels 0..7. Because the AD7928 protocol
    -- is pipelined, the first completed frame is dummy, so wait for enough
    -- SPI frames to populate all eight sequential SPRAM locations.
    -------------------------------------------------------------------------
    wait until spi_frame_count >= 10;
    wait for 2 ms;

    -------------------------------------------------------------------------
    -- Sanity check the added LED behavior
    -------------------------------------------------------------------------
    assert LED_4 = '1'
      report "LED_4 was not held high"
      severity error;

    assert LED_5 = '1'
      report "LED_5 was not held high"
      severity error;

    -------------------------------------------------------------------------
    -- Send UART read request for 8 words starting at 0x0200:
    --
    --   SOF   = 7E
    --   LEN   = 04
    --   CMD   = 0F
    --   COUNT = 08
    --   ADDR_H= 02
    --   ADDR_L= 00
    --   CHKSUM= E6
    -------------------------------------------------------------------------
    uart_send_byte(RX, x"7E");
    uart_send_byte(RX, x"04");
    uart_send_byte(RX, x"0F");
    uart_send_byte(RX, x"08");
    uart_send_byte(RX, x"02");
    uart_send_byte(RX, x"00");
    uart_send_byte(RX, x"E6");

    -------------------------------------------------------------------------
    -- Expect UART response:
    --
    -- Payload should be:
    --   0A 08 02 00
    --   01 11 02 22 03 33 04 44 05 55 06 66 07 77 08 88
    --
    -- Full packet:
    --   SOF   = 7E
    --   LEN   = 14
    --   CMD   = 0A
    --   COUNT = 08
    --   ADDR_H= 02
    --   ADDR_L= 00
    --   DATA  = 0111 0222 0333 0444 0555 0666 0777 0888
    --   CHKSUM= 63
    -------------------------------------------------------------------------
    uart_expect_byte(TX, x"7E", "SOF");
    uart_expect_byte(TX, x"14", "LEN");
    uart_expect_byte(TX, x"0A", "CMD");
    uart_expect_byte(TX, x"08", "COUNT");
    uart_expect_byte(TX, x"02", "ADDR_H");
    uart_expect_byte(TX, x"00", "ADDR_L");
    uart_expect_byte(TX, x"01", "DATA0_H");
    uart_expect_byte(TX, x"11", "DATA0_L");
    uart_expect_byte(TX, x"02", "DATA1_H");
    uart_expect_byte(TX, x"22", "DATA1_L");
    uart_expect_byte(TX, x"03", "DATA2_H");
    uart_expect_byte(TX, x"33", "DATA2_L");
    uart_expect_byte(TX, x"04", "DATA3_H");
    uart_expect_byte(TX, x"44", "DATA3_L");
    uart_expect_byte(TX, x"05", "DATA4_H");
    uart_expect_byte(TX, x"55", "DATA4_L");
    uart_expect_byte(TX, x"06", "DATA5_H");
    uart_expect_byte(TX, x"66", "DATA5_L");
    uart_expect_byte(TX, x"07", "DATA6_H");
    uart_expect_byte(TX, x"77", "DATA6_L");
    uart_expect_byte(TX, x"08", "DATA7_H");
    uart_expect_byte(TX, x"88", "DATA7_L");
    uart_expect_byte(TX, x"63", "CHKSUM");

    report "ADC-to-SPRAM-to-UART test completed successfully."
      severity note;

    wait for 5 ms;

    assert false
      report "End of simulation."
      severity failure;
  end process;

end architecture;
