----------------------------------------------------------------------------------
-- Testbench: Bus_Interface_TestBench_adc_model
--
-- Purpose:
--   Alternate version of Bus_Interface_tb.vhd that preserves the same UART
--   command stimulus while replacing the custom ADC stimulus process with the
--   project AD7928 behavioral model.
----------------------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use ieee.numeric_std.all;

entity Bus_Interface_TestBench_adc_model is
end Bus_Interface_TestBench_adc_model;

architecture behavior of Bus_Interface_TestBench_adc_model is

  signal SCI_RX   : std_logic;
  signal SCI_TX   : std_logic;
  signal LED_1    : std_logic;
  signal LED_2    : std_logic;
  signal LED_3    : std_logic;
  signal LED_4    : std_logic;
  signal LED_5    : std_logic;
  signal LED_6    : std_logic;
  signal LED_7    : std_logic;
  signal LED_8    : std_logic;
  signal ADC_SCLK : std_logic;
  signal ADC_DIN  : std_logic;
  signal ADC_CSn  : std_logic;
  signal ADC_DOUT : std_logic;

  -- Clock period definitions
  constant clk_period : time := 20 ns;
  constant read_Time  : time := 104100 ns; --for 9,600 Baud

  -- Memory Array
  type Memory is array (255 downto 0) of std_logic_vector(7 downto 0);
  signal RS232_Cmd : Memory;

begin

  -- Remapped to this project's actual top-level while preserving the original
  -- UART stimulus behavior.
  uut : entity work.uart_spram_top
    port map (
      RX       => SCI_RX,
      TX       => SCI_TX,
      SPI_Dout => ADC_DOUT,
      SPI_clk  => ADC_SCLK,
      SPI_Din  => ADC_DIN,
      SPI_CSn  => ADC_CSn,
      PWM_0    => LED_1,
      PWM_1    => LED_2,
      PWM_2    => LED_3,
      PWM_3    => LED_4,
      LED_4    => LED_5,
      LED_5    => LED_6,
      LED_6    => LED_7,
      LED_7    => LED_8
    );

  -- Project AD7928 behavioral model
  u_adc_model : entity work.ad7928_model
    port map (
      cs_n_i => ADC_CSn,
      sclk_i => ADC_SCLK,
      din_i  => ADC_DIN,
      dout_o => ADC_DOUT
    );

  ---- Example Serial Commands
  --- Set Registers
  -- 7E12 0A07 0100 000F 0000 0000 0200 0400 0800 0FFF C2

  ---Read Registers
  -- 7E04 0F10 0100 DF

  ----Define Command Memory
  --Test Write
  RS232_Cmd(0)  <= X"7E"; --Start Deliminator
  RS232_Cmd(1)  <= X"12"; --Pkt Length
  RS232_Cmd(2)  <= X"0A"; --Cmd (Write)
  RS232_Cmd(3)  <= X"07"; --Register Count
  RS232_Cmd(4)  <= X"01"; --Start Address High
  RS232_Cmd(5)  <= X"00"; --Start Address Low
  RS232_Cmd(6)  <= X"00"; --LED Enable High
  RS232_Cmd(7)  <= X"0F"; --LED Enable Low (enable PWM_0..PWM_3)
  RS232_Cmd(8)  <= X"00"; --Blink Period High
  RS232_Cmd(9)  <= X"00"; --Blink Period Low
  RS232_Cmd(10) <= X"00"; --LED On Time High
  RS232_Cmd(11) <= X"00"; --LED On Time Low
  RS232_Cmd(12) <= X"02"; --LED 1 Intensity High
  RS232_Cmd(13) <= X"00"; --LED 1 Intensity Low
  RS232_Cmd(14) <= X"04"; --LED 2 Intensity High
  RS232_Cmd(15) <= X"00"; --LED 2 Intensity Low
  RS232_Cmd(16) <= X"08"; --LED 3 Intensity High
  RS232_Cmd(17) <= X"00"; --LED 3 Intensity Low
  RS232_Cmd(18) <= X"0F"; --LED 4 Intensity High
  RS232_Cmd(19) <= X"FF"; --LED 4 Intensity Low
  RS232_Cmd(20) <= X"C2"; --Check Sum

  --Test Read
  RS232_Cmd(30) <= X"7E"; --Start Deliminator
  RS232_Cmd(31) <= X"04"; --Pkt Length
  RS232_Cmd(32) <= X"0F"; --Cmd (Read)
  RS232_Cmd(33) <= X"10"; --Register Count
  RS232_Cmd(34) <= X"01"; --Start Address High
  RS232_Cmd(35) <= X"00"; --Start Address Low
  RS232_Cmd(36) <= X"DF"; --Check Sum

  -- Stimulus process
  stim_proc : process
  begin
    -- initialize serial ports to idle state
    SCI_RX <= '1';

    wait for clk_period * 100;

    for j in 0 to 20 loop
      SCI_RX <= '0'; --Send Start Bit
      wait for read_Time;
      for i in 0 to 7 loop
        SCI_RX <= RS232_Cmd(0 + j)(i);
        wait for read_Time;
      end loop;
      SCI_RX <= '1'; --Send Stop Bit
      wait for read_Time;
    end loop;

    wait for read_Time * 10;

    for j in 0 to 7 loop
      SCI_RX <= '0'; --Send Start Bit
      wait for read_Time;
      for i in 0 to 7 loop
        SCI_RX <= RS232_Cmd(30 + j)(i);
        wait for read_Time;
      end loop;
      SCI_RX <= '1'; --Send Stop Bit
      wait for read_Time;
    end loop;

    wait; -- will wait forever
  end process;

end;
