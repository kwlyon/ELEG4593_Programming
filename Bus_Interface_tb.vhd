----------------------------------------------------------------------------------
-- Company:  University of Arkansas (NCREPT)
-- Engineer: Chris Farnell
-- 
-- Create Date:			9Jun2019
-- Last Updated:		25Apr2021
-- Design Name: 		Bus_Interface_TestBench
-- Module Name: 		Bus_Interface_TestBench - Behavioral
-- Project Name: 		Bus Interface Example
-- Target Devices: 		LCMXO3D-9400HC-6BG256C (MachXO3D_BreakoutBrd)
-- Tool versions: 		Lattice Diamond_x64 Build  3.11.3.469.0
--
---- Description: 
-- This Test Bench first sends a write command to update registers used to control the PWM Modules.
-- Next it issues a read command to read from the registers.
-- Total Simulation Time for these commands is approximatley 75 ms.
--
-- The Commands sent and recieved are documented in the comments below:
--
-- The packet below writes 7 16-bit registers starting at register 0x0100; Values listed below. 
-- PWM_Enable     => 0x000F (Enable PWM_0 through PWM_3) 
-- LED_BlinkFreq  => 0x0000 (Blink disabled) 
-- LED_OnTime     => 0x0000 (Unused when blink disabled) 
-- LED1_Intensity => 0x0200 [12.5%] (0200/0FFF) (%)
-- LED2_Intensity => 0x0400 [25%]   (0400/0FFF) (%)
-- LED3_Intensity => 0x0800 [50%]   (0800/0FFF) (%)
-- LED4_Intensity => 0x0FFF [100%]  (0FFF/0FFF) (%)
--
-- Start Delimiter| Pkt Len |Op ID| Register_Cnt |Start Address   |Register Data (16bit x Register_Cnt) | ChkSum
-- 0x7E           | 0x12    |0x0A |0x07          |0x0100          |0x000F000000000200040008000FFF       | 0xC2
-- 0x7E120A070100000F000000000200040008000FFFC2 enables PWM_0 through PWM_3 and sets
-- the four LED duty-cycle registers to the values listed above.
--
-- The Register Read Command Packet is used to read registers internal to the CPLD. 
-- The following example breaks down a read request packet. 
-- The packet below reads 16 16-bit registers starting at register 0x0100. 
--  Start Delimiter| Pkt Len |Op ID| Register_Cnt |Start Address | ChkSum
--  0x7E           | 0x04    |0x0F |0x10          |0x0100        | 0xDF
-- 0x7E040F100100DF 
--
-- The above command results in a write command being sent from the CPLD which contains data from 16 16-bit registers starting at address 0x0100. 
-- An example response from the CPLD is shown below: 
-- 0x7E240A1001000001BFFF6000200040008000FFFF000000000000000000000000000000000000E7 
--  Start Delimiter| Pkt Len |Op ID| Register_Cnt |Start Address |Register Data (16bit x Register_Cnt)                               | ChkSum
--  0x7E           | 0x24    |0x0A |0x10          |0x0100        |0x0001BFFF6000200040008000FFFF000000000000000000000000000000000000 | 0xE7
-- 
----
--
--
-- Revisions:--
---- Revision 2.0a - 
-- Updated for MachXO3D and so it could serve as an example for the "Programming for Power Electronics" Class.
-- Testbed updated to allow for Read and Write Command Verification
--
---- Revision 1.1b - 
-- Minor Updates to documentation and PWMs.
-- Testbed updated to allow for Read and Write Command Verification
--
---- Revision 1.1a - 
-- Updated to use Protocols based on Zigbee Implementation
--
-- Revision 1.0b - 
-- Updated to use UCB instead of Evaluation Board
--
-- Revision 0.01 - 
-- File Created; Basic\Classical Operation Implemented
--
--
-- Additional Comments: 
-- 
--
----------------------------------------------------------------------------------

--
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use ieee.numeric_std.all;

ENTITY Bus_Interface_TestBench IS
END Bus_Interface_TestBench;

ARCHITECTURE behavior OF Bus_Interface_TestBench IS

	SIGNAL SCI_RX :  std_logic;
	SIGNAL SCI_TX :  std_logic;
	SIGNAL LED_1 :  std_logic;
	SIGNAL LED_2 :  std_logic;
	SIGNAL LED_3 :  std_logic;
	SIGNAL LED_4 :  std_logic;
	SIGNAL LED_5 :  std_logic;
	SIGNAL LED_6 :  std_logic;
	SIGNAL LED_7 :  std_logic;
	SIGNAL LED_8 :  std_logic;
	SIGNAL ADC_SCLK :  std_logic;
	SIGNAL ADC_DIN :  std_logic;
	SIGNAL ADC_CSn :  std_logic;
	SIGNAL ADC_DOUT :  std_logic;

	
	
	-- Clock period definitions
	constant clk_period : time := 20 ns;
	--constant read_Time: time :=8680 ns; --for 115,200 Baud
	constant read_Time: time :=104100 ns; --for 9,600 Baud
	
	-- System Clk and Reset(Not Needed Here)
	--signal SYSCLK :  std_logic;
	--signal RESETn :  std_logic;

	-- Memory Array
	type Memory is array (255 downto 0) of STD_LOGIC_VECTOR (7 downto 0);
	signal RS232_Cmd: Memory;		
	
	----ADC Data for SPI
	type Memory_36 is array (101 downto 0) of STD_LOGIC_VECTOR (47 downto 0);
	signal SinSim: Memory_36:= (X"5824612B5DC5",X"589760FB5D83",X"590860D15D3B",X"597960AE5CED",X"59E760925C9B",X"5A54607D5C44",X"5ABE606F5BE8",X"5B2560685B88",X"5B8860685B25",X"5BE8606F5ABE",X"5C44607D5A54",X"5C9B609259E7",X"5CED60AE5979",X"5D3B60D15908",X"5D8360FB5897",X"5DC5612B5824",X"5E02616157B2",X"5E38619E573F",X"5E6861E056CC",X"5E926228565B",X"5EB5627655EA",X"5ED162C8557C",X"5EE6631F550F",X"5EF4637B54A5",X"5EFB63DB543E",X"5EFB643E53DB",X"5EF464A5537B",X"5EE6650F531F",X"5ED1657C52C8",X"5EB565EA5276",X"5E92665B5228",X"5E6866CC51E0",X"5E38673F519E",X"5E0267B25161",X"5DC56824512B",X"5D83689750FB",X"5D3B690850D1",X"5CED697950AE",X"5C9B69E75092",X"5C446A54507D",X"5BE86ABE506F",X"5B886B255068",X"5B256B885068",X"5ABE6BE8506F",X"5A546C44507D",X"59E76C9B5092",X"59796CED50AE",X"59086D3B50D1",X"58976D8350FB",X"58246DC5512B",X"57B26E025161",X"573F6E38519E",X"56CC6E6851E0",X"565B6E925228",X"55EA6EB55276",X"557C6ED152C8",X"550F6EE6531F",X"54A56EF4537B",X"543E6EFB53DB",X"53DB6EFB543E",X"537B6EF454A5",X"531F6EE6550F",X"52C86ED1557C",X"52766EB555EA",X"52286E92565B",X"51E06E6856CC",X"519E6E38573F",X"51616E0257B1",X"512B6DC55824",X"50FB6D835897",X"50D16D3B5908",X"50AE6CED5979",X"50926C9B59E7",X"507D6C445A54",X"506F6BE85ABE",X"50686B885B25",X"50686B255B88",X"506F6ABE5BE8",X"507D6A545C44",X"509269E75C9B",X"50AE69795CED",X"50D169085D3B",X"50FB68975D83",X"512B68245DC5",X"516167B15E02",X"519E673F5E38",X"51E066CC5E68",X"5228665B5E92",X"527665EA5EB5",X"52C8657C5ED1",X"531F650F5EE6",X"537B64A55EF4",X"53DB643E5EFB",X"543E63DB5EFB",X"54A5637B5EF4",X"550F631F5EE6",X"557C62C85ED1",X"55EA62765EB5",X"565B62285E92",X"56CC61E05E68",X"573F619E5E38",X"573F619E5E38");
	
	--ADC Simulator Data
	signal data_ADC : std_logic_vector(15 downto 0):= b"0001000011110000";
	signal Data_ADC_L1: std_logic_vector(127 downto 0):= X"00E010E120E230E340E450E560E670E7";
	signal Data_ADC_L2: std_logic_vector(127 downto 0):= X"00F010F120F230F340F450F560F670F7";
	signal ADC_Count : std_logic_vector(6 downto 0):=b"1111110";
	signal Sin_Cnt1: integer:=0;
	signal t_Sin_Cnt1: std_logic:='0';	




BEGIN

-- Remapped to this project's actual top-level while preserving the original
-- UART stimulus and ADC simulation behavior.
	uut: entity work.uart_spram_top
	PORT MAP(
		RX => SCI_RX,
		TX => SCI_TX,
		SPI_Dout => ADC_DOUT,
		SPI_clk => ADC_SCLK,
		SPI_Din => ADC_DIN,
		SPI_CSn => ADC_CSn,
		PWM_0 => LED_1,
		PWM_1 => LED_2,
		PWM_2 => LED_3,
		PWM_3 => LED_4,
		LED_4 => LED_5,
		LED_5 => LED_6,
		LED_6 => LED_7,
		LED_7 => LED_8
	);



---- Example Serial Commands
--- Set Registers
-- 7E12 0A07 0100 000F 0000 0000 0200 0400 0800 0FFF C2

---Read Registers
-- 7E04 0F10 0100 DF 

	----Define Command Memory
	--Test Write
	RS232_Cmd(0) <= X"7E";			--Start Deliminator
	RS232_Cmd(1) <= X"12";			--Pkt Length
	RS232_Cmd(2) <= X"0A";			--Cmd (Read)
	RS232_Cmd(3) <= X"07";			--Register Count
	RS232_Cmd(4) <= X"01";			--Start Address High
	RS232_Cmd(5) <= X"00";			--Start Address Low
	RS232_Cmd(6) <= X"00";			--LED Enable High
	RS232_Cmd(7) <= X"0F";			--LED Enable Low (enable PWM_0..PWM_3)
	RS232_Cmd(8) <= X"00";			--Blink Period High
	RS232_Cmd(9) <= X"00";			--Blink Period Low
	RS232_Cmd(10) <= X"00";		--LED On Time High
	RS232_Cmd(11) <= X"00";		--LED On Time Low
	RS232_Cmd(12) <= X"02";		--LED 1 Intensity High
	RS232_Cmd(13) <= X"00";		--LED 1 Intensity Low
	RS232_Cmd(14) <= X"04";		--LED 2 Intensity High
	RS232_Cmd(15) <= X"00";		--LED 2 Intensity Low
	RS232_Cmd(16) <= X"08";		--LED 3 Intensity High
	RS232_Cmd(17) <= X"00";		--LED 3 Intensity Low
	RS232_Cmd(18) <= X"0F";		--LED 4 Intensity High
	RS232_Cmd(19) <= X"FF";		--LED 4 Intensity Low
	RS232_Cmd(20) <= X"C2";		--Check Sum
	
	
	
	--Test Read
	RS232_Cmd(30) <= X"7E";			--Start Deliminator
	RS232_Cmd(31) <= X"04";			--Pkt Length
	RS232_Cmd(32) <= X"0F";			--Cmd (Read)
	RS232_Cmd(33) <= X"10";			--Register Count
	RS232_Cmd(34) <= X"01";			--Start Address High
	RS232_Cmd(35) <= X"00";			--Start Address Low
	RS232_Cmd(36) <= X"DF";			--Check Sum
	
	
   -- Stimulus process
   stim_proc: process
   begin		
		-- initialize serial ports to idle state
		SCI_RX <= '1';

		---- hold reset state for 100 ns.
		--RESETn <='0';
		--wait for clk_period*100;
		--RESETn <='1';
		--wait for clk_period*100;
		---- insert stimulus here 
		
		wait for clk_period*100;
		
		for j in 0 to 20 loop
			SCI_RX <= '0';					--Send Start Bit
			wait for read_Time;	
			for i in 0 to 7 loop
				SCI_RX <= RS232_Cmd(0+j)(i);
				wait for read_Time;
			end loop;
			SCI_RX <= '1';					--Send Stop Bit
			wait for read_Time;
		end loop;
		
		wait for read_Time*10;
		
		for j in 0 to 7 loop
			SCI_RX <= '0';					--Send Start Bit
			wait for read_Time;	
			for i in 0 to 7 loop
				SCI_RX <= RS232_Cmd(30+j)(i);
				wait for read_Time;
			end loop;
			SCI_RX <= '1';					--Send Stop Bit
			wait for read_Time;
		end loop;
		
   
      wait; -- will wait forever
	  
   END PROCESS;
   
   
   
   
   
   -- ADC Sim
 ADC_SIM1 :process
	begin
		wait until ADC_SCLK'event and ADC_SCLK = '1';
		if (ADC_CSn = '0') then
			if((ADC_Count >= 32) and (ADC_Count <= 47)) then
				ADC_DOUT <= SinSim(Sin_Cnt1)(32+(CONV_INTEGER(ADC_Count)-32));

			elsif((ADC_Count >= 16) and ( ADC_Count <= 31)) then
				ADC_DOUT <= SinSim(Sin_Cnt1)(16+CONV_INTEGER(ADC_Count)-16);
				
			else
				ADC_DOUT <= Data_ADC_L1(CONV_INTEGER(ADC_Count));
			end if;
			
			if(ADC_Count = 1) then
				if(Sin_Cnt1 < 101) then
					if(t_Sin_Cnt1 = '1') then
						Sin_Cnt1 <= Sin_Cnt1+1;
						t_Sin_Cnt1 <= '0';
					else
						t_Sin_Cnt1 <= '1';
					end if;
				else
					Sin_Cnt1 <= 0;
				end if;
			end if;
			
			ADC_Count <= ADC_Count - 1;
			
		else
			ADC_DOUT <= '1';
			ADC_Count <= ADC_Count;
		end if;
	end process;

   
   
   
   
   
END;
