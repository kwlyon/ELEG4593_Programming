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
-- PWM_Enable     => 0x0001 (1=Enable; 0=Disable) 
-- LED_BlinkFreq  => 0xBFFF [75%]   (BFFF/FFFF) (%) 
-- LED_OnTime     => 0x6000 [50%]   (6000/BFFF) (%) 
-- LED1_Intensity => 0x2000 [12.5%] (2000/FFFF) (%)
-- LED2_Intensity => 0x4000 [25%]   (4000/FFFF) (%)
-- LED3_Intensity => 0x8000 [50%]   (8000/FFFF) (%)
-- LED4_Intensity => 0xFFFF [100%]  (FFFF/FFFF) (%)
--
-- Start Delimiter| Pkt Len |Op ID| Register_Cnt |Start Address   |Register Data (16bit x Register_Cnt) | ChkSum
-- 0x7E           | 0x12    |0x0A |0x07          |0x0100          |0x0001BFFF6000200040008000FFFF       | 0xF0
-- 0x7E120A0701000001BFFF6000200040008000FFFFF0 results in all LEDs being set to the above parameters.
-- 0x7E120A0701000001BFFF60000000000000000000CE results in all LED Intensities being set to 0%. 
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

	COMPONENT Bus_Interface_Top
	PORT(
		SCI_RX : IN std_logic;          
		SCI_TX : INOUT std_logic;
		LED_1 : OUT std_logic;
		LED_2 : OUT std_logic;
		LED_3 : OUT std_logic;
		LED_4 : OUT std_logic;
		LED_5 : OUT std_logic;
		LED_6 : OUT std_logic;
		LED_7 : OUT std_logic;
		LED_8 : OUT std_logic;
		PWM_Test_Out : OUT std_logic;
		ADC_SCLK : INOUT  std_logic;
		ADC_DIN : INOUT  std_logic;
		ADC_CSn : OUT  std_logic;
		ADC_DOUT : IN  std_logic;
		DSP_G1 : out STD_LOGIC;		
		DSP_G2 : out STD_LOGIC			
		);
	END COMPONENT;

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
	SIGNAL PWM_Test_Out :  std_logic;
	SIGNAL ADC_SCLK :  std_logic;
	SIGNAL ADC_DIN :  std_logic;
	SIGNAL ADC_CSn :  std_logic;
	SIGNAL ADC_DOUT :  std_logic;
	SIGNAL DSP_G1 :  std_logic;
	SIGNAL DSP_G2 :  std_logic;

	
	
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

-- Please check and add your generic clause manually
	uut: Bus_Interface_Top PORT MAP(
		SCI_RX => SCI_RX,
		SCI_TX => SCI_TX,
		LED_1 => LED_1,
		LED_2 => LED_2,
		LED_3 => LED_3,
		LED_4 => LED_4,
		LED_5 => LED_5,
		LED_6 => LED_6,
		LED_7 => LED_7,
		LED_8 => LED_8,
		PWM_Test_Out => PWM_Test_Out,
		ADC_SCLK => ADC_SCLK,
		ADC_DIN => ADC_DIN,
		ADC_CSn => ADC_CSn,
		ADC_DOUT => ADC_DOUT,
		DSP_G1 => DSP_G1,	
		DSP_G2 => DSP_G2
	);



---- Example Serial Commands
--- Set Registers
-- 7E12 0A07 0100 0001 FFFF 0000 2000 4000 8000 FFFF 10

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
	RS232_Cmd(7) <= X"01";			--LED Enable Low
	RS232_Cmd(8) <= X"FF";			--Blink Period High
	RS232_Cmd(9) <= X"FF";			--Blink Period Low
	RS232_Cmd(10) <= X"00";		--LED On Time High
	RS232_Cmd(11) <= X"00";		--LED On Time Low
	RS232_Cmd(12) <= X"20";		--LED 1 Intensity High
	RS232_Cmd(13) <= X"00";		--LED 1 Intensity Low
	RS232_Cmd(14) <= X"40";		--LED 2 Intensity High
	RS232_Cmd(15) <= X"00";		--LED 2 Intensity Low
	RS232_Cmd(16) <= X"80";		--LED 3 Intensity High
	RS232_Cmd(17) <= X"00";		--LED 3 Intensity Low
	RS232_Cmd(18) <= X"FF";		--LED 4 Intensity High
	RS232_Cmd(19) <= X"FF";		--LED 4 Intensity Low
	RS232_Cmd(20) <= X"10";		--Check Sum
	
	
	
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
