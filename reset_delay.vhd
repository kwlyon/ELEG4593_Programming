-- ============================================================================
-- File: reset_delay.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 16 March 2026
--
-- Description:
--   Simple synchronous reset delay.
--
--   When rst_in = '1', reset is asserted immediately and the counter reloads.
--   When rst_in = '0', the module counts down for DELAY_CYCLES clock cycles.
--   Reset is released once the counter reaches zero.
--
--   Typical use:
--     Hold logic in reset for a short time after PLL lock.
--
--       rst_in  <= not pll_lock;
--       rst_out -> global reset
--
--   DELAY_CYCLES specifies how many clock cycles reset remains asserted
--   after rst_in is released.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity reset_delay is
  generic (
    DELAY_CYCLES : integer := 16
  );
  port (
    clk     : in  std_logic;
    rst_in  : in  std_logic;  -- active-high immediate reset request
    rst_out : out std_logic   -- active-high delayed reset output
  );
end entity;

architecture rtl of reset_delay is

  ---------------------------------------------------------------------------
  -- Compile-time helper function
  -- Determines the number of bits required to represent DELAY_CYCLES
  ---------------------------------------------------------------------------
  function clog2(n : integer) return integer is
    variable r : integer := 0;
    variable v : integer := n - 1;
  begin
    while v > 0 loop
      v := v / 2;
      r := r + 1;
    end loop;
    return r;
  end function;

  ---------------------------------------------------------------------------
  -- Counter width automatically sized
  ---------------------------------------------------------------------------
  constant CNT_W : integer := clog2(DELAY_CYCLES + 1);

  ---------------------------------------------------------------------------
  -- Countdown timer
  ---------------------------------------------------------------------------
  signal counter   : unsigned(CNT_W-1 downto 0) := (others => '0');
  signal rst_r     : std_logic := '1';

begin

  rst_out <= rst_r;

  ---------------------------------------------------------------------------
  -- Reset delay process
  ---------------------------------------------------------------------------
  process(clk)
  begin
    if rising_edge(clk) then

      -- Immediate reset request
      if rst_in = '1' then

        counter <= to_unsigned(DELAY_CYCLES, CNT_W);
        rst_r   <= '1';

      else

        -- Countdown in progress
        if counter /= 0 then
          counter <= counter - 1;
          rst_r   <= '1';

        -- Countdown complete
        else
          rst_r <= '0';
        end if;

      end if;

    end if;
  end process;

end architecture;