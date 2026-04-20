-- ============================================================================
-- File: My_Own_FN_FIFO.vhd
--
-- Creator: Kevin Lyon
-- Date Created: 05 March 2026
--
-- Description:
--   "My Own F'n FIFO" - simple parameterized synchronous FIFO.
--
--   Created after vendor FIFO showed unexpected Empty flag behavior that
--   caused state-machine timing issues. This implementation provides fully
--   predictable flag behavior.
--
-- Implementation Notes:
--   Single clock domain FIFO
--   push/pop are 1-clock pulses
--   dout is show-ahead (always displays next unread entry)
--   empty = count = 0
--   full  = count = DEPTH
--   DEPTH must be a power of two
--
-- Typical Use:
--   push <= data_valid
--   pop  <= consumer_ready
--
-- Revision History:
--   2026-03-05  Initial version
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity My_Own_FN_FIFO is
  generic (
    WIDTH : integer := 8;
    DEPTH : integer := 16   -- MUST be power of 2
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;  -- active-high

    push  : in  std_logic;  -- 1-clock pulse
    din   : in  std_logic_vector(WIDTH-1 downto 0);

    pop   : in  std_logic;  -- 1-clock pulse
    dout  : out std_logic_vector(WIDTH-1 downto 0);

    empty : out std_logic;
    full  : out std_logic
  );
end entity;

architecture rtl of My_Own_FN_FIFO is

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

  constant ADDR_W : integer := clog2(DEPTH);  --Just enough bits to count to DEPTH-1

  type ram_t is array (0 to DEPTH-1) of std_logic_vector(WIDTH-1 downto 0);
  signal ram : ram_t := (others => (others => '0'));

  signal wptr  : unsigned(ADDR_W-1 downto 0) := (others => '0');
  signal rptr  : unsigned(ADDR_W-1 downto 0) := (others => '0');
  signal count : unsigned(ADDR_W downto 0)   := (others => '0'); -- 0..DEPTH

  constant COUNT0    : unsigned(count'length-1 downto 0) := (others => '0');
  constant COUNT_MAX : unsigned(count'length-1 downto 0) := to_unsigned(DEPTH, count'length);

  -- internal flag signals (so we can read them)
  signal empty_i : std_logic;
  signal full_i  : std_logic;

  signal do_push : std_logic;
  signal do_pop  : std_logic;

begin

  -- show-ahead output (dout always shows next unread entry)
  dout <= ram(to_integer(rptr));

  empty_i <= '1' when count = COUNT0    else '0';
  full_i  <= '1' when count = COUNT_MAX else '0';

  empty <= empty_i;
  full  <= full_i;

  -- only allow legal operations
  do_push <= push and (not full_i);
  do_pop  <= pop  and (not empty_i);

  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        wptr  <= (others => '0');
        rptr  <= (others => '0');
        count <= (others => '0');
      else
        -- WRITE
        if do_push = '1' then
          ram(to_integer(wptr)) <= din;
          wptr <= wptr + 1;  
        end if;

        -- READ POINTER ADVANCE
        if do_pop = '1' then
          rptr <= rptr + 1;
        end if;

        -- COUNT UPDATE
        if (do_push = '1') and (do_pop = '0') then
          count <= count + 1;
        elsif (do_push = '0') and (do_pop = '1') then
          count <= count - 1;
        else
          null; -- "00" or "11" => count unchanged
        end if;

      end if;
    end if;
  end process;

end architecture;