-- ============================================================================
-- File: bus_pkg.vhd
--
-- Creator: Kevin Lyon / ChatGPT
-- Date Created: 18 March 2026
--
-- Description:
--   Common bus interface type definitions for the SPRAM client interface.
--
-- Notes:
--   I can't figure out mixed-direction record for entity ports in VHDL,
--   so the interface is split into:
--
--     t_bus_req : client -> Bus_Master
--     t_bus_rsp : Bus_Master -> client
--
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;

package bus_pkg is

  ---------------------------------------------------------------------------
  -- Client request bundle
  ---------------------------------------------------------------------------
  type t_bus_req is record
    req   : std_logic;
    we    : std_logic;
    addr  : std_logic_vector(9 downto 0);
    wdata : std_logic_vector(15 downto 0);
  end record;

  ---------------------------------------------------------------------------
  -- Client response bundle
  ---------------------------------------------------------------------------
  type t_bus_rsp is record
    ack   : std_logic;
    rdata : std_logic_vector(15 downto 0);
  end record;

end package;

package body bus_pkg is
end package body;