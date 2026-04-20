-- ==============================================================
-- clock.vhd
--
-- Module for the MachXO3D internal oscillator.
--
-- Author: Kevin Lyon
-- Date Created: 27 February 2026
-- Last Updated: 27 February 2026
--
-- Description:
-- This module instantiates the MachXO3D internal OSCJ primitive
-- and provides a stable system clock output for use elsewhere
-- in the design.
--
-- Reference:  MachXO3D sysCLOCK PLL Usage Guide 
--             FPGA-TN-02070-1.1 
--             February 2024 
-- ============================================================== 

library ieee;
use ieee.std_logic_1164.all;   -- Defines std_logic types

library machxo3d;
use machxo3d.components.all;   -- Gives access to OSCJ primitive

-- ==============================================================
-- Entity Declaration
-- ==============================================================

entity clock is
  generic (
    OSC_FREQ : string := "8.31"   -- Must match an allowed internal oscillator frequency (MHz)
  );
  port (
    clk : out std_logic           -- System clock output
  );
end entity clock;

-- ==============================================================
-- Architecture
-- ==============================================================

architecture rtl of clock is

  -- Internal signal that carries the raw oscillator output
  signal clk_osc : std_logic;

  -- --------------------------------------------------------------
  -- OSCJ Primitive Declaration
  -- --------------------------------------------------------------
  -- OSCJ represents the physical on-chip oscillator hardware.
  -- It is a vendor-specific primitive provided by Lattice.
  -- --------------------------------------------------------------

  component OSCJ
    generic (
      NOM_FREQ : string := "8.31" -- Sets oscillator frequency (MHz)
    );
    port (
      STDBY    : in  std_logic;   -- '1' = standby mode, '0' = oscillator enabled
      OSC      : out std_logic;   -- Oscillator clock output
      SEDSTDBY : out std_logic    -- Standby status (unused here)
    );
  end component;

begin

  -- -----------------------------------------------------------------
  -- Instantiate Internal Oscillator  (I love this word...instantiate)
  -- -----------------------------------------------------------------

  u_osch : OSCJ
    generic map (
      NOM_FREQ => OSC_FREQ        -- Pass desired frequency to hardware
    )
    port map (
      STDBY    => '0',            -- Keep oscillator running
      OSC      => clk_osc,        -- Wire oscillator clock
      SEDSTDBY => open            -- Not quite sure what this is
    );

  -- Output Assignment
  clk <= clk_osc;                 -- Provide oscillator clock to entity port

end architecture rtl;