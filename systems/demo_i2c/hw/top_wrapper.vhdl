--------------------------------------------------------------------------------
-- This file is part of the PicoNut project.
--
-- Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
--     Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
--
--
-- Redistribution and use in source and binary forms, with or without modification,
-- are permitted provided that the following conditions are met:
--
-- 1. Redistributions of source code must retain the above copyright notice, this
--    list of conditions and the following disclaimer.
--
-- 2. Redistributions in binary form must reproduce the above copyright notice,
--    this list of conditions and the following disclaimer in the documentation and/or
--    other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
-- WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
-- DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
-- ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
-- LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
-- ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
-- (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
-- SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-- 
--------------------------------------------------------------------------------

--------------------------------------------------------------------------------
--! @brief VHDL wrapper providing tri-state I2C signals handling
--!
--! This module implements the required tri-state logic for the I2C bus.
--! SDA and SCL are bidirectional open-drain signals and therefore must be
--! switchable between driven low and high-impedance ('Z').
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;


entity TOP_WRAPPER is
  port (
    clk: in std_logic;
    reset: in std_logic;
    tx_o: out std_logic;
    rx_i: in std_logic;
    i2c_scl: inout std_logic;
    i2c_sda: inout std_logic;
    led: out std_logic_vector(7 downto 0)
  );
end;


architecture RTL of TOP_WRAPPER is

  component m_demo_i2c is
    port ( clk: in std_logic;
           reset: in std_logic;
           tx_o: out std_logic;
           rx_i: in std_logic;

           scl_i: in std_logic;
           scl_o: out std_logic;
           scl_oe: out std_logic;
           sda_i: in std_logic;
           sda_o: out std_logic;
           sda_oe: out std_logic
         );
  end component;

  signal scl_o, scl_oe, scl_i : std_logic;
  signal sda_o, sda_oe, sda_i : std_logic;

begin

  i_demo_i2c: m_demo_i2c port map (
      clk   => clk, 
      reset => reset,
      tx_o  => tx_o,
      rx_i  => rx_i,

      scl_i => scl_i,
      scl_o => scl_o,
      scl_oe => scl_oe,
      sda_i => sda_i,
      sda_o => sda_o,
      sda_oe => sda_oe
    );

  i2c_scl <= '0' when scl_oe  = '1' else 'Z';
  i2c_sda <= '0' when sda_oe  = '1' else 'Z';

  scl_i <= i2c_scl;
  sda_i <= i2c_sda;

  led(0) <= '1';
  led(1) <= '0';

end RTL;


