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

library ieee;
use ieee.std_logic_1164.all;


entity TOP_WRAPPER is
  port (
    clk: in std_logic;
    reset: in std_logic;
    tx_o: out std_logic;
    rx_i: in std_logic;
    pmod_c: in std_logic_vector(7 downto 0);

    gpio_up: in std_logic;
    gpio_down: in std_logic;
    gpio_left: in std_logic;
    gpio_right: in std_logic;
    gpio_fire: in std_logic;

    vga_red_o: out std_logic_vector(3 downto 0);
    vga_green_o: out std_logic_vector(3 downto 0);
    vga_blue_o: out std_logic_vector(3 downto 0);
    vga_hsync_o: out std_logic;
    vga_vsync_o: out std_logic;

    i2c_scl: inout std_logic;
    i2c_sda: inout std_logic
  );
end;


architecture RTL of TOP_WRAPPER is

  component m_refdesign_demonstrator is
    port ( clk: in std_logic;
           reset: in std_logic;
           tx_o: out std_logic;
           rx_i: in std_logic;

           gpio_up: in std_logic;
           gpio_down: in std_logic;
           gpio_left: in std_logic;
           gpio_right: in std_logic;
           gpio_fire: in std_logic;

           vga_red_o: out std_logic_vector(3 downto 0);
           vga_green_o: out std_logic_vector(3 downto 0);
           vga_blue_o: out std_logic_vector(3 downto 0);
           vga_hsync_o: out std_logic;
           vga_vsync_o: out std_logic;

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
  signal combined_up, combined_down, combined_left, combined_right, combined_fire : std_logic;

begin

  -- BUTTON LOGIC (OR-Mode, Active-Low)
  -- PMOD Mapping: 0: A 1: B 2: TA 3: TB 4: Down 5: Up 6: R 7: L
  combined_up    <= gpio_up    or  not pmod_c(5);
  combined_down  <= gpio_down  or  not pmod_c(4);
  combined_left  <= gpio_left  or  not pmod_c(7);
  combined_right <= gpio_right or  not pmod_c(6);
  -- Fire triggers if the main fire button OR PMOD Button A OR PMOD Button B is pressed
  combined_fire  <= gpio_fire  or  not pmod_c(0) or  not pmod_c(1);

  i_dut: m_refdesign_demonstrator port map (
      clk   => clk, 
      reset => reset,
      tx_o  => tx_o,
      rx_i  => rx_i,

      gpio_up    => combined_up,
      gpio_down  => combined_down,
      gpio_left  => combined_left,
      gpio_right => combined_right,
      gpio_fire  => combined_fire,

      vga_red_o   => vga_red_o,
      vga_green_o => vga_green_o,
      vga_blue_o  => vga_blue_o,
      vga_hsync_o => vga_hsync_o,
      vga_vsync_o => vga_vsync_o,

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

end RTL;


