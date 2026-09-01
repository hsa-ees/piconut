--------------------------------------------------------------------------------
-- This file is part of the PicoNut project.
--
-- Copyright (C) 2026 Hermann Zoha <hermann.zoha@ha.de>
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

    sdram_clk: out std_logic;
    sdram_clk_en: out std_logic;
    sdram_cp_sel_n: out std_logic;
    sdram_wr_en_n: out std_logic;
    sdram_col_ac_sel_n: out std_logic;
    sdram_row_ac_sel_n: out std_logic;
    sdram_addr: out std_logic_vector(12 downto 0);
    sdram_ba: out std_logic_vector(1 downto 0);
    sdram_dq: inout std_logic_vector(15 downto 0);
    sdram_dqm: out std_logic_vector(1 downto 0)

  );
end;


architecture RTL of TOP_WRAPPER is

  component m_demo_sdram is 
    port (
      clk                     : in  std_logic;
      reset                   : in  std_logic;
      tx_o                    : out std_logic;
      rx_i                    : in std_logic;
      sdram_clk          : out std_logic;
      sdram_clk_en            : out std_logic;
      sdram_cp_sel_n           : out std_logic;
      sdram_wr_en_n          : out std_logic;
      sdram_col_ac_sel_n  : out std_logic;
      sdram_row_ac_sel_n     : out std_logic;
      sdram_addr      : out std_logic_vector(12 downto 0);
      sdram_ba                  : out std_logic_vector(1 downto 0);
      c_we_o                  : out std_logic;
      sdram_dq_i                    : in  std_logic_vector(15 downto 0);
      sdram_dq_o                    : out std_logic_vector(15 downto 0);
      sdram_dqm                  : out std_logic_vector(1 downto 0)
    );
    end component;


  signal sdram_dq_o, sdram_dq_i : std_logic_vector (15 downto 0);
  signal c_we_o : std_logic;
begin

  i_demo_sdram: m_demo_sdram port map(
      clk => clk,
      reset => reset,
      tx_o  => tx_o,
      rx_i  => rx_i,
      sdram_clk => sdram_clk,
      sdram_clk_en => sdram_clk_en,
      sdram_cp_sel_n => sdram_cp_sel_n,
      sdram_wr_en_n => sdram_wr_en_n,
      sdram_col_ac_sel_n => sdram_col_ac_sel_n,
      sdram_row_ac_sel_n => sdram_row_ac_sel_n,
      sdram_addr => sdram_addr,
      sdram_ba => sdram_ba,
      c_we_o => c_we_o,
      sdram_dq_i => sdram_dq_i,
      sdram_dq_o => sdram_dq_o,
      sdram_dqm => sdram_dqm
  );

  -- if c_we = 1, then we write it into dq_c_i.
  -- but if c_we = 0, then it becomes high-impedance
  sdram_dq <= sdram_dq_o when (c_we_o = '1') else "ZZZZZZZZZZZZZZZZ";
  -- Data is always read whenever ther is a signal on dq_s_io. 
  -- However, if c_we = 0, the data from the SDRAM arrives here
  sdram_dq_i <= sdram_dq;


end RTL;
