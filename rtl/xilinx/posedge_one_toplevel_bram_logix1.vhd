--
-- Copyright (c) 2015 Mahmoud Haghighi
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
-- FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
-- DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
-- LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
-- OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--
-- $Id$
--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.MATH_REAL.ALL;

library unisim;
use unisim.vcomponents.all;

use work.f32c_pack.all;


entity glue is
    generic (
	 
	-- Main clock (100 MHz)
	C_clk_freq: integer := 100;
	
	-- ISA: either ARCH_MI32 or ARCH_RV32
	C_arch: integer := ARCH_MI32;
	C_debug: boolean := false;

	-- SRAM parameters
   C_sram_wait_cycles: integer := 3;       -- ISSI, OK do 87.5 MHz
   C_sram_pipelined_read: boolean := false;  -- works only at 81.25 MHz !!!
	
	-- SoC configuration options
	C_mem_size: integer := 32;
	C_icache_size: integer := 2;  -- 0, 2, 4 or 8 KBytes
   C_dcache_size: integer := 2;  -- 0, 2, 4 or 8 KBytes
	C_sio: integer := 1;
	C_spi: integer := 2;
	C_gpio: integer := 32;
	C_simple_in: integer range 0 to 128 := 32;
   C_simple_out: integer range 0 to 128 := 32;
	
	C_vgahdmi: boolean := false; -- enable VGA/HDMI output to vga_ and tmds_
	C_vgahdmi_mem_kb: integer := 10; -- (KB) mem size of framebuffer 
	C_vgahdmi_test_picture: integer := 0; -- 0: disable 1:show test picture in Red and Blue channel

	C_vgatext: boolean := true; -- Xark's feature-rich bitmap+textmode VGA
	C_vgatext_label: string :=  "f32c: Posedge-One MIPS compatible soft-core 100MHz 32KB BRAM";	-- default banner in screen memory
	C_vgatext_mode: integer := 0; -- 0=640x480, 1=640x400, 2=800x600 (you must still provide proper pixel clock [25MHz or 40Mhz])
	C_vgatext_bits: integer := 2; -- bits of VGA color per red, green, blue gun (e.g., 1=8, 2=64 and 4=4096 total colors possible)
	C_vgatext_bram_mem: integer := 8; -- BRAM size 1, 2, 4, 8 or 16 depending on font and screen size/memory
	C_vgatext_palette: boolean := false; -- true for run-time color look-up table, else 16 fixed VGA color palette
	C_vgatext_text: boolean := true; -- enable text generation
	C_vgatext_monochrome: boolean := false;	-- true for 2-color text for whole screen, else additional color attribute byte per character
	C_vgatext_font_height: integer := 16; -- font data height 8 or 16 for 8x8 or 8x16 font
	C_vgatext_char_height: integer := 16; -- font cell height (text lines will be C_visible_height / C_CHAR_HEIGHT rounded down, 19=25 lines on 480p)
	C_vgatext_font_linedouble: boolean := false; -- double font height by doubling each line (e.g., so 8x8 font fills 8x16 cell)
	C_vgatext_font_depth: integer := 7; -- font char bits 7 for 128 characters or 8 for 256 characters
	C_vgatext_text_fifo: boolean := false; -- true to use videofifo for text+color, else BRAM for text+color memory
	C_vgatext_text_fifo_step: integer := (80*2)/4; -- step for the fifo refill and rewind
	C_vgatext_text_fifo_width: integer := 6; -- width of FIFO address space (default=4) len = 2^width * 4 byte
	C_vgatext_bitmap: boolean := false; -- true to enable bitmap generation
	C_vgatext_bitmap_depth: integer := 8;	-- bitmap bits per pixel (1, 2, 4, 8)
	C_vgatext_bitmap_fifo: boolean := false; -- true to use videofifo, else SRAM port for bitmap memory
	C_vgatext_bitmap_fifo_step: integer := 0; -- bitmap step for the fifo refill and rewind (0 unless repeating lines)
	C_vgatext_bitmap_fifo_width: integer := 8; -- bitmap width of FIFO address space len = 2^width * 4 byte
	
	C_ps2: boolean := false;
	C_pcm: boolean := false;
	C_dds: boolean := false;
	C_fmrds: boolean := true;
	C_rds_msg_len: integer := 260; -- bytes of RAM for RDS binary message
   C_fmdds_hz: integer := 250000000; -- Hz clk_fmdds (>2*108 MHz, e.g. 250 MHz, 325 MHz)
   C_rds_clock_multiply: integer := 57; -- multiply and divide from cpu clk 100 MHz
   C_rds_clock_divide: integer := 3125; -- to get 1.824 MHz for RDS logic
	-- warning long compile time on ISE 14.7
	-- C_pids = 2: 1 hour
	-- C_pids = 4: 4 hours
	C_pids: integer := 0;
	C_pid_simulator: std_logic_vector(7 downto 0) := ext("1111", 8);
	C_pid_prescaler: integer := 18;
	C_pid_precision: integer := 1;
	C_pid_pwm_bits: integer := 12
	
    );
    port (
	clk_24MHz: in std_logic;
	
	rs232_dce_txd: out std_logic;
	rs232_dce_rxd: in std_logic;
	
	flash_so: in std_logic;
	flash_cen, flash_sck, flash_si: out std_logic;
	
	spibus_so: in std_logic;
	spibus_cen, spibus_sck, spibus_si: out std_logic;
	
	sram_addr: out std_logic_vector(18 downto 0);
	sram_data: inout std_logic_vector(7 downto 0);
	sram_we: out std_logic;
	sram_oe_n: out std_logic;
	sram_ce_n: out std_logic;

	Audio1: out std_logic; -- fm antenna is here
	
-- Peripherals on LogiX1 Megawing (comment when Megawing is not used)	
	LED: out std_logic_vector(7 downto 0);
	Switch: in std_logic_vector(3 downto 0);
	sw: in std_logic_vector(7 downto 0);
	SevenSegment: out std_logic_vector(7 downto 0); -- 7-segment display
	SevenSegmentEnable: out std_logic_vector(3 downto 0); -- 7-segment display
	HSync,VSync: out std_logic;
	Red: out std_logic_vector(1 downto 0);
	Green: out std_logic_vector(1 downto 0);
	Blue: out std_logic_vector(1 downto 0)

-- Comment following ports when using megawing
---- 12-bit VGA output	
--	HSync,VSync: out std_logic;
--	Red: out std_logic_vector(3 downto 0);
--	Green: out std_logic_vector(3 downto 0);
--	Blue: out std_logic_vector(3 downto 0);	
---- GPIO Pins
--	WING_AH: inout std_logic_vector(7 downto 0);
--	WING_AL: inout std_logic_vector(7 downto 0);
--	WING_BL: inout std_logic_vector(7 downto 0);	
----	WING_BH: inout std_logic_vector(7 downto 0); used for spibus and fm antenna	
---- TMDS Interface for HDMI output
--	TMDS_out_P, TMDS_out_N: out std_logic_vector(2 downto 0);
--	TMDS_out_CLK_P, TMDS_out_CLK_N: out std_logic
  );
 
end glue;

architecture Behavioral of glue is
    signal clk_100MHz, rs232_break: std_logic;
	 signal clk_25MHz, clk_250MHz: std_logic := '0';
    signal obuf_tmds_clock: std_logic;
    signal tmds_out_rgb: std_logic_vector(2 downto 0);
	 
begin
    --  clock synthesizer: Xilinx Spartan-6 specific

    clk25: if C_clk_freq = 100 generate
    clkgen25: entity work.pll_24M_25M_100M_250M
    port map(
      CLK_IN1 => clk_24MHz, CLK_OUT1 => clk_25MHz, CLK_OUT2 => clk_100MHz , CLK_OUT3 => clk_250MHz
    );
    end generate;

    -- reset hard-block: Xilinx Spartan-6 specific
    reset: startup_spartan6
    port map (
	clk => clk_100MHz, gsr => rs232_break, gts => rs232_break,
	keyclearb => '0'
    );

    -- generic BRAM glue
    glue_sram: entity work.glue_bram_sram8
    generic map (
	C_clk_freq => C_clk_freq,
	
	C_arch => C_arch,	
	C_debug => C_debug,
	
	C_sram_wait_cycles => C_sram_wait_cycles,
	C_sram_pipelined_read => C_sram_pipelined_read,
	
	C_mem_size => C_mem_size,
	C_icache_size => C_icache_size,
	C_dcache_size => C_dcache_size,
	C_sio => C_sio,
	C_spi => C_spi,
	C_gpio => C_gpio,
	C_simple_in => C_simple_in,
	C_simple_out => C_simple_out,
	
	C_vgahdmi => C_vgahdmi,
	C_vgahdmi_mem_kb => C_vgahdmi_mem_kb,
	C_vgahdmi_test_picture => C_vgahdmi_test_picture,
	
	C_vgatext => C_vgatext,
	C_vgatext_label => C_vgatext_label,
	C_vgatext_mode => C_vgatext_mode,
	C_vgatext_bits => C_vgatext_bits,
	C_vgatext_bram_mem => C_vgatext_bram_mem,
	C_vgatext_palette => C_vgatext_palette,
	C_vgatext_text => C_vgatext_text,
	C_vgatext_monochrome => C_vgatext_monochrome,
	C_vgatext_font_height => C_vgatext_font_height,
	C_vgatext_char_height => C_vgatext_char_height,
	C_vgatext_font_linedouble => C_vgatext_font_linedouble,
	C_vgatext_font_depth => C_vgatext_font_depth,
	C_vgatext_text_fifo => C_vgatext_text_fifo,
	C_vgatext_text_fifo_step => C_vgatext_text_fifo_step,
	C_vgatext_text_fifo_width => C_vgatext_text_fifo_width,
	C_vgatext_bitmap => C_vgatext_bitmap,
	C_vgatext_bitmap_depth => C_vgatext_bitmap_depth,
	C_vgatext_bitmap_fifo => C_vgatext_bitmap_fifo,
	C_vgatext_bitmap_fifo_step => C_vgatext_bitmap_fifo_step,
	C_vgatext_bitmap_fifo_width => C_vgatext_bitmap_fifo_width,

	C_ps2 => C_ps2,
	C_pcm => C_pcm,
	C_fmrds => C_fmrds,
	C_fmdds_hz => C_fmdds_hz,
	C_rds_msg_len => C_rds_msg_len,
   C_rds_clock_multiply => C_rds_clock_multiply,
   C_rds_clock_divide => C_rds_clock_divide,
	
	C_pids => C_pids,
	C_pid_simulator => C_pid_simulator,
	C_pid_prescaler => C_pid_prescaler, -- set control loop frequency
	C_pid_fp => integer(floor((log2(real(C_clk_freq)*1.0E6))+0.5))-C_pid_prescaler, -- control loop approx freq in 2^n Hz for math, 26-C_pid_prescaler = 8
	C_pid_precision => C_pid_precision, -- fixed point PID precision
	C_pid_pwm_bits => C_pid_pwm_bits -- clock divider bits define PWM output frequency
	
   )
   port map (
	clk => clk_100MHz,
	clk_25MHz => clk_25MHz, -- pixel clock
	clk_250MHz => clk_250MHz,
	clk_fmdds => clk_250MHz,
	clk_cw => '0', -- CW clock (433.92 MHz)	
	sio_txd(0) => rs232_dce_txd, sio_rxd(0) => rs232_dce_rxd, 
	sio_break(0) => rs232_break,	
	spi_sck(0) => flash_sck,spi_sck(1) => spibus_sck,
	spi_ss(0) => flash_cen,spi_ss(1) => spibus_cen,
	spi_mosi(0) => flash_si,spi_mosi(1) => spibus_si, 
	spi_miso(0) => flash_so,spi_miso(1) => spibus_so,
   fm_antenna => Audio1,	
   sram_addr(18 downto 0) => sram_addr,
	sram_addr(19) => open,
   sram_data(7 downto 0) => sram_data,
   sram_we => sram_we,
	vga_vsync => VSync,
	vga_hsync => HSync,

--port mapping for logix1 megawing	connected
	vga_b(7 downto 6) => Blue(1 downto 0),
	vga_b(5 downto 0) => open,
	vga_g(7 downto 6) => Green(1 downto 0),
	vga_g(5 downto 0) => open,
	vga_r(7 downto 6) => Red(1 downto 0),
	vga_r(5 downto 0) => open,
	simple_out(7 downto 0) => LED(7 downto 0),
	simple_out(15 downto 8) => SevenSegment(7 downto 0),
	simple_out(19 downto 16) => SevenSegmentEnable(3 downto 0),
	simple_out(31 downto 20) => open,
	simple_in(3 downto 0) => Switch(3 downto 0), 
	simple_in(15 downto 4) => X"000",
	simple_in(23 downto 16) => sw(7 downto 0), 
	simple_in(31 downto 24) => open,

--port mapping for no addone board 
--  simple_out(7 downto 0) => WING_AL(7 downto 0),
--  simple_out(31 downto 8) => open,
--  simple_in(7 downto 0) => WING_AH(7 downto 0),
--  simple_in(31 downto 8) => open,
--  gpio(7 downto 0) => WING_BL(7 downto 0),
--  gpio(127 downto 8) => open,
--  vga_b(7 downto 4) => Blue(3 downto 0),
--  vga_b(3 downto 0) => open,
--  vga_g(7 downto 4) => Green(3 downto 0),
--  vga_g(3 downto 0) => open,
--  vga_r(7 downto 4) => Red(3 downto 0),
--  vga_r(3 downto 0) => open,

	ps2_clk_in => '0',
	ps2_dat_in => '0'
    );
	 
	sram_oe_n <= '0'; 
	sram_ce_n <= '0';
	
 -- differential output buffering for HDMI clock and video
-- hdmi_output: entity work.hdmi_out
--	port map (
--	  tmds_in_clk => clk_25MHz,
--	  tmds_out_clk_p => tmds_out_clk_p,
--	  tmds_out_clk_n => tmds_out_clk_n,
--	  tmds_in_rgb => tmds_out_rgb,
--	  tmds_out_rgb_p => tmds_out_p,
--	  tmds_out_rgb_n => tmds_out_n
--	);
end Behavioral;
