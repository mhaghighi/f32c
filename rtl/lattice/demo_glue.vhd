--
-- Copyright 2011 University of Zagreb.
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
-- THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
-- FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
-- DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
-- LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
-- OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--

-- $Id: glue.vhd 116 2011-03-28 12:43:12Z marko $

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity glue is
    generic (
	-- Main clock: 50, 62, 75, 81, 87, 100, 112, 125, 137, 150 MHz
	C_clk_freq: integer := 81;

	-- ISA options
	C_big_endian: boolean := false;
	C_mult_enable: boolean := true;
	C_branch_likely: boolean := true;
	C_sign_extend: boolean := true;
	C_PC_mask: std_logic_vector(31 downto 0) := x"800fffff";

	-- COP0 options
	C_cop0_count: boolean := true;
	C_cop0_config: boolean := true;

	-- CPU core configuration options
	C_branch_prediction: boolean := true;
	C_result_forwarding: boolean := true;
	C_load_aligner: boolean := true;
	C_register_technology: string := "lattice";

	-- These may negatively influence timing closure:
	C_movn_movz: boolean := false; -- true: +16 LUT4, -DMIPS, incomplete
	C_fast_ID: boolean := true; -- false: +7 LUT4, -Fmax

	-- debugging options
	C_debug: boolean := false; -- true: +883 LUT4, -Fmax
	C_no_cpu: boolean := true;

	-- SoC configuration options
	C_mem_size: string := "16k";
	C_sram: boolean := true;
	C_sram_wait_cycles: std_logic_vector := x"5"; -- ISSI, OK do 87.5 MHz
	C_sio: boolean := true;
	C_gpio: boolean := true;
	C_flash: boolean := true;
	C_sdcard: boolean := true;
	C_pcmdac: boolean := true;
	C_ddsfm: boolean := true
    );
    port (
	clk_25m: in std_logic;
	rs232_tx: out std_logic;
	rs232_rx: in std_logic;
	flash_so: in std_logic;
	flash_cen, flash_sck, flash_si: out std_logic;
	sdcard_so: in std_logic;
	sdcard_cen, sdcard_sck, sdcard_si: out std_logic;
	p_ring: out std_logic;
	p_tip: out std_logic_vector(3 downto 0);
	led: out std_logic_vector(7 downto 0);
	btn_left, btn_right, btn_up, btn_down, btn_center: in std_logic;
	sw: in std_logic_vector(3 downto 0);
	j1: out std_logic_vector(23 downto 20);
	j2: out std_logic_vector(5 downto 2);
	sram_a: out std_logic_vector(18 downto 0);
	sram_d: inout std_logic_vector(15 downto 0);
	sram_wel, sram_lbl, sram_ubl: out std_logic
    );
end glue;

architecture Behavioral of glue is
    signal clk: std_logic;
    signal imem_addr: std_logic_vector(31 downto 2);
    signal imem_data_read: std_logic_vector(31 downto 0);
    signal imem_addr_strobe, imem_data_ready: std_logic;
    signal dmem_addr: std_logic_vector(31 downto 2);
    signal dmem_addr_strobe, dmem_write: std_logic;
    signal dmem_bram_enable, dmem_data_ready: std_logic;
    signal dmem_byte_sel: std_logic_vector(3 downto 0);
    signal dmem_to_cpu, imem_to_cpu, cpu_to_dmem: std_logic_vector(31 downto 0);
    signal io_to_cpu: std_logic_vector(31 downto 0);
    signal final_to_cpu_i, final_to_cpu_d: std_logic_vector(31 downto 0);

    -- SRAM
    signal sram_data_strobe, sram_data_ready: std_logic;
    signal sram_instr_strobe, sram_instr_ready: std_logic;
    signal from_sram: std_logic_vector(31 downto 0);

    -- I/O
    signal from_sio: std_logic_vector(31 downto 0);
    signal sio_txd, sio_ce: std_logic;
    signal R_flash_cen, R_flash_sck, R_flash_si: std_logic;
    signal R_sdcard_cen, R_sdcard_sck, R_sdcard_si: std_logic;
    signal R_led: std_logic_vector(7 downto 0);
    signal R_sw: std_logic_vector(3 downto 0);
    signal R_btns: std_logic_vector(4 downto 0);
    signal R_dac_in_l, R_dac_in_r: std_logic_vector(15 downto 2);
    signal R_dac_acc_l, R_dac_acc_r: std_logic_vector(16 downto 2);

    -- debugging only
    signal trace_addr: std_logic_vector(5 downto 0);
    signal trace_data: std_logic_vector(31 downto 0);
    signal debug_txd: std_logic;
    signal res, intr: std_logic;

    -- FM TX DDS
    signal clk_dds, dds_out: std_logic;
    signal R_dds_cnt, R_dds_div, R_dds_div1: std_logic_vector(21 downto 0);

    -- Video framebuffer
    signal video_dac: std_logic_vector(3 downto 0);
    signal fb_addr_strobe, fb_data_ready: std_logic;
    signal fb_addr: std_logic_vector(19 downto 2);

begin

    -- clock synthesizer
    clkgen: entity work.clkgen
    generic map (
	C_clk_freq => C_clk_freq,
	C_debug => C_debug
    )
    port map (
	clk_25m => clk_25m, clk => clk, clk_325m => clk_dds,
	sel => sw(2), key => btn_down, res => '0'
    );
    res <= btn_up and sw(0) when C_debug else '0';
    intr <= btn_center and sw(0) when C_debug else '0';

    -- f32c core
    G_CPU:
    if not C_no_cpu generate
    pipeline: entity work.pipeline
    generic map (
	C_clk_freq => C_clk_freq,
	C_big_endian => C_big_endian, C_branch_likely => C_branch_likely,
	C_sign_extend => C_sign_extend, C_movn_movz => C_movn_movz,
	C_mult_enable => C_mult_enable, C_PC_mask => C_PC_mask,
	C_cop0_count => C_cop0_count, C_cop0_config => C_cop0_config,
	C_branch_prediction => C_branch_prediction,
	C_result_forwarding => C_result_forwarding,
	C_load_aligner => C_load_aligner,
	C_fast_ID => C_fast_ID,
	C_register_technology => C_register_technology,
	-- debugging only
	C_debug => C_debug
    )
    port map (
	clk => clk, reset => res, intr => intr,
	imem_addr => imem_addr, imem_data_in => final_to_cpu_i,
	imem_addr_strobe => imem_addr_strobe,
	imem_data_ready => imem_data_ready,
	dmem_addr_strobe => dmem_addr_strobe, dmem_addr => dmem_addr,
	dmem_write => dmem_write, dmem_byte_sel => dmem_byte_sel,
	dmem_data_in => final_to_cpu_d, dmem_data_out => cpu_to_dmem,
	dmem_data_ready => dmem_data_ready,
	trace_addr => trace_addr, trace_data => trace_data
    );
    end generate;

    -- RS232 sio
    G_sio:
    if C_sio generate
    sio: entity work.sio
    generic map (
	C_big_endian => C_big_endian,
	C_clk_freq => C_clk_freq
    )
    port map (
	clk => clk, ce => sio_ce, txd => sio_txd, rxd => rs232_rx,
	bus_write => dmem_write, byte_sel => dmem_byte_sel,
	bus_in => cpu_to_dmem, bus_out => from_sio
    );
    sio_ce <= dmem_addr_strobe when dmem_addr(31 downto 28) = x"f" and
      dmem_addr(4 downto 2) = "001" else '0';
    end generate;

    -- PCM stereo 1-bit DAC
    G_pcmdac:
    if C_pcmdac generate
    process(clk)
    begin
	if rising_edge(clk) then
	    R_dac_acc_l <= (R_dac_acc_l(16) & R_dac_in_l) + R_dac_acc_l;
	    R_dac_acc_r <= (R_dac_acc_r(16) & R_dac_in_r) + R_dac_acc_r;
	end if;
    end process;
    p_tip(3) <= R_dac_acc_l(16) when sw(3) = '0' else video_dac(3);
    p_tip(2) <= R_dac_acc_l(16) when sw(3) = '0' else video_dac(2);
    p_tip(1) <= R_dac_acc_l(16) when sw(3) = '0' else video_dac(1);
    p_tip(0) <= '0' when sw(3) = '0' else video_dac(0);
    p_ring <= R_dac_acc_r(16);
    end generate;

    -- I/O port map:
    -- 0x8*******: (4B, RW) * SRAM
    -- 0xf*****00: (4B, RW) * GPIO (LED, switches/buttons)
    -- 0xf*****04: (4B, RW) * SIO
    -- 0xf*****0c: (4B, WR) * PCM signal
    -- 0xf*****10: (1B, RW) * SPI Flash
    -- 0xf*****14: (1B, RW) * SPI MicroSD
    -- 0xf*****1c: (4B, WR) * FM DDS register

    -- I/O write access:
    process(clk)
    begin
	if rising_edge(clk) and dmem_addr_strobe = '1'
	  and dmem_write = '1' and dmem_addr(31 downto 28) = x"f" then
	    -- GPIO
	    if C_gpio and dmem_addr(4 downto 2) = "000" then
		R_led <= cpu_to_dmem(7 downto 0);
	    end if;
	    -- PCMDAC
	    if C_pcmdac and dmem_addr(4 downto 2) = "011" then
		if dmem_byte_sel(2) = '1' then
		    if C_big_endian then
			R_dac_in_l <= cpu_to_dmem(23 downto 16) &
			  cpu_to_dmem(31 downto 26);
		    else
			R_dac_in_l <= cpu_to_dmem(31 downto 18);
		    end if;
		end if;
		if dmem_byte_sel(0) = '1' then
		    if C_big_endian then
			R_dac_in_r <= cpu_to_dmem(7 downto 0) &
			  cpu_to_dmem(15 downto 10);
		    else
			R_dac_in_r <= cpu_to_dmem(15 downto 2);
		    end if;
		end if;
	    end if;
	    -- SPI Flash
	    if C_flash and dmem_addr(4 downto 2) = "100" then
		R_flash_si <= cpu_to_dmem(7);
		R_flash_sck <= cpu_to_dmem(6);
		R_flash_cen <= cpu_to_dmem(5);
	    end if;
	    -- SPI MicroSD
	    if C_sdcard and dmem_addr(4 downto 2) = "101" then
		R_sdcard_si <= cpu_to_dmem(7);
		R_sdcard_sck <= cpu_to_dmem(6);
		R_sdcard_cen <= cpu_to_dmem(5);
	    end if;
	    -- DDS
	    if C_ddsfm and dmem_addr(4 downto 2) = "111" then
		if C_big_endian then
		    R_dds_div <= cpu_to_dmem(15 downto 10) & 
		      cpu_to_dmem(23 downto 16) & cpu_to_dmem(31 downto 24);
		else
		    R_dds_div <= cpu_to_dmem(21 downto 0);
		end if;
	    end if;
	end if;
    end process;
    led <= R_led when C_gpio else "--------";
    flash_si <= R_flash_si when C_flash else 'Z';
    flash_sck <= R_flash_sck when C_flash else 'Z';
    flash_cen <= R_flash_cen when C_flash else 'Z';
    sdcard_si <= R_sdcard_si when C_sdcard else 'Z';
    sdcard_sck <= R_sdcard_sck when C_sdcard else 'Z';
    sdcard_cen <= R_sdcard_cen when C_sdcard else 'Z';

    process(clk)
    begin
	if C_gpio and rising_edge(clk) then
	    R_sw <= sw;
	    R_btns <= btn_center & btn_up & btn_down & btn_left & btn_right;
	end if;
    end process;

    -- XXX replace with a balanced multiplexer
    process(dmem_addr, R_sw, R_btns, from_sio, flash_so, sdcard_so)
    begin
	case dmem_addr(4 downto 2) is
	when "000"  =>
	    io_to_cpu <="----------------" & "----" & R_sw & "---" & R_btns;
	when "001"  => io_to_cpu <= from_sio;
	when "100"  =>
	    if C_flash then
		io_to_cpu <= "------------------------" & "0000000" & flash_so;
	    else
		io_to_cpu <= "--------------------------------";
	    end if;
	when "101"  =>
	    if C_sdcard then
		io_to_cpu <= "------------------------" & "0000000" & sdcard_so;
	    else
		io_to_cpu <= "--------------------------------";
	    end if;
	when others =>
	    io_to_cpu <= "--------------------------------";
	end case;
    end process;

    final_to_cpu_d <= io_to_cpu when dmem_addr(31 downto 28) = x"f"
      else from_sram when sram_data_strobe = '1'
      else dmem_to_cpu;
    final_to_cpu_i <= from_sram when sram_instr_strobe = '1'
      else imem_to_cpu;

    -- Block RAM
    dmem_bram_enable <= dmem_addr_strobe when dmem_addr(31) /= '1' else '0';
    bram: entity work.bram
    generic map (
	C_mem_size => C_mem_size
    )
    port map (
	clk => clk, imem_addr_strobe => imem_addr_strobe,
	imem_addr => imem_addr, imem_data_out => imem_to_cpu,
	dmem_addr_strobe => dmem_bram_enable, dmem_write => dmem_write,
	dmem_byte_sel => dmem_byte_sel, dmem_addr => dmem_addr,
	dmem_data_out => dmem_to_cpu, dmem_data_in => cpu_to_dmem
    );

    -- SRAM
    sram_data_strobe <= dmem_addr_strobe when
      dmem_addr(31 downto 28) = x"8" and C_sram else '0';
    dmem_data_ready <= sram_data_ready when sram_data_strobe = '1' else '1';
    sram_instr_strobe <= imem_addr_strobe when
      imem_addr(31 downto 28) = x"8" and C_sram else '0';
    imem_data_ready <= sram_instr_ready when sram_instr_strobe = '1' else '1';
    sram: entity work.sram
    generic map (
	C_sram_wait_cycles => C_sram_wait_cycles
    )
    port map (
	clk => clk, sram_a => sram_a, sram_d => sram_d,
	sram_wel => sram_wel, sram_lbl => sram_lbl, sram_ubl => sram_ubl,
	data_out => from_sram,
	-- Port A: CPU, data bus
	A_addr_strobe => sram_data_strobe, A_write => dmem_write,
	A_byte_sel => dmem_byte_sel, A_addr => dmem_addr(19 downto 2),
	A_data_in => cpu_to_dmem, A_ready => sram_data_ready,
	-- Port B: CPU, instruction bus
	B_addr_strobe => sram_instr_strobe, B_write => '0',
	B_byte_sel => x"f", B_addr => imem_addr(19 downto 2),
	B_data_in => (others => '-'), B_ready => sram_instr_ready,
	-- Port C: video framebuffer
	C_addr_strobe => fb_addr_strobe, C_write => '0',
	C_byte_sel => x"f", C_addr => fb_addr,
	C_data_in => (others => '-'), C_ready => fb_data_ready,
	-- Port D: currently unused
	D_addr_strobe => '0', D_write => '0',
	D_byte_sel => x"f", D_addr => (others => '-'),
	D_data_in => (others => '-'), D_ready => open
    );

    -- debugging design instance
    G_debug:
    if C_debug generate
    debug: entity work.serial_debug
    port map (
	clk => clk_25m, rs232_txd => debug_txd,
	trace_addr => trace_addr, trace_data => trace_data
    );
    end generate;

    rs232_tx <= debug_txd when C_debug and sw(3) = '1' else sio_txd;

    -- DDS FM transmitter
    G_ddsfm:
    if C_ddsfm generate
    process(clk_dds)
    begin
	if (rising_edge(clk_dds)) then
	    R_dds_div1 <= R_dds_div; -- Cross clock domain
	    R_dds_cnt <= R_dds_cnt + R_dds_div1;
	end if;
    end process;
    dds_out <= R_dds_cnt(21);
    end generate;

    -- make a dipole?
    j1(20) <= dds_out when C_ddsfm else 'Z';
    j1(21) <= dds_out when C_ddsfm else 'Z';
    j1(22) <= dds_out when C_ddsfm else 'Z';
    j1(23) <= dds_out when C_ddsfm else 'Z';
    j2(2) <= not dds_out when C_ddsfm else 'Z';
    j2(3) <= not dds_out when C_ddsfm else 'Z';
    j2(4) <= not dds_out when C_ddsfm else 'Z';
    j2(5) <= not dds_out when C_ddsfm else 'Z';

    -- Breakout game, producing PAL composite video
    fb: entity work.fb
    port map (
	clk => clk, clk_dac => clk_dds,
	addr_strobe => fb_addr_strobe,
	addr_out => fb_addr,
	data_ready => fb_data_ready,
	data_in => from_sram,
	dac_out => video_dac
    );

end Behavioral;
