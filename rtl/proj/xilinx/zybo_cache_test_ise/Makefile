project = zybo
vendor = xilinx
family = zinq
part = xc7z010-clg400-1 # last number -1 is speed grade
flashsize = 8192 # KB (kilobytes) - SPI flash device is M25P40
top_module = glue
#isedir = /usr/local/xilinx/14.7/ISE_DS
isedir = /opt/Xilinx/ISE/14.7/ISE_DS
xil_env = . $(isedir)/settings64.sh
openocd_interface = ftdi-zybo.ocd
xc3sprog_interface = jtaghs1_fast
xc3sprog_device = 1
SHELL = /bin/bash

vfiles = \
../../../soc/pid/ctrlpid_v.v \
../../../soc/pid/simotor_v.v \
../../../soc/pid/rotary_decoder_v.v \
../../../soc/vgahdmi/vgahdmif_v.v \

vhdfiles = \
../../../xilinx/zybo_toplevel_cache_test.vhd \
../../../generic/glue_sram_min.vhd \
../../../generic/bram_mi32_eb.vhd \
../../../generic/bram_mi32_el.vhd \
../../../generic/bram_rv32.vhd \
../../../generic/bram_true2p_1clk.vhd \
../../../generic/bptrace.vhd \
../../../generic/reg1w2r.vhd \
../../../cpu/cache_generic_param.vhd \
../../../cpu/alu.vhd \
../../../cpu/loadalign.vhd \
../../../cpu/pipeline.vhd \
../../../cpu/shift.vhd \
../../../cpu/defs_f32c.vhd \
../../../cpu/defs_mi32.vhd \
../../../cpu/defs_rv32.vhd \
../../../cpu/idecode_mi32.vhd \
../../../cpu/idecode_rv32.vhd \
../../../cpu/debug.vhd \
../../../soc/sram.vhd \
../../../soc/sram_pack.vhd \
../../../soc/sio.vhd \
../../../soc/spi.vhd \
../../../soc/gpio.vhd \
../../../soc/timer.vhd \
../../../soc/pid/pid.vhd \
../../../soc/pid/ctrlpid.vhd \
../../../soc/pid/simotor.vhd \
../../../soc/pid/rotary_decoder.vhd \
../../../soc/fm/fm.vhd \
../../../soc/fm/fmgen.vhd \
../../../soc/fm/rds.vhd \
../../../soc/fm/bram_rds.vhd \
../../../soc/fm/message.vhd \
../../../soc/vgahdmi/vgahdmif.vhd \
../../../soc/vgahdmi/videofifo.vhd \
../../../xilinx/hdmi_out_xc7.vhd \
../../../xilinx/pll_125M_250M_100M_25M.vhd \
../../../xilinx/mmcm_125M_81M25_250M521_25M052.vhd \
../../../generic/bram_video.vhd \

include ../include/xilinx.mk
