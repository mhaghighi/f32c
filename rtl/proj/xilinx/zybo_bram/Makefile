project = zybo
vendor = xilinx
family = zinq
part = xc7z010-clg400-1 # last number -1 is speed grade
flashsize = 8192 # KB (kilobytes) - SPI flash device is M25P40
top_module = glue
isedir = /usr/local/xilinx/14.7/ISE_DS
SHELL = /bin/bash

# ../../../xilinx/glue_xc6slx9.vhd \

vhdfiles = \
../../../xilinx/zybo_toplevel_bram.vhd \
../../../generic/glue_bram.vhd \
../../../generic/bptrace.vhd \
../../../generic/bram.vhd \
../../../generic/reg1w2r.vhd \
../../../cpu/alu.vhd \
../../../cpu/loadalign.vhd \
../../../cpu/pipeline.vhd \
../../../cpu/shift.vhd \
../../../soc/sio.vhd \
../../../cpu/defs_f32c.vhd \
../../../cpu/defs_mi32.vhd \
../../../cpu/defs_rv32.vhd \
../../../cpu/idecode_mi32.vhd \
../../../cpu/idecode_rv32.vhd \
../../../xilinx/pll_25M_112M5.vhd \
../../../soc/timer.vhd 

include ../include/xilinx.mk