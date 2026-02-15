#!/bin/sh

# Run Icarus verilog:
iverilog -g2012 -Wall -o sim_gps_baseband_axi4lite \
  ../test/tb_gps_baseband_axi4lite.sv \
  ../rtl/time_base.v\
  ../rtl/code_nco.v \
  ../rtl/code_gen.v \
  ../rtl/carrier_nco.v \
  ../rtl/carrier_mixer.v \
  ../rtl/accumulator.v \
  ../rtl/epoch_counter.v \
  ../rtl/tracking_channel.v \
  ../rtl/gps_baseband.v \
  ../rtl/gps_baseband_axi4lite_wrapper.v
vvp sim_gps_baseband_axi4lite

