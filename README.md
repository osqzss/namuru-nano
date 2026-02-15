# namuru-nano

This repository provides a Verilog implementation of a single-channel GPS L1 C/A baseband tracking correlator, based on the NAMURU receiver code originally open-sourced by UNSW (The University of New South Wales). The design integrates a carrier NCO, code NCO, C/A code generation, epoch counting, and early/prompt/late I/Q accumulators. Simulation testbenches with file-driven IF input are included to validate code delay (slew), Doppler, and correlation performance.

## Repository Structure

- `rtl/`: Verilog source code for the GPS L1 C/A tracking channel and its submodules.
- `test/`: Testbenches for verifying the SystemVerilog modules.
- `sh/`: Helper scripts to run simulations with Icarus Verilog.
- `sim/`: IF signal generator used by the testbenches.

## Quick Check

### 1\. Generate IF data

Generate a single-PRN IF data stream (text format) with the IF generator:
```bash
cd sim
./gps_if_sim --prn 1 --delay 200 --dopp -2500 --ms 100 --cn0 55 > gps_if.txt
```

### 2\. Run the tracking channel testbench

Run `test/tb_tracking_channel.sv` using Icarus Verilog via the provided script:
```bash
cd sh
./tb_tracking_channel.sh
```
The testbench reads the IF file from Step 1 and outputs correlation results (early/prompt/late I/Q and power) to `corr_dump.txt`.

Make sure the following parameters in `test/tb_tracking_channel.sv` match the IF generator settings:

- `PRN_KEY_INIT`: PRN selection / G2 initial state (See [prn_keys.txt](https://github.com/osqzss/namuru-nano/blob/main/prn_keys.txt))
- `CODE_DELAY`: Code delay in chip
- `IF_DOPPLER_HZ`: Carrier Doppler in Hz

### 3\. Plot the outputs

Plot `corr_dump.txt` using the provided Python script:
```bash
cd sh
python corr_dump_plot.py
```

### 4\. Run the multi-channel baseband testbench

`gps_baseband.v` integrates multiple tracking channels and exposes a simple register interface (Avalon-style) for configuration and data readout. The testbench polls `new_data` every 900 us and reads correlator dumps once `new_data` is asserted.

```bash
cd sh
./tb_gps_baseband.sh
```

### 5\. Run the AXI4-Lite wrapper testbench (Updated)

`gps_baseband_axi4lite_wrapper.v` provides an AXI4-Lite slave interface on top of `gps_baseband.v`. Due to synchronous `read_data` updates in the baseband, the wrapper intentionally pipelines read responses to guarantee coherent register reads.

```bash
cd sh
./tb_gps_baseband_axi4lite.sh
```

## Status

(**2026/01/13**) As of now, the publicly available HDL is verified up to `tracking_channel.sv`.
An AXI4-Lite control interface wrapper will be added after its operation is fully validated.

(**2026/02/15**) Verified in simulation (Icarus Verilog): `tracking_channel.v`, multi-channel `gps_baseband.v`, and AXI4-Lite access via `gps_baseband_axi4lite_wrapper.v` using file-driven IF input.

**Upcoming work**: CDC support for mixed clock domains (CPU/AXI vs RF sampling clock) with per-dump NCO updates.
