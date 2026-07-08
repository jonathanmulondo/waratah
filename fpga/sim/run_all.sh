#!/bin/sh
#-----------------------------------------------------------------------------
# run_all.sh -- compile and run every EDDI-FPGA testbench in Questa (CLI)
# Usage: from fpga/sim:  sh run_all.sh   [testbench_name]
#-----------------------------------------------------------------------------
set -e
cd "$(dirname "$0")"

python gen_vectors.py

[ -d work ] || vlib work
vlog -quiet ../rtl/fft_r2.v ../rtl/psd_avg.v ../rtl/qtn_core.v \
     ../rtl/debris_detector.v ../rtl/watchdog.v \
     +incdir+mem \
     ../tb/tb_fft.v ../tb/tb_qtn.v ../tb/tb_debris.v ../tb/tb_watchdog.v

TBS="${1:-tb_fft tb_qtn tb_debris tb_watchdog}"
FAILED=0
for tb in $TBS; do
    echo "================ $tb ================"
    vsim -c -quiet work.$tb -do "run -all; quit -f" | tee $tb.log | grep -E "TB_PASS|TB_FAIL|FAIL|Error" || true
    grep -q "TB_PASS" $tb.log || FAILED=1
done

echo "====================================="
if [ $FAILED -eq 0 ]; then echo "ALL TESTBENCHES PASSED"; else echo "SOME TESTBENCHES FAILED"; exit 1; fi
