#!/bin/sh
#-----------------------------------------------------------------------------
# run_all.sh -- compile and run every EDDI-FPGA testbench in Questa (CLI)
# Usage: from fpga/sim:  sh run_all.sh   [testbench_name]
#-----------------------------------------------------------------------------
set -e
cd "$(dirname "$0")"

python gen_vectors.py

[ -d work ] || vlib work
vlog -quiet ../rtl/*.v +incdir+mem ../tb/*.v

TBS="${1:-tb_fft tb_qtn tb_debris tb_watchdog tb_cic tb_burst tb_payload tb_top}"
FAILED=0
for tb in $TBS; do
    echo "================ $tb ================"
    vsim -c -quiet work.$tb -do "run -all; quit -f" | tee $tb.log | grep -E "TB_PASS|TB_FAIL|FAIL|Error" || true
    grep -q "TB_PASS" $tb.log || FAILED=1
done

echo "====================================="
if [ $FAILED -eq 0 ]; then echo "ALL TESTBENCHES PASSED"; else echo "SOME TESTBENCHES FAILED"; exit 1; fi
