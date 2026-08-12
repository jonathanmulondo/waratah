# build_power_estimate.tcl -- EDDI-FPGA: headless Libero build for a real,
# post-synthesis power number (Stage 2 of the power-budget plan).
#
# Targets the bare MPF300T-1FCG784E directly -- no SoM2 board-support
# package, no PDC yet (blocked on the SE200 carrier's Bank 2/4 voltages).
# eddi_top.v's ports are all generic clk/rst/adc_valid/adc_sample-style
# signals today (the real LVDS ADC interface, adc_if.v, doesn't exist yet),
# so unconstrained default I/O is a reasonable first pass -- this gets you
# real synthesized/placed-and-routed core power now, and the I/O-power
# number can be refined once the PDC exists.
#
# Run from a Windows command prompt (adjust the Libero install path to
# wherever yours landed):
#
#   "C:\Microchip\Libero_SoC_v2024.2\Designer\bin\libero.exe" ^
#       SCRIPT:build_power_estimate.tcl LOGFILE:build_power_estimate.log
#
# Commands below are sourced directly from Microchip's Libero SoC Tcl
# Scripting Overview and Tcl Command Reference (v2024.2) -- every run_tool
# name and argument here is documented, not guessed.

set prj_family  "PolarFire"
set prj_die     "MPF300T"
;# Stand-in package: the real board is FCG784E, but that package isn't in
;# this Libero install's device list (likely gated behind Sundance's SoM2
;# board support package -- requested separately, not yet in hand). 784
;# and 1152 are the SAME die, just different ball counts broken out, so
;# core logic/DSP/LSRAM resources and thus core power are unaffected --
;# only I/O-specific numbers would differ, and those are already the
;# rougher half of the estimate. Swap back to FCG784 once the BSP arrives.
set prj_package "FCG1152"
set prj_speed   "-1"
;# part_range EXT = the "E" in MPF300T-1FCG1152E (Extended Commercial).
set prj_part_range "EXT"

set prj_root     "eddi_top"
set prj_location "./eddi_power_prj"

# ------------------------------------------------------------- new project
eval file delete -force $prj_location
new_project -location $prj_location -name {eddi_power_prj} \
    -project_description {EDDI-FPGA pre-flight power estimate} \
    -hdl {VERILOG} -family "$prj_family" -die "$prj_die" \
    -package "$prj_package" -speed "$prj_speed" \
    -die_voltage {1.0} -part_range "$prj_part_range" \
    -adv_options {IO_DEFT_STD:LVCMOS 1.8V} \
    -adv_options {RESTRICTPROBEPINS:1} \
    -adv_options {RESTRICTSPIPINS:0} \
    -adv_options {TEMPR:EXT} -adv_options {VOLTR:EXT}

# ------------------------------------------------------------- import RTL
# All ten v0.2 modules -- Libero resolves the hierarchy from eddi_top down,
# but every source file needs to be in the project regardless.
foreach f {
    ../rtl/fft_r2.v
    ../rtl/psd_avg.v
    ../rtl/qtn_core.v
    ../rtl/debris_detector.v
    ../rtl/watchdog.v
    ../rtl/cic_decim.v
    ../rtl/burst_ctrl.v
    ../rtl/uart.v
    ../rtl/payload_if.v
    ../rtl/eddi_top.v
} {
    import_files -hdl_source $f -library {work}
}

build_design_hierarchy
set_root -module "${prj_root}::work"

# ------------------------------------------------------------- run the flow
# Each step wrapped in catch, per Microchip's own recommended pattern --
# a failure partway through still leaves a clear log instead of a silent
# stop.
proc run_step {name} {
    if {[catch {run_tool -name $name} err]} {
        puts "STEP FAILED: $name -- $err"
        return 0
    } else {
        puts "STEP PASSED: $name"
        return 1
    }
}

if {[run_step {SYNTHESIZE}]} {
    if {[run_step {PLACEROUTE}]} {
        run_step {VERIFYTIMING}
        ;# VERIFYPOWER is SmartPower -- this is the real, measured number.
        if {[run_step {VERIFYPOWER}]} {
            # Exports into the same MPE-report format as the standalone
            # spreadsheet -- can be imported there for a clean side-by-side
            # against the pre-synthesis estimate.
            smartpower_export_mpe_report -filename {eddi_power_report.xml}
            puts "Power report exported: eddi_power_report.xml"
        }
    }
}

close_project -save 1
