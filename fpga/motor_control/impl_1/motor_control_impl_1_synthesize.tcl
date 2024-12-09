if {[catch {

# define run engine funtion
source [file join {C:/lscc/radiant/2023.1} scripts tcl flow run_engine.tcl]
# define global variables
global para
set para(gui_mode) 1
set para(prj_dir) "X:/Documents/Polarbear/College/Classes/Engineering/MicroPs/RAT/motor_control"
# synthesize IPs
# synthesize VMs
# synthesize top design
file delete -force -- motor_control_impl_1.vm motor_control_impl_1.ldc
run_engine_newmsg synthesis -f "motor_control_impl_1_lattice.synproj"
run_postsyn [list -a iCE40UP -p iCE40UP5K -t SG48 -sp High-Performance_1.2V -oc Industrial -top -w -o motor_control_impl_1_syn.udb motor_control_impl_1.vm] "X:/Documents/Polarbear/College/Classes/Engineering/MicroPs/RAT/motor_control/impl_1/motor_control_impl_1.ldc"

} out]} {
   runtime_log $out
   exit 1
}
