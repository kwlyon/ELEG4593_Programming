lappend auto_path "C:/lscc/diamond/3.14/data/script"
package require simulation_generation
set ::bali::simulation::Para(DEVICEFAMILYNAME) {MachXO3D}
set ::bali::simulation::Para(PROJECT) {sim2}
set ::bali::simulation::Para(PROJECTPATH) {C:/Users/kwlyon/Box/Codex/HW9_5_Code}
set ::bali::simulation::Para(FILELIST) {"C:/Users/kwlyon/Box/Codex/HW9_5_Code/bus_pkg.vhd" "C:/Users/kwlyon/Box/Codex/AD7928_core/AD7928_model.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/SPRAM.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/Bus_Master.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/My_Own_FN_FIFO.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/UART_rs232.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/UART_rs232_fifo_wrap.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/uart_rs232_fifo_wrap_controller.vhd" "C:/Users/kwlyon/Box/Codex/AD7928_core/AD7928_Core.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/adc_spram_client.vhd" "C:/Users/kwlyon/Box/Codex/PWM_Module/PWM_Module.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/pwm_spram_client.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/reset_delay.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/pll_24m93.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/Internal_clk.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/uart_spram_top.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/Bus_Interface_tb_adc_model.vhd" "C:/Users/kwlyon/Box/Codex/HW9_5_Code/Bus_Interface_tb.vhd" }
set ::bali::simulation::Para(GLBINCLIST) {}
set ::bali::simulation::Para(INCLIST) {"none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none" "none"}
set ::bali::simulation::Para(WORKLIBLIST) {"work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" "work" }
set ::bali::simulation::Para(COMPLIST) {"VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" "VHDL" }
set ::bali::simulation::Para(LANGSTDLIST) {"" "" "" "" "" "" "" "" "" "" "" "" "" "" "" "" "" "" }
set ::bali::simulation::Para(SIMLIBLIST) {pmi_work ovi_machxo3d}
set ::bali::simulation::Para(MACROLIST) {}
set ::bali::simulation::Para(SIMULATIONTOPMODULE) {Bus_Interface_TestBench_adc_model}
set ::bali::simulation::Para(SIMULATIONINSTANCE) {}
set ::bali::simulation::Para(LANGUAGE) {VHDL}
set ::bali::simulation::Para(SDFPATH)  {}
set ::bali::simulation::Para(INSTALLATIONPATH) {C:/lscc/diamond/3.14}
set ::bali::simulation::Para(ADDTOPLEVELSIGNALSTOWAVEFORM)  {1}
set ::bali::simulation::Para(RUNSIMULATION)  {1}
set ::bali::simulation::Para(SIMULATION_RESOLUTION)  {ps}
set ::bali::simulation::Para(HDLPARAMETERS) {}
set ::bali::simulation::Para(POJO2LIBREFRESH)    {}
set ::bali::simulation::Para(POJO2MODELSIMLIB)   {}
set ::bali::simulation::Para(OPTIMIZEARGS)  {+acc}
set ::bali::simulation::Para(OPTIMIZATION_DEBUG)  {1}
::bali::simulation::QuestaSim_Run
