onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb_task1/DUT/clk
add wave -noupdate /tb_task1/DUT/rst_n
add wave -noupdate /tb_task1/DUT/start_pc
add wave -noupdate /tb_task1/DUT/outdata
add wave -noupdate /tb_task1/DUT/pc_curr
add wave -noupdate /tb_task1/DUT/next_pc
add wave -noupdate /tb_task1/DUT/datapath_out
add wave -noupdate /tb_task1/DUT/data_addr_reg_out
add wave -noupdate /tb_task1/DUT/ram_addr
add wave -noupdate /tb_task1/DUT/ram_r_data
add wave -noupdate /tb_task1/DUT/ALU_op
add wave -noupdate /tb_task1/DUT/shift_op
add wave -noupdate /tb_task1/DUT/opcode
add wave -noupdate /tb_task1/DUT/load_ir
add wave -noupdate /tb_task1/DUT/load_pc
add wave -noupdate /tb_task1/DUT/clear_pc
add wave -noupdate /tb_task1/DUT/load_addr
add wave -noupdate /tb_task1/DUT/sel_addr
add wave -noupdate /tb_task1/DUT/ram_w_en
add wave -noupdate /tb_task1/DUT/reg_sel
add wave -noupdate /tb_task1/DUT/wb_sel
add wave -noupdate /tb_task1/DUT/w_en
add wave -noupdate /tb_task1/DUT/en_A
add wave -noupdate /tb_task1/DUT/en_B
add wave -noupdate /tb_task1/DUT/en_C
add wave -noupdate /tb_task1/DUT/sel_A
add wave -noupdate /tb_task1/DUT/sel_B
add wave -noupdate /tb_task1/DUT/sximm5
add wave -noupdate /tb_task1/DUT/sximm8
add wave -noupdate /tb_task1/DUT/ir_reg_out
add wave -noupdate /tb_task1/DUT/Z_out
add wave -noupdate /tb_task1/DUT/V_out
add wave -noupdate /tb_task1/DUT/N_out
add wave -noupdate /tb_task1/DUT/cond
add wave -noupdate /tb_task1/DUT/r_addr
add wave -noupdate /tb_task1/DUT/w_addr
add wave -noupdate /tb_task1/DUT/en_status
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 150
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {0 ps} {880 ps}
