
module task3(input clk, input rst_n, input [7:0] start_pc, output [15:0] outdata);
   
   reg [7:0] pc_curr;  //regs: ir_reg_out, next_pc, pc_curr, data_addr_reg_out, ram_addr
   reg [7:0] next_pc;
   reg [15:0] datapath_out;
   reg [7:0] data_addr_reg_out;
   reg [7:0] ram_addr;
   wire [15:0] ram_r_data;
   wire [1:0] ALU_op;
   wire [1:0] shift_op;
   wire [2:0] opcode;
   wire load_ir;
   wire load_pc;
   wire clear_pc;
   wire load_addr;
   wire sel_addr;
   wire ram_w_en;
   wire [1:0] reg_sel;
   wire [1:0] wb_sel;
   wire w_en, en_A, en_B, en_C, sel_A, sel_B;
   wire [15:0] sximm5;
   wire [15:0] sximm8;
   reg [15:0] ir_reg_out;
   wire Z_out;
   wire V_out;
   wire N_out;
   wire [2:0] cond;
   wire [2:0] r_addr;
   wire [2:0] w_addr;
   wire en_status;
   wire jump_sel;

  assign outdata = datapath_out;
   
controller Contr(clk, rst_n,
                  opcode, ALU_op, cond, shift_op,
                  Z_out, N_out, V_out,
                  load_ir, load_pc , 
                  clear_pc, load_addr, sel_addr,
                  ram_w_en,
                  reg_sel, wb_sel, w_en,
                  en_A, en_B, en_C,
                  sel_A, sel_B, en_status, jump_sel);

                  

idecoder Dec(ir_reg_out, reg_sel, opcode, ALU_op, shift_op, sximm5, sximm8, r_addr, w_addr, cond);



datapath Datapath(clk, ram_r_data, pc_curr, wb_sel, w_addr, w_en, r_addr, en_A, en_B, shift_op, sel_A, sel_B
, ALU_op, en_C, en_status, sximm8, sximm5, datapath_out, Z_out, N_out, V_out);


ram RAM(clk, ram_w_en, ram_addr, ram_addr,
           datapath_out, ram_r_data);

always_ff@(posedge clk) begin //instruction register
  if(load_ir) begin
    ir_reg_out <= ram_r_data;
  end
  
end

always_comb begin //#4 multiplexer
  casex({clear_pc, jump_sel})
    2'b00: begin
      next_pc = pc_curr + 1;
    end
    2'b01: begin
      next_pc = pc_curr + 1 + sximm8;
    end
    2'b1x: begin
      next_pc = start_pc;
    end
    default: begin
      next_pc = start_pc;
    end
  endcase
end



always_ff@(posedge clk) begin //program counter reg
  if(load_pc) begin
    pc_curr <= next_pc;
  end
end

always_ff@(posedge clk) begin //data address register
  if(load_addr)begin
    data_addr_reg_out <= datapath_out;
  end
end

always_comb begin //mux #6
  case(sel_addr)
    1'b0: begin
      ram_addr <= data_addr_reg_out;
    end
    1'b1: begin
      ram_addr <= pc_curr;
    end
  endcase
end



endmodule: task3


