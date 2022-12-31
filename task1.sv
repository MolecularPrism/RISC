`define idle 3'b000
`define writeRf 3'b001
`define load_A 3'b010
`define load_B 3'b011
`define load_output 3'b100
`define HALT 3'b101
`define RAMop 3'b110
`define Rn 2'b10
`define Rd 2'b01
`define Rm 2'b00

module task1(input clk, input rst_n, input [7:0] start_pc, output[15:0] out);

   reg [7:0] pc_curr;
   reg [7:0] next_pc;
   wire [15:0] datapath_out;
   reg [7:0] data_addr_reg_out;
   reg ram_addr;
   reg [15:0] ram_r_data;
   reg [1:0] ALU_op;
   reg [1:0] shift_op;
   reg start;
   reg [2:0] opcode;
   reg waiting;
   reg load_ir;
   reg load_pc;
   reg clear_pc;
   reg load_addr;
   reg sel_addr;
   reg ram_w_en;
   reg [1:0] reg_sel;
   reg [1:0] wb_sel;
   reg w_en, en_A, en_B, en_C, sel_A, sel_B;
   reg [15:0] sximm5;
   reg [15:0] sximm8;
   reg [15:0] ir_reg_out;

   assign start = ~waiting;
   
controller Contr(clk, rst_n, start, opcode, ALU_op, shift_op, waiting, 
load_ir, load_pc , clear_pc, load_addr, sel_addr, ram_w_en, reg_sel, wb_sel, w_en, en_A, en_B, en_C, sel_A, sel_B);

idecoder Dec(ir_reg_out, reg_sel, opcode, ALU_op, shift_op, sximm5, sximm8, r_addr, w_addr);

datapath Datapath( clk,  ram_r_data,  pc_curr,  wb_sel, w_addr, w_en,  r_addr,  en_A, en_B, shift_op, 
sel_A, sel_B, ALU_op,en_C, sximm8, sximm5, datapath_out);

assign out = datapath_out;  

ram RAM(clk, ram_w_en, ram_addr, ram_addr,
           datapath_out, ram_r_data);

always_ff@(posedge clk) begin //instruction register
  if(load_ir) begin
    ir_reg_out <= ram_r_data;
  end
  
end

always_comb begin //#4 multiplexer
  case(clear_pc)
    1'b0: begin
      next_pc = pc_curr + 1;
    end
    1'b1: begin
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

endmodule: task1

module controller(input clk, input rst_n, input start,
                  input [2:0] opcode, input [1:0] ALU_op, input [1:0] shift_op,
                  output waiting, output load_ir, output load_pc , 
                  output clear_pc, output load_addr, output sel_addr,
                  output ram_w_en,
                  output [1:0] reg_sel, output [1:0] wb_sel, output w_en,
                  output en_A, output en_B, output en_C,
                  output sel_A, output sel_B);

reg [2:0] state;

reg [2:0] next_state;
reg isWaiting;

reg current_load_ir;
reg current_load_pc;
reg current_clear_pc;
reg current_load_addr;
reg current_sel_addr;
reg current_ram_w_en;
reg [1:0]current_reg_sel;
reg [1:0]current_wb_sel;
reg current_w_en;
reg current_en_A;
reg current_en_B;
reg current_en_C;
reg current_sel_A;
reg current_sel_B;

assign sel_addr = current_sel_addr;
assign load_ir = current_load_ir;
assign load_pc = current_load_pc;
assign clear_pc = current_clear_pc;
assign load_addr = current_load_addr;
assign ram_w_en = current_ram_w_en;
assign w_en = current_w_en;
assign en_A = current_en_A;
assign en_B = current_en_B;
assign en_C = current_en_C;
assign sel_A = current_sel_A;
assign sel_B = current_sel_B;
assign wb_sel = current_wb_sel;
assign waiting = isWaiting;


always_ff@(posedge clk) begin
  if(~rst_n) begin
    state <= `idle;
  
  end 
  else if(rst_n) begin
    state <= next_state;

 
  end
end

always_comb begin
  case(state)
    `idle: isWaiting = 1'b1;
    `writeRf: isWaiting = 1'b0;
    `load_A: isWaiting = 1'b0;
    `load_B: isWaiting = 1'b0;
    `load_output: isWaiting = 1'b0;
    `HALT: isWaiting = 1'b0;
    `RAMop: isWaiting = 1'b0;
    default: isWaiting = 1'b0;
  endcase
end


//A: MOV Rn, #<im8>
//B: MOV Rd, Rm{,<sh_op>}
//C: ADD Rd, Rn, Rm{,<sh_op>}
//D: CMP Rn, Rm{,<sh_op>}
//E: AND Rd, Rn, Rm{,<sh_op>} 
//F: MVN Rd, Rm{,<sh_op>} 
//G: LDR Rd,[Rn{,#<im5>}] 
//H: STR Rd,[Rn{,#<im5>}] 
//I: HALT

/*YOU WANT LOAD_PC AND LOAD_ADDR TO BE ALWAYS ON BECAUSE THEY ARE NOT PART
OF THE STATES AND JUST KEEP THEM ON*/

/*sel_addr needs to be 1'b1 forever i think???*/

/*after reset, clear_pc 1'b1. Otherwise, clear_pc = 1'b0*/
always_comb begin
  casex({start, rst_n, state, opcode, ALU_op})
    10'b0100011010: begin //idle -> idle (A loaded)
      current_reg_sel = `Rn;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b10;
      current_w_en = 1'b0;
      
      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      
      next_state = `idle;
    end
    10'b1100011010: begin //idle -> A1 (001)
      current_reg_sel = `Rn;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b10;
      current_w_en = 1'b1;
      
      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      
      next_state = `writeRf;
    end

    10'bx100111010: begin //A1 -> idle (000)
      current_reg_sel = `Rn;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b10;
      current_w_en = 1'b1; //changed this to one because you are in writing state rn and w_en needs to be 1
     
      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      
      
      next_state = `idle;
      
    end

    10'b0100011000: begin //idle -> idle (B loaded)
      current_reg_sel = `Rm;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;
      
      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end
    10'b1100011000: begin //idle -> B1 (011)
      current_reg_sel = `Rm;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `load_B;
    end
    10'bx101111000: begin //B1 -> B2 (100)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0; 
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      
      
      next_state = `load_output;
    end
    10'bx110011000: begin //B2 -> B3 (001)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b1;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      
      
      next_state = `writeRf;
    end
    10'bx100111000: begin //B3 -> idle (000)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b1;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      
      
      next_state = `idle;
    end

    10'b0100010100: begin //idle -> idle (C loaded)
      current_reg_sel = `Rn;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end
    10'b1100010100: begin //idle -> C1 (010)
      current_reg_sel = `Rm;
      current_en_A = 1'b1;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `load_A;
    end
    10'bx101010100: begin //C1 -> C2 (011)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `load_B;
    end
    10'bx101110100: begin //C2 -> C3 (100)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b1;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
     

      next_state = `load_output;
    end
    10'bx110010100: begin //C3 -> C4 (001)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b1;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `writeRf;
    end
    10'bx100110100: begin //C4 -> idle (000)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `idle;
    end
    
    10'b0100010101: begin //idle -> idle (D loaded)
      current_reg_sel = `Rn;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end
    10'b1100010101: begin //idle -> D1 (010)
      current_reg_sel = `Rm;
      current_en_A = 1'b1;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `load_A;

      
    end
    10'bx101010101: begin //D1 -> D2 (011)
      current_reg_sel = `Rm;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `load_B;
    end
    10'bx101110101: begin //D2 -> D3 (100)
      current_reg_sel = `Rm;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `load_output;
    end
    10'bx110010101: begin //D3 -> idle (000)
      current_reg_sel = `Rm;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `idle;
    end
    
    10'b0100010110: begin //idle -> idle (E loaded)
      current_reg_sel = `Rn;
      current_en_A = 1'b1;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end
    10'b1100010110: begin //idle -> E1 (010)
      current_reg_sel = `Rm;
      current_en_A = 1'b1;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `load_A;
    end
    10'bx101010110: begin //E1 -> E2 (011)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `load_B;
    end
    10'bx101110110: begin //E2 -> E3 (100)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b1;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `load_output;
    end
    10'bx110010110: begin //E3 -> E4 (001)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b1;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `writeRf;
    end
    10'bx100110110: begin //E4 -> idle (001)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `idle;
    end
    
    10'b0100010111: begin //idle -> idle (000)
      current_reg_sel = `Rm;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end
    10'b1100010111: begin //idle -> F1 (011)
      current_reg_sel = `Rm;
      current_en_A = 1'b0;
      current_en_B = 1'b1;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `load_B;
    end
    10'bx101110111: begin //F1 -> F2 (100)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b1;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `load_output;
    end
    10'bx110010111: begin //F2 -> F3 (001)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b1;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `writeRf;
    end
    10'bx100110111: begin //F3 -> idle (000)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b1;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b1;
      
      current_sel_addr = 1'b1;
      current_load_ir = 1'b1;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      

      next_state = `idle;
    end

    {1'b0, 1'b1, `idle, 3'b111, 2'bxx}: begin //idle -> idle (000)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;      
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b0;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end

    {1'b1, 1'b1, `idle, 3'b111, 2'bxx}: begin //idle -> HALT (101)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;      
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `HALT;
    end

    {1'bx, 1'b1, `HALT, 3'bxxx, 2'bxx}: begin //HALT -> HALT (101)
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;      
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `HALT;
    end

    {1'bx, 1'b0, 3'bxxx, 3'bxxx, 2'bxx}: begin //reset is being pressed
      current_reg_sel = `Rd;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;      
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b00;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b1;
      current_load_addr = 1'b0;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end

    default:begin
      current_reg_sel = `Rn;
      current_en_A = 1'b0;
      current_en_B = 1'b0;
      current_en_C = 1'b0;
      current_sel_A = 1'b0;
      current_sel_B = 1'b0;
      current_wb_sel = 2'b10;
      current_w_en = 1'b0;

      current_sel_addr = 1'b1;
      current_load_ir = 1'b0;
      current_load_pc = 1'b1;
      current_clear_pc = 1'b0;
      current_load_addr = 1'b1;
      current_ram_w_en = 1'b0; 
      next_state = `idle;
    end
  endcase
end   

endmodule: controller





module datapath(input clk, input [15:0] mdata, input [7:0] pc, input [1:0] wb_sel,
                input [2:0] w_addr, input w_en, input [2:0] r_addr, input en_A,
                input en_B, input [1:0] shift_op, input sel_A, input sel_B,
                input [1:0] ALU_op, input en_C, 
		input [15:0] sximm8, input [15:0] sximm5,
                output [15:0] datapath_out);
  
  reg [15:0] writeData;
  reg [15:0] r_data;
  reg [15:0] A_reg_out;
  reg [15:0] B_reg_out;
  reg [15:0] val_A;
  reg [15:0] val_B;
  wire [15:0] ALU_out;
  //wire Z;
  //reg N;
  //reg V;
 // reg N_output;
 // reg V_output;
  //reg Z_output;
  reg [15:0] shift_output;
  reg [15:0] datapath_output;

 // assign mdata = 16'b0;
 // assign pc = 8'b0;

  assign w_data = writeData;
 // assign Z_out = Z_output;
 // assign N_out = N_output;
 // assign V_out = V_output;
  assign datapath_out = datapath_output;
 
  always_comb begin //multiplexer #9 logic
    case(wb_sel)
    2'b00: writeData = datapath_out;
    2'b01: writeData = {8'b0, pc};
    2'b10: writeData = sximm8;
    2'b11: writeData = mdata;
    endcase
  end

  regfile RegFile(writeData, w_addr, w_en, r_addr, clk, r_data); //Reg File 

  always_ff@(posedge clk) begin //reg A (#3) with enable
    if(en_A) begin
      A_reg_out <= r_data;
    end 
  end

  always_ff@(posedge clk) begin //reg B (#4) with enable
    if(en_B) begin
      B_reg_out <= r_data;
    end 
  end

  shifter Shifter(B_reg_out, shift_op, shift_output); //shifter

  always_comb begin //multiplexer #6 logic
    case(sel_A)
    1'b0: val_A = A_reg_out;
    1'b1: val_A = 16'b0;
    endcase
  end

  always_comb begin //multiplexer #7 logic
    case(sel_B)
    1'b0: val_B = shift_output;
    1'b1: val_B = sximm5;
    endcase
  end

  ALU alu(val_A, val_B, ALU_op, ALU_out); //ALU 
/*
  always_comb begin //determine whether overflow or underflow occurred and compute V
        casex({val_A[15], val_B[15], ALU_out[15], ALU_op})
        5'b00100: V = 1'b1;
        5'b11000: V = 1'b1;
        5'b01101: V = 1'b1;
        5'b10001: V = 1'b1;
        default: V = 1'b0;
        endcase 
  end

 assign N = ALU_out[15]; //N = 1 if ALU_out is neg

  always_ff@(posedge clk) begin //status register
    if(en_status)begin
        Z_output <= Z;
        N_output <= N;
        V_output <= V;
    end
  end */

  always_ff@(posedge clk) begin //C register logic
    if(en_C) begin
      datapath_output = ALU_out;
    end 
  end
endmodule: datapath




module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output [2:0] opcode, output [1:0] ALU_op, output [1:0] shift_op,
		output [15:0] sximm5, output [15:0] sximm8,
                output [2:0] r_addr, output [2:0] w_addr);
        

reg [2:0] r_address;
reg [2:0] w_address;
wire [2:0] Rm;
wire [2:0] Rd;
wire [2:0] Rn;
reg [15:0] sximm_5;
reg [15:0] sximm_8;

assign r_addr = r_address;
assign w_addr = w_address;

assign shift_op = ir[4:3];
assign Rm = ir[2:0];
assign Rd = ir[7:5];
assign Rn = ir[10:8];
assign opcode = ir[15:13];
assign ALU_op = ir[12:11];
assign sximm5 = sximm_5;
assign sximm8 = sximm_8;




always_comb begin //multiplexer controlling output r_addr and w_addr
        case(reg_sel)
        2'b00: begin
           r_address = Rm;
           w_address = Rm;     
        end
                
        2'b01: begin
           r_address = Rd;
           w_address = Rd;     
        end
        2'b10: begin
           r_address = Rn;
           w_address = Rn;
        end
        2'b11: begin
           r_address = Rn; //random
           w_address = Rn;
        end
        endcase    
end

always_comb begin //sign extend for sximm5
        casex(ir[4:0])
        5'b0xxxx: sximm_5 = {11'b00000000000, ir[4:0]};
        5'b1xxxx: sximm_5 = {11'b11111111111, ir[4:0]};
        default: sximm_5 = {11'b00000000000, ir[4:0]};
        endcase
end

always_comb begin //sign extend for sximm8
        casex(ir[7:0])
        8'b0xxxxxxx: sximm_8 = {8'b00000000, ir[7:0]};
        8'b1xxxxxxx: sximm_8 = {8'b11111111, ir[7:0]};
        default: sximm_8 = {8'b00000000, ir[7:0]};
        endcase
end

endmodule: idecoder


module regfile(input logic clk, input logic [15:0] w_data, input logic [2:0] w_addr, input logic w_en, input logic [2:0] r_addr, output logic [15:0] r_data);
    logic [15:0] m[0:7];
    assign r_data = m[r_addr];
    always_ff @(posedge clk) if (w_en) m[w_addr] <= w_data;
endmodule: regfile

module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, output [15:0] ALU_out);
reg [15:0] ALU_output;

assign ALU_out = ALU_output;


    always_comb begin
      case(ALU_op)
        2'b00: ALU_output = val_A + val_B;
        2'b01: ALU_output = val_A - val_B;
        2'b10: ALU_output = val_A & val_B;
        2'b11: ALU_output = ~val_B;
        default: ALU_output = 16'b0000000000000000;
      endcase
    end

    

endmodule: ALU

module shifter(input [15:0] shift_in, input [1:0] shift_op, output reg [15:0] shift_out);
  
  reg temp1;
  reg [15:0] temp2;

  always_comb begin : shift_select
    case (shift_op)
      2'b00: begin
        shift_out = shift_in; //no shift
        temp1 = 0;
        temp2 = 16'b0000000000000000;
      end
      2'b01: begin
        shift_out = shift_in << 1; //left shift
        temp1 = 0;
        temp2 = 16'b0000000000000000;
      end
      2'b10: begin 
        shift_out = shift_in >> 1; //logical right shift
        temp1 = 0;
        temp2 = 16'b0000000000000000;
      end
      2'b11:begin 
              temp1 = shift_in[15];
              temp2 = shift_in >> 1; //arithmatic right shift
              shift_out = {temp1, temp2[14:0]};
            end
      default: shift_out = shift_in;
    endcase
  end

endmodule: shifter