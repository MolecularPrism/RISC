`define COUNT_RST 5'b00000
`define COUNT 5'b00001  
`define COUNT_JUMP 5'b00010  
`define IDLE 5'b00011  
`define DECODE 5'b00100 
`define RM_TO_B 5'b00101 
`define RN_TO_A 5'b00110 
`define RD_TO_B 5'b00111 
`define LOAD_SELA1_SELB0 5'b01000 
`define LOAD_SELA0_SELB0 5'b01001 
`define LOAD_SELA0_SELB1 5'b01010 
`define STATUS 5'b01011 
`define WRITE_CONST 5'b01100 
`define WRITEBACK_RD 5'b01101 
`define WRITE_MDATA 5'b01110 
`define DAR 5'b01111 
`define WRITE_RAM 5'b10000
`define WAIT_FOR_RDATA 5'b10001
`define ISRESET 5'b10011

`define Rn 2'b10
`define Rd 2'b01
`define Rm 2'b00

`define a_mdata 2'b11
`define a_sximm8 2'b10 
`define a_C 2'b00

`define ON 1'b1
`define OFF 1'b0  

module controller(input clk, input rst_n,
                  input [2:0] opcode, input [1:0] ALU_op, input [2:0] cond, input [1:0] shift_op, 
                  input isZero, input isNeg, input isOver,
                  output load_ir, output load_pc , 
                  output clear_pc, output load_addr, output sel_addr,
                  output ram_w_en,
                  output [1:0] reg_sel, output [1:0] wb_sel, output w_en,
                  output en_A, output en_B, output en_C,
                  output sel_A, output sel_B, output en_status, output jump_sel);

reg [4:0] state;

reg [4:0] next_state;

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
reg current_en_status;
reg current_jump_sel;
reg isZ;
reg isN;
reg isV;

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
assign reg_sel = current_reg_sel;
assign en_status = current_en_status;
assign jump_sel = current_jump_sel;
assign isZero = isZ;
assign isNeg = isN;
assign isOver = isV;


always_ff@(posedge clk) begin
  if(~rst_n) begin
    state <= `ISRESET;
  
  end 
  else if(rst_n) begin
    state <= next_state;

 
  end
end

always_comb begin
  casex({opcode, ALU_op, cond, state, isZ, isN, isV})


  {8'bxxxxxxxx, `ISRESET, 3'bxxx}: begin
    next_state = `COUNT_RST;
    
    current_clear_pc = 1'b1;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'bxxxxxxxx, `COUNT_RST, 3'bxxx}: begin
    next_state = `WAIT_FOR_RDATA;

    current_clear_pc = 1'b1;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'bxxxxxxxx, `WAIT_FOR_RDATA, 3'bxxx}: begin
    next_state = `DECODE;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = 2'd0;
    current_wb_sel = `a_sximm8;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `ON;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'bxxxxxxxx, `COUNT, 3'bxxx}: begin
    next_state = `WAIT_FOR_RDATA;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b001xxxxx, `COUNT_JUMP, 3'bxxx}: begin
    next_state = `WAIT_FOR_RDATA;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end  

  //MOV Rn, #im8
  {8'b11010xxx, `DECODE, 3'bxxx}: begin
    next_state = `WRITE_CONST;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = `Rn;
    current_wb_sel = `a_sximm8;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `ON;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF; 
     current_en_status = `OFF;
  end
  {8'b11010xxx, `WRITE_CONST, 3'bxxx}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //MOV Rd, Rm
  {8'b11000xxx, `DECODE, 3'bxxx}: begin
    next_state = `RM_TO_B;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rm;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `ON;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b11000xxx, `RM_TO_B, 3'bxxx}: begin
    next_state = `LOAD_SELA1_SELB0;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b1;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `ON;
     current_en_status = `OFF;
  end
  {8'b11000xxx, `LOAD_SELA1_SELB0, 3'bxxx}: begin
    next_state = `WRITEBACK_RD;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rd;
    current_wb_sel = `a_C;
    current_sel_A = 1'b1;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `ON;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b11000xxx, `WRITEBACK_RD, 3'bxxx}: begin
    next_state = `COUNT; 

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //ADD Rd, Rm, Rn
  {8'b10100xxx, `DECODE, 3'bxxx}: begin
    next_state = `RM_TO_B;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rm;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `ON;
    current_en_C = `OFF;
     current_en_status = `OFF;

  end
  {8'b10100xxx, `RM_TO_B, 3'bxxx}: begin
    next_state = `RN_TO_A;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rn;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `ON;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b10100xxx, `RN_TO_A, 3'bxxx}: begin
    next_state = `LOAD_SELA0_SELB0;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `ON;
     current_en_status = `OFF;
  end
  {8'b10100xxx, `LOAD_SELA0_SELB0, 3'bxxx}: begin
    next_state = `WRITEBACK_RD;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rd;
    current_wb_sel = `a_C;
    current_sel_A = 1'b1;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `ON;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b10100xxx, `WRITEBACK_RD, 3'bxxx}: begin
    next_state = `COUNT; 

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //CMP Rn, Rm
  {8'b10101xxx, `DECODE, 3'bxxx}: begin
    next_state = `RN_TO_A;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rn;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `ON;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;

  end
  {8'b10101xxx, `RN_TO_A, 3'bxxx}: begin
    next_state = `RM_TO_B;

    current_clear_pc = 1'b0;  
    current_sel_addr = 1'b1;
    current_reg_sel = `Rm;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `ON;
    current_en_C = `OFF;
     current_en_status = `OFF;

  end
  {8'b10101xxx, `RM_TO_B, 3'bxxx}: begin
    next_state = `STATUS;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `ON;

  end
  {8'b10101xxx, `STATUS, 3'bxxx}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;


  end

  //ADD Rd, Rn, Rm
  {8'b10110xxx, `DECODE, 3'bxxx}: begin
    next_state = `RN_TO_A;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rn;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `ON;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;

  end
  {8'b10110xxx, `RN_TO_A, 3'bxxx}: begin
    next_state = `RM_TO_B;

    current_clear_pc = 1'b0;  
    current_sel_addr = 1'b1;
    current_reg_sel = `Rm;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `ON;
    current_en_C = `OFF;
     current_en_status = `OFF;

  end
  {8'b10110xxx, `RM_TO_B, 3'bxxx}: begin
    next_state = `LOAD_SELA0_SELB0;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `ON;
     current_en_status = `OFF;
  end
  {8'b10110xxx, `LOAD_SELA0_SELB0, 3'bxxx}: begin
    next_state = `WRITEBACK_RD;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rd;
    current_wb_sel = `a_C;
    current_sel_A = 1'b1;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `ON;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b10110xxx, `WRITEBACK_RD, 3'bxxx}: begin
    next_state = `COUNT; 

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //MVN Rd, Rm
  {8'b10111xxx, `DECODE, 3'bxxx}: begin
    next_state = `RM_TO_B;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rm;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `ON;
    current_en_C = `OFF;
     current_en_status = `OFF;

  end
  {8'b10111xxx, `RM_TO_B, 3'bxxx}: begin
    next_state = `LOAD_SELA0_SELB0;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `ON;
     current_en_status = `OFF;
  end
  {8'b10111xxx, `LOAD_SELA0_SELB0, 3'bxxx}: begin
    next_state = `WRITEBACK_RD;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rd;
    current_wb_sel = `a_C;
    current_sel_A = 1'b1;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `ON;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b10111xxx, `WRITEBACK_RD, 3'bxxx}: begin
    next_state = `COUNT; 

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //LDR Rd, [Rn, #im5]
  {8'b01100xxx, `DECODE, 3'bxxx}: begin
    next_state = `RN_TO_A;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rn;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `ON;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b01100xxx, `RN_TO_A, 3'bxxx}: begin
    next_state = `LOAD_SELA0_SELB1;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `ON;
    current_en_status = `OFF;

  end
  {8'b01100xxx, `LOAD_SELA0_SELB1, 3'bxxx}: begin
    next_state = `DAR;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `ON;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b01100xxx, `DAR, 3'bxxx}: begin
    next_state = `IDLE;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b01100xxx, `IDLE, 3'bxxx}: begin
    next_state = `WRITE_MDATA;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rd;
    current_wb_sel = `a_mdata;
    current_sel_A = 1'b0;
    current_sel_B = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `ON;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b01100xxx, `WRITE_MDATA, 3'bxxx}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //STR Rd, [Rn, #im5]
  {8'b10000xxx, `DECODE, 3'bxxx}: begin
    next_state = `RN_TO_A;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = `Rn;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `ON;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b10000xxx, `RN_TO_A, 3'bxxx}: begin
    next_state = `LOAD_SELA0_SELB1;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `ON;
    current_en_status = `OFF;

  end
  {8'b10000xxx, `LOAD_SELA0_SELB1, 3'bxxx}: begin
    next_state = `DAR;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `ON;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b10000xxx, `DAR, 3'bxxx}: begin
    next_state = `RD_TO_B;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = `Rd;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `ON;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b10000xxx, `RD_TO_B, 3'bxxx}: begin
    next_state = `LOAD_SELA1_SELB0;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b1;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `ON;
    current_en_status = `OFF;

  end
  {8'b10000xxx, `LOAD_SELA1_SELB0, 3'bxxx}: begin
    next_state = `WRITE_RAM;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `ON;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
    current_en_status = `OFF;

  end
  {8'b10000xxx, `WRITE_RAM, 3'bxxx}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //HALT
    {8'b111xxxxx, `DECODE, 3'bxxx}: begin
    next_state = `IDLE;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
    end
    {8'b111xxxxx, `IDLE, 3'bxxx}: begin
    next_state = `IDLE;

   current_clear_pc = 1'b0;
    current_sel_addr = 1'b0;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `OFF;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //B
  {8'b00100000, `DECODE, 3'bxxx}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //BEQ
  {8'b00100001, `DECODE, 3'b1xx}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b00100001, `DECODE, 3'b0xx}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //BNE
  {8'b00100010, `DECODE, 3'b0xx}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b00100010, `DECODE, 3'b1xx}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //BLT
  {8'b00100011, `DECODE, 3'bx01}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b00100011, `DECODE, 3'bx10}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b00100011, `DECODE, 3'bx00}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
   {8'b00100011, `DECODE, 3'bx11}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end

  //BLE
  {8'b00100100, `DECODE, 3'b001}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b00100100, `DECODE, 3'b010}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b00100100, `DECODE, 3'b1xx}: begin
    next_state = `COUNT_JUMP;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b1;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
  {8'b00100100, `DECODE, 3'b000}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end
   {8'b00100100, `DECODE, 3'b011}: begin
    next_state = `COUNT;

    current_clear_pc = 1'b0;
    current_sel_addr = 1'b1;
    current_reg_sel = 2'd0;
    current_wb_sel = 2'd0;
    current_sel_A = 1'b0;
    current_sel_B = 1'b0;
    current_jump_sel = 1'b0;

    current_load_ir = `OFF;
    current_load_pc = `ON;
    current_load_addr = `OFF;
    current_ram_w_en = `OFF;
    current_w_en = `OFF;
    current_en_A = `OFF;
    current_en_B = `OFF;
    current_en_C = `OFF;
     current_en_status = `OFF;
  end






  endcase 
end



   

endmodule: controller


