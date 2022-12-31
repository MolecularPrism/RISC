module datapath(input clk, input [15:0] mdata, input [7:0] pc, input [1:0] wb_sel,
                input [2:0] w_addr, input w_en, input [2:0] r_addr, input en_A,
                input en_B, input [1:0] shift_op, input sel_A, input sel_B,
                input [1:0] ALU_op, input en_C, input en_status,
		input [15:0] sximm8, input [15:0] sximm5,
                output [15:0] datapath_out, output Z_out, output N_out, output V_out);
  
  reg [15:0] writeData;
  reg [15:0] r_data;
  reg [15:0] A_reg_out;
  reg [15:0] B_reg_out;
  reg [15:0] val_A;
  reg [15:0] val_B;
  wire [15:0] ALU_out;
  wire Z;
  reg N;
  reg V;
  reg N_output;
  reg V_output;
  reg Z_output;
  reg [15:0] shift_output;
  reg [15:0] datapath_output;

  //assign mdata = 16'b0;
 // assign pc = 8'b0;

  assign w_data = writeData;
  assign Z_out = Z_output;
  assign N_out = N_output;
  assign V_out = V_output;
  assign datapath_out = datapath_output;
 
  always_comb begin //multiplexer #9 logic
    case(wb_sel)
    2'b00: writeData = datapath_out;
    2'b01: writeData = {8'b0, pc};
    2'b10: writeData = sximm8;
    2'b11: writeData = mdata;
    endcase
  end

  regfile RegFile(clk, writeData, w_addr, w_en, r_addr, r_data); //Reg File 

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

  ALU alu(val_A, val_B, ALU_op, ALU_out, Z); //ALU 

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
  end 

  always_ff@(posedge clk) begin //C register logic
    if(en_C) begin
      datapath_output = ALU_out;
    end 
  end
endmodule: datapath
