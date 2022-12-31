module ALU(input [15:0] val_A, input [15:0] val_B, input [1:0] ALU_op, output [15:0] ALU_out, output Z);
reg [15:0] ALU_output;
reg Z_1;
assign ALU_out = ALU_output;
assign Z = Z_1;

    always_comb begin
      case(ALU_op)
        2'b00: ALU_output = val_A + val_B;
        2'b01: ALU_output = val_A - val_B;
        2'b10: ALU_output = val_A & val_B;
        2'b11: ALU_output = ~val_B;
        default: ALU_output = 16'b0000000000000000;
      endcase
    end

    always_comb begin
      case(ALU_out)
      16'b0: Z_1 = 1'b1;
      default: Z_1 = 1'b0;
      endcase
    end

endmodule: ALU