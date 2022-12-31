module idecoder(input [15:0] ir, input [1:0] reg_sel,
                output [2:0] opcode, output [1:0] ALU_op, output [1:0] shift_op,
		output [15:0] sximm5, output [15:0] sximm8,
                output [2:0] r_addr, output [2:0] w_addr, output [2:0] cond);
        

reg [2:0] r_address;
reg [2:0] w_address;
wire [2:0] Rm;
wire [2:0] Rd;
wire [2:0] Rn;
reg [15:0] sximm_5;
reg [15:0] sximm_8;
reg [2:0] cond_1;

assign r_addr = r_address;
assign w_addr = w_address;
assign cond = cond_1;
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

always_comb begin
        case(opcode)
        3'b001: begin
                cond_1 = ir[10:8];
        end
        default: begin
                cond_1 = 3'b000;
        end
        endcase
end

endmodule: idecoder