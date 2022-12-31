module tb_task1(output err);

`define YES 1'b1
`define NO 1'b0
`define ANY16BITS 16'd0
 
reg clk;
reg rst_n;
reg [7:0] start_pc;

reg [15:0] outdata;


wire [15:0] mdata;
wire [15:0]sximm8;
wire [15:0] sximm5;
wire [7:0] pc;
wire [1:0] wb_sel;
wire [1:0] shift_op;
wire [1:0] ALU_op;
wire [2:0] w_addr;
wire [2:0] r_addr;
wire w_en,en_A,en_B,sel_A,sel_B,en_C,en_status;
wire Z_out, N_out, V_out;

wire [15:0] ir;
wire [1:0] reg_sel;
wire [2:0] opcode;
wire [2:0] cond;




task3 DUT(clk, rst_n, start_pc, outdata);
datapath DTP(clk, mdata, pc, wb_sel, w_addr, w_en, r_addr, en_A, en_B, shift_op, sel_A, sel_B, ALU_op, en_C,
en_status, sximm8, sximm5, outdata, Z_out, N_out,V_out);


idecoder DCD(ir, reg_sel, opcode, ALU_op, shift_op, sximm5, sximm8, r_addr, w_addr, cond);



reg is_err;
int fail_count;
int pass_count;
int n;

assign err = is_err;

initial begin
  clk <= 1'b0;
  forever #5 clk <= ~clk;
end

task checkOutput;
    input [15:0] expectedOUT;
    input isZero;
    input isNeg;
    begin
        if(DUT.opcode === 3'b110 && DUT.ALU_op === 2'b10) begin
            $display("Instruction: MOV Rn, #im8\n");
            n = 0;
            while(n < 1) begin
                #10;
                n++;
            end
            $display("------------------------------------------\n");
        end 
        else if(DUT.opcode === 3'b110 && DUT.ALU_op === 2'b00) begin
            $display("Instruction: MOV Rd, Rm\n");
            n = 0;
            while(n < 3) begin
                #10;
                n++;
            end
            $display("------------------------------------------\n");
        end
        else if(DUT.opcode === 3'b101 && DUT.ALU_op === 2'b00) begin
            $display("Instruction: ADD Rd, Rn, Rm \n");
            n = 0;
            while(n < 4) begin
                #10;
                n++;
            end
            assert(outdata === expectedOUT) begin
                $display("[PASS] output SUM is %b or %d as expected\n", outdata, expectedOUT);
            end else begin
                $error("[FAIL] out: %b or %d\n", outdata, outdata);
                $error("expectedOUT: %b or %d\n", expectedOUT, expectedOUT);
            end
            $display("------------------------------------------\n");
        
        end
        else if(DUT.opcode === 3'b101 && DUT.ALU_op === 2'b01) begin
            $display("Instruction: CMP Rn, Rm \n");
            n = 0;
            while(n < 3) begin
                #10;
                n++;
            end
            assert(DUT.Z_out === isZero) begin
                $display("[PASS] status Z is %b as expected\n", isZero);

            end else begin
                $error("[FAIL] status Z is %b but expected is %b\n", DUT.Z_out, isZero);
            end
            assert(DUT.N_out === isNeg) begin
                $display("[PASS] status N is %b as expected\n", isNeg);
            end else begin
                $error("[FAIL] status N is %b but expected is %b\n", DUT.N_out, isNeg);
            end
            $display("------------------------------------------\n");
            
        end
        else if(DUT.opcode === 3'b101 && DUT.ALU_op === 2'b10)begin
            $display("Instruction: AND Rd, Rn, Rm \n");
            n = 0;
            while(n < 4) begin
                #10;
                n++;
            end
            assert(outdata === expectedOUT) begin
                $display("[PASS] output AND is %b or %d as expected\n", outdata, expectedOUT);
            end else begin
                $error("[FAIL] out: %b or %d\n", outdata, outdata);
                $error("expectedOUT: %b or %d\n", expectedOUT, expectedOUT);
            end
            $display("------------------------------------------\n");
        end
        else if(DUT.opcode === 3'b101 && DUT.ALU_op === 2'b11)begin
            $display("Instruction: MVN Rd, Rm \n");
            n = 0;
            while(n < 3) begin
                #10;
                n++;
            end
            assert(outdata === expectedOUT) begin
                $display("[PASS] output MVN is %b or %d as expected\n", outdata, expectedOUT);
            end else begin
                $error("[FAIL] out: %b or %d\n", outdata, outdata);
                $error("expectedOUT: %b or %d\n", expectedOUT, expectedOUT);
            end
            $display("------------------------------------------\n");
        end
        else if(DUT.opcode === 3'b011 && DUT.ALU_op === 2'b00)begin
            $display("Instruction: LDR Rd, [Rn, im5] \n");

            n = 0;
            while(n < 5) begin
                #10;
                n++;
            end
            assert(outdata === expectedOUT) begin
                $display("[PASS] offset address of [Rn + im5] is %h as expected\n", outdata);
            end else begin
                $error("[FAIL] offset address: %h\n", outdata);
                $error("expectedOUT: %h\n", expectedOUT);
            end
            $display("------------------------------------------\n");
        end
        else if(DUT.opcode === 3'b100 && DUT.ALU_op === 2'b00)begin
            $display("Instruction: STR Rd, [Rn, im5] \n");

            check_STR_OFFSET_ADDR(16'd12);

            n = 0;
            while(n < 4) begin
                #10;
                n++;
            end
            assert(outdata === expectedOUT) begin
                $display("[PASS] write value of Rd is %b as expected\n", outdata);
            end else begin
                $error("[FAIL] value of Rd: %b\n", outdata);
                $error("expectedOUT: %b\n", expectedOUT);
            end
            $display("------------------------------------------\n");
        end
        else if(DUT.opcode === 3'b111)begin
            $display("Instruction: HALT\n");
            n = 0;
            while(n < 10)begin
                #10;

                assert(outdata === expectedOUT) begin
                    $display("[PASS] output is still %b at time cycle %d\n", outdata, n);
                end else begin
                    $error("[ERROR] output is NOT/has changed to %b at time cycle %d\n", outdata, n);
                    $error("while expecting %b\n", expectedOUT);
                end

                n++;
            end
            $display("------------------------------------------\n");
            
        end
        else begin
            $display("Instruction unknown: \n");
            $display("opcode: %b\n", DUT.opcode);
            $display("ALU_op: %b\n", DUT.ALU_op);

        end
    end
endtask

task check_STR_OFFSET_ADDR;
    input [15:0] expected_offset_addr;
    begin
        #20;
        assert(expected_offset_addr === outdata) begin
            $display("[PASS] offset address of [Rn + im5] is %h as expected\n", outdata);
        end else begin
            $error("[FAIL] offset address: %h\n", outdata);
            $error("expected_offset_addr: %h\n", expected_offset_addr);
        end
    end
endtask

task checkBranching;
    input [7:0] expected_ram_addr;
    begin
        assert(DUT.opcode === 3'b001 && DUT.ALU_op === 2'b00 && DUT.cond === 3'b000) begin
            $display("Instruction: B\n");
            #10;
            assert(DUT.ram_addr === expected_ram_addr) begin
                $display("[PASS] ram_addr has jumped to %h as expected", expected_ram_addr);
            end else begin
                $error("[FAIL] ram_addr has jumped to %h instead of %h", DUT.ram_addr, expected_ram_addr);
            end
        end
    end
endtask

initial begin
    start_pc = 8'd0;
    rst_n = 1'b0;
    #12;
    rst_n = 1'b1;
    #10;
    #10;
    #10;

    checkOutput(`ANY16BITS, 1'b0, 1'b0);
    #30;
    checkOutput(`ANY16BITS, 1'b0, 1'b0);
    #30;
    checkOutput(16'd2, 1'b0, 1'b0);
    #30;
    checkOutput(`ANY16BITS, `NO, `NO);
    #30;
    checkOutput(16'd0, 1'b0, 1'b0);
    #30;
    checkOutput(16'b1111111111111101, 1'b0, 1'b0);
    #30;
    checkOutput(16'd10, 1'b0, 1'b0);
    #30;
    checkOutput(16'd2, 1'b0, 1'b0);
    #30;
    checkOutput(16'd2, 1'b0, 1'b0);
    #30;

    start_pc = 8'd16;
    rst_n = 1'b0;
    #10;
    rst_n = 1'b1;
    #10;
    #10;
    #10;
    checkOutput(`ANY16BITS, 1'b0, 1'b0);
    #30;
    checkOutput(`ANY16BITS, 1'b0, 1'b0);
    #30;
    checkOutput(16'd66, 1'b0, 1'b0);
    #30;
    checkOutput(`ANY16BITS, `NO, `YES);
    #30;
    checkOutput(16'd0, 1'b0, 1'b0);
    #30;
    checkOutput(16'd65502, 1'b0, 1'b0);
    #30;
    checkBranching(8'd27);



    $stop;
end


endmodule: tb_task1
