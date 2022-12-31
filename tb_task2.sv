module tb_task2(output err);


`define YES 1'b1
`define NO 1'b0
 
reg clk;
reg rst_n;
reg [7:0] start_pc;
wire [15:0] out;

task2 DUT(clk, rst_n, start_pc, out);

reg is_err;
int fail_count;
int pass_count;

assign err = is_err;

initial begin
  clk <= 1'b0;
  forever #5 clk <= ~clk;
end

task check_output;
    input [5:0] num_cycles;
    input [15:0] expected_out;
    input canCheckOutput;
    begin
        
        if(canCheckOutput)begin
            assert(expected_out === out)begin
                $display("[PASS] output is %b and expected output is %b", out, expected_out);
                pass_count++;
            end else begin
                $error("[FAIL] output is %b but expected output is %b", out, expected_out);
                fail_count++;
            end
        end
        else begin
            $display("Current datapath_out is: %b and datapath_out shouldn't have updated", out);
        end
    end


endtask

task check_Halt;

    reg [15:0] savedOut;

    begin

        savedOut = out;

        #50;

        assert(savedOut === out)begin
            $display("[PASS] Output has not been changed for 50 ticks!");
            pass_count++;
        end else begin
            $error("[FAIL] Output has been changed to %b from %b after 50 ticks!", out, savedOut);
            fail_count++;
        end

    end

endtask

task waiting_for_waiting;
begin

n = 0;
while(n <= 5 || ~waiting) begin
  #10;
  n++;
end
end
endtask


initial begin

fail_count = 0;
pass_count = 0;

start_pc = 8'd7 //assuming that MOV is in address 0 in ram
#12;
// start_pc is loaded into PC register
#10; 
//ram_r_addr is updated with start_pc

#10; 
//reads data from address start_pc by instruction reg 

rst_n = 1'b0;

#10;
//now we are in idle state

rst_n = 1'b1;
#10;

//MOV#1   #23
check_output(6'd3, 16'd0, `NO); //from pre-fetching, MOV has 3 states
//idle -> writeRF -> idle 

//MOV#2   #30
check_output(6'd3, 16'd0, `NO);

//ADD
check_output(6'd6,16'd53, `YES);

//CMP
check_output(6'd6, 16'd0, `NO);

//AND
check_output(6'd6, 16'd9062, `YES);

//MVN d30 (0000000000011110) -> d65505 (1111111111100001)
check_output(6'd5, 16'd65505, `YES);

//LDR
check_output(6'd6, 16'd0, `NO);

//HALT
check_Halt; 
   
if(fail_count != 0) is_err = 1'b1;
else is_err = 1'b0;

$display("err = %b, %d cases passed, %d cases failed", is_err, pass_count, fail_count);

end
endmodule: tb_task2
