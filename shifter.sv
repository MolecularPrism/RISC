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
       // shift_out = shift_in << 1; //left shift
       // temp1 = 0;
       // temp2 = 16'b0000000000000000;

        shift_out = shift_in; //no shift
        temp1 = 0;
        temp2 = 16'b0000000000000000;
      end
      2'b10: begin 
       // shift_out = shift_in >> 1; //logical right shift
       // temp1 = 0;
       // temp2 = 16'b0000000000000000;

        shift_out = shift_in; //no shift
        temp1 = 0;
        temp2 = 16'b0000000000000000;
      end
      2'b11:begin 
           //   temp1 = shift_in[15];
            //  temp2 = shift_in >> 1; //arithmatic right shift
            //  shift_out = {temp1, temp2[14:0]};

            shift_out = shift_in; //no shift
        temp1 = 0;
        temp2 = 16'b0000000000000000;
            end
      default: shift_out = shift_in;
    endcase
  end

endmodule: shifter
