
module shift_sign_extender(output reg [31:0] extender_out, output reg carry, input [31:0] instruction, B, input Cin);

reg [7:0] temp;

always @(*) 
begin

    case (instruction[27:25])
    3'b001: 
        begin
            {temp} = instruction[7:0];
            {extender_out} = {temp,temp} >>(2*instruction[11:8]);
            if(instruction[11:8]==4'b0000)
                carry = Cin;
            else
                carry = extender_out[31];
        end 
    3'b000: 
    begin
        if(instruction[4]==0)
        begin
            if(instruction[6:5]==2'b00)
            begin
                if(instruction[11:7] == 5'b00000)
                begin
                    extender_out = B;
                    carry = Cin;
                end
                else
                    begin
                        extender_out = B << instruction[11:7];
                        carry = B[32-instruction[11:7]];
                    end
            end 

            else if(instruction[6:5]==2'b01)
                begin
                    if(instruction[11:7]==5'b00000)
                        begin
                            extender_out = 32'b0;
                            carry = B[31];
                        end
                    else
                        begin
                            extender_out = B >> instruction[11:7];
                            carry = B[instruction[11:7]-1];
                        end
                end
            else if(instruction[6:5]==2'b10)
                begin
                    if(instruction[11:7]==5'b00000) begin
                        if(B[31]==0)
                            begin
                                extender_out = 32'b0;
                                carry = B[31];
                            end
                        else
                            begin
                                extender_out = 32'hFFFFFFFF;
                                carry = B[31];
                            end
                    end
                    else        
                        begin
                        extender_out = $signed(B) >>> instruction[11:7];
                        carry = B[instruction[11:7]-1];
                        end
                end
            else if(instruction[6:5]== 2'b11)
                    begin
                         extender_out = {B, B} >> instruction[11:7];
                         carry = B[instruction[11:7]-1];
                    end
        end
        else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b10 && instruction[24]== 1'b1)       
                extender_out = (instruction[11:8]<<4) | instruction[3:0]; 

        else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b00 && instruction[24]== 1'b1)
                extender_out = B;

        else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b11 && instruction[24]== 1'b1)
               extender_out =  (instruction[11:8]<<4) | instruction[3:0];

        else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b01 && instruction[24]== 1'b1)
                extender_out = B;

        else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b10 && instruction[24]== 1'b0)   
                extender_out = (instruction[11:8]<<4) | instruction[3:0];

        else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b00 && instruction[24]== 1'b0)
                extender_out = B;
    end
    3'b101:  
        {extender_out} = {{8{instruction[23]}},instruction[23:0]} << 2;
    endcase
        

end           

//end 

endmodule


module test;

reg [31:0] inst;
reg cin;
wire [31:0] out;
wire carry;
reg [31:0] RM;

initial begin
   cin = 1'b0; 
   RM = 32'b00000000000000000000000000000011;
end

initial begin
 inst = 32'b00000000000001111111000000000000;
//10 inst = 32'b0000000101010000000000111001111;
end

shift_sign_extender sse(out,carry,inst,RM, cin);

initial begin
$display("Testing");
$monitor("out:%b    inst: %b   carry:%b", out, inst, carry);
end

endmodule
