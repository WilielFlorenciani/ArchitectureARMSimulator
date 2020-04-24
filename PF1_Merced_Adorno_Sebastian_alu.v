
module alu_32 (output reg [31:0] Out, output reg Carry,Zero,Neg,Vflow,
input [31:0] A,B, input [4:0] Sel, input Cin);

always @(*)
    begin
    Out = 32'b0;
    Carry = 1'b0;
    Zero = 1'b0;
    Neg = 1'b0;
    Vflow = 1'b0;
    
    case(Sel)
    //Arithmetic Operations
    5'b00000: Out = A & B;
    5'b00001: Out = A ^ B;
    5'b00010: Out = A - B;
    5'b00011: Out = B - A;
    5'b00100:  {Carry,Out} = A + B; //suma    
    5'b00101:  {Carry,Out} = A + B + Cin; //suma con carry
    5'b00110:   Out = A - B - (!Cin);
    5'b00111:   Out = B - A - (!Cin);
   
   //Update Flags
    5'b01000:  Out = A & B; //bitwise and   
    5'b01001:  Out = A ^ B; // xor
    5'b01010:  Out = A - B;  //resta
    5'b01011:   Out = A + B;
    5'b01100:   Out = A | B;
    5'b01101:   Out = B;
    5'b01110:   Out = A & (!B);
    5'b01111:   Out = !B;
    5'b10000:   Out = A;
    5'b10001:   Out = A + 4;
    5'b10010: Out = A + B + 4;
    5'b10011:  Out = A >>1; // right shift
    5'b10100:  Out = A <<1; // left shift

    
    endcase
    
     Zero = (~|Out ); //bitwise or
    
     Neg = (Out[31] == 1);
    
     Vflow = ((~Out[31]&A[31]&B[31]) || (Out[31] & ~A[31] & ~B[31]));
    
    end
    

endmodule

module test;
 wire [31:0] out;
 wire C,Z,N,V;
 reg [4:0] Sel;
 reg [31:0] A, B;
 reg Cin;
 
 alu_32 alu (out, C, Z, N, V, A, B, Sel, Cin);
 
 initial #400 $finish;
 initial begin
 
 A = 32'hffffffff;      //7fffffff - la suma genera overflow
 B = 32'hffffffff;
 Cin=1'b0;
 Sel = 3'b000;
 
 repeat(20)#15 Sel = Sel + 5'b00001;
 end
 
 initial begin
    $display("                          Out        Out          A          B     C Z N V Sel");
    $monitor("%b %d %d %d  %b %b %b %b %b", out,out,A, B, C, Z, N, V, Sel);
    end
endmodule
 
