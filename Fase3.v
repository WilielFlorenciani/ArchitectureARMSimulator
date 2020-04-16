module IncrementerRegister(output reg [6:0] Q, input [6:0] D, input  Clk);

always @(posedge Clk)

Q <= D;
endmodule
//---------------------------//-----------------------//
module Adder(output reg [6:0] out, input [6:0] in);

always@(in)

out <= in + 1;
endmodule
//---------------------------//-----------------------//
module InstructionRegister(output reg [31:0] Q, input [31:0] D, input LE, Clk);

always @(posedge Clk)

if(LE) Q <= D;
endmodule
//------------------------//--------------------------//
module IREncoder(output reg [6:0] Out, input [31:0] Instruction);

always @(Instruction)
begin
// Instruccion ADD immediate
if ((Instruction[27:25]== 3'b001) && (Instruction[24:21] == 4'b0100)) 
    Out= 7'b0000111;
//Instruccion ADD shift 
else if((Instruction[27:25]== 3'b000) && (Instruction[24:21] == 4'b0100) && (Instruction[4] == 1'b0))
    Out = 7'b0000101;
//Instruccion ADD R-R
else if ((Instruction[27:25]== 3'b000) && (Instruction[24:21] == 4'b0100) && (Instruction[11:5]== 7'b0000000))
    Out = 7'b0000110;
//Instruction STRB immediate offset ADD
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b11100))
    Out = 7'b0001000;
//Instruction STRB immediate offset SUB
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b10100))
    Out = 7'b0001100;
//Instruction STRB Register offset ADD
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b11100) && (Instruction[11:4]== 8'b00000000))
    Out = 7'b0010000;
//Instruction STRB Register offset SUB
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b10100) && (Instruction[11:4]== 8'b00000000))
    Out = 7'b0010100;
//Instruccion STRB immediate pre-index Add
else if ((Instruction[27:25] == 3'b010) && (Instruction[24:20]== 5'b11110))
    Out = 7'b0011000;
//Instruccion STRB immediate pre-index Sub
else if((Instruction[27:25] == 3'b010) && (Instruction[24:20]== 5'b10110))
    Out = 7'b0011101;
//Instruccion STRB Register pre-index Add
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b11110) && (Instruction[11:4]== 8'b00000000))
    Out = 7'b0100010;
//Instruccion STRB Register pre-index Sub
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b10110) && (Instruction[11:4]== 8'b00000000))
    Out = 7'b0100010;
//Instruccion STRB immediate post-index ADD
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b01100))
    Out = 7'b0101100;
//Instruction STRB immediate post-index SUB
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b00100))
    Out = 7'b0110001;
//Instruction STRB Register post-index ADD
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b01100) && (Instruction[11:4]== 8'b00000000))
    Out = 7'b0110110;
//Instruction STRB Register post-index SUB
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b00100) && (Instruction[11:4]== 8'b00000000))
    Out = 7'b0111011;
//Instruction Branch
else if(Instruction[27:25]== 3'b101)
    Out = 7'b1000000;
end
endmodule

module test;

parameter sim_time = 1500;
reg clk;
wire [6:0] EncOut;
reg [31:0] EncIn;
wire [31:0] Instruction;
reg LE;

IREncoder encoder(EncOut, EncIn);
//InstructionRegister register(Instruction,EncIn,LE,clk);


initial #sim_time $finish;

//manejar clock
initial begin
  clk = 1'b1;
  repeat(100) #10 clk = ~clk;
  EncIn = 32'b1110_000_0100_0_0110_0100_00001110_0100;
end

//initial fork
//LE = 1'b0; #5 LE = 1'b1; #100 EncIn = 32'b0000_000_0000_0_0000_0000_00000000_0000;
//join

initial #1 begin
  $display("Signals to be tested\n");

  $display("NextState   Time    Instruction");
  $display("--------------------------------------------------------------------------------");
  $monitor("%d, %0d",EncOut,$time);
end

    
endmodule