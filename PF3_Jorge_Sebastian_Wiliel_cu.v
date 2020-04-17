module main;
//main is used strictly for testing purposes
    
parameter sim_time = 600;
reg Clk;

//====
//section for module instances
//====

//simulation time
initial #sim_time $finish;

//manejar clock
initial begin
  Clk <= 1'b1;
  repeat(100) #10 Clk = ~Clk;
end

initial begin
  $display("Control Unit Test - Jorge Vega | Sebastian Merced | Wiliel Florenciani \n");
end

initial #1 begin
  $display("Signals to be tested\n");

  $display("Clock  Load   Mux1    Mux7   Time");
  $display("--------------------------------------------------------------------------------");
  $monitor("%b, %d",Clk,$time);
end

    
endmodule

module inverter(output reg out, input in, inv);
    always @ (*)
    begin
        case(inv)
            1'b0: out <= in;
            1'b1: out <= ~in;
        endcase
        end
endmodule

// multiplexer4x2
module Multiplexer7_4x2(output reg [6:0] out, input [6:0] I0, I1, I2, I3, input [1:0] S);

    always @ (*)
    begin
        case(S)
            2'h0: out <= I0;
            2'h1: out <= I1;
            2'h2: out <= I2;
            2'h3: out <= I3;
        endcase
        end
    
endmodule 

module Multiplexer1_4x2(output reg out, input I0, I1, I2, I3, input [1:0] S);

    always @ (*)
    begin
        case(S)
            2'h0: out <= I0;
            2'h1: out <= I1;
            2'h2: out <= I2;
            2'h3: out <= I3;
        endcase
        end
    
endmodule 

module NextStateAddressSelector(output reg [1:0] M, input Sts, input [2:0] N);

    always @ (*)
    begin
        case(N)
            3'o0: M <= 2'b00; //Encoder
            3'o1: M <= 2'b01; // ?
            3'o2: M <= 2'b10; //Control Register
            3'o3: M <= 2'b11; // Incrementer
            3'o4: begin 
                    M[1] = ~Sts;
                    M[0] = 1'b0;
                  end
            3'o5: begin 
                    M[1] = 1'b0;
                    M[0] = ~Sts;
                  end
            3'o6: begin 
                    M[1] = ~Sts;
                    M[0] = ~Sts;
                  end
            3'o7: M <= 2'b00;
        endcase
        end
endmodule

module ControlRegister(output reg [31:0] Qs, input Clk, input [31:0] Ds); //32b bus, 20 moore lines, 6 CR and 6 NSAS
  always @ (posedge Clk) begin
   Qs <= Ds;
   //$display("Output of control register is %b", Qs);
end
endmodule

module Microstore (output reg [31:0] Out, input Clk, input wire [5:0] Address);
    reg [31:0] Mem[0:65];
    // always @ (posedge Clk, Address) begin
    always @ (posedge Clk) begin
        //#2 
        Out[31:0] = Mem[Address]; 
        //$display("Microstore output: %b", Out); 
    end

    initial begin //n2n1n0 inv s1s0 moore cr(6)

        Mem[0] = 32'b01100001000000011011001101000000; //reset 
        Mem[1] = 32'b01100000010001000001010000000000; 
        Mem[2] = 32'b01100001000111000011010001000000; 
        Mem[3] = 32'b10110000100110000000000000000011;
        Mem[4] = 32'b10010100000000000000000000000001; 

        // //ADD shift
        // Mem[5] = 32'b010000 01000000001000000000 000001; //done
        
    //     //ADD R-R
    //     Mem[6] = 32'b000000000000000000000000100000000000010000000100011000;
        
    //     //ADD imm
    //     Mem[7] = 32'b000000000000000001001000001001011000010000000100011100;
        
    //     //STRB imm offset ADD
    //     Mem[8] = 32'b000000000000000100100000100000000000010000000100100000;
    //     Mem[9] = 32'b000000000000000000000100001001001000010000000100100100;
    //     Mem[10] = 32'b001000010000000100010000100000000100010000000100101000;
    //     Mem[11] = 32'b000000000000000000000110001001001000010000000100101100;

    //     //STRB imm offset SUB
    //     Mem[12] = 32'b000001001000010100100000000000000000010001001000110000;
    //     Mem[13] = 32'b001000001000100000010010000000000000010001100000110100;
    //     Mem[14] = 32'b000000000111100000000000010000000000010001001100111000;
    //     Mem[15] = 32'b0000000001110000000000000010000000010001001100111100;
        
    //     //STRB reg offset ADD
    //     Mem[16] = 32'b0000000000110000000000000010000000010001001101000000;
    //     Mem[17] = 32'b0000000001110000000000000000000000010001001101000100;
    //     Mem[18] = 32'b000000000011000000000000000000000000011000000001001000;
    //     Mem[19] = 32'b000000000011000000000000000000000000101101001101001100;
        
    //     //STRB reg offset SUB
    //     Mem[20] = 32'b001000010100001000100010100000000000010000000101010000; 
    //     Mem[21] = 32'b0000000000000000000000000000000000000000000000000; 
    //     Mem[22] = 32'b000000000101000000000000010000000000010000000101011000;
    //     Mem[23] = 32'b000000000001000000000000000010000000010001100101011000;
        
    //     //STRB imm pre ADD
    //     Mem[24] = 32'b000000000001000000000000000000000000010000000101100000;
    //     Mem[25] = 32'b000000000001000000000000000000000000101101100101100100;
    //     Mem[26] = 32'b000000000000000000000000000000000000010000000101101000;
    //     Mem[27] = 32'b000000000000000000000000001001000010010000000101101100;
    //     Mem[28] = 32'b000000000000000000000000001001001000010000000101110010;
        
    //     //STRB imm pre SUB
    //     Mem[29] = 32'b000000000000000000000000001001001000010000000101110101;
    //     Mem[30] = 32'b000000000000000000010000000000000000010000000101111000;
    //     Mem[31] = 32'b000000000000000001000000000000000000010000000101111100;
    //     Mem[32] = 32'b000000000000000000001000001000001000010010001010000000;
    //     Mem[33] = 32'b000000000000000000000000001000001000010010001010000100;
       
    //    //STRB Reg pre ADD
    //     Mem[34] = 32'b000000000000000000000000001000001000010010010110001000;
    //     Mem[35] = 32'b001000011000000000000000000001000000101100000110001101;
    //     Mem[36] = 32'b011000100000000100000000000010000001010000000110010000;
    //     Mem[37] = 32'b000000000000000000000000000001100110010000000110010100;
    //     Mem[38] = 32'b001001001000010100100000000000000000010000110110011000;

    //    //STRB Reg pre SUB 
    //     Mem[39] = 32'b001111001000000000000000000001000000101100000110011101;
    //     Mem[40] = 32'b011000100000000100000000000010000001010000000110100000;
    //     Mem[41] = 32'b001001001000010100100000000000000000010000111010100100; 
    //     Mem[42] = 32'b001001001000010100100000010000000000011000000010101000; 
    //     Mem[43] = 32'b001000001000100000010010000000000000010001011010101100; 
       
    //    //STRB imm post ADD
    //     Mem[44] = 32'b001000101000000000000000000010000001010000000110110000;
    //     Mem[45] = 32'b111100110000000000000000100010000000010010010010110100;
    //     Mem[46] = 32'b000000000000000011000000000000000000010000000110111000;
    //     Mem[47] = 32'b000000000000000000000000100000010000010000000110111100;
    //     Mem[48] = 32'b000000000000000000000000100000100000010000000111000000;
        
    //    //STRB imm post SUB
    //     Mem[49] = 32'b000000000000000001000000000000000000010000000111000100;
    //     Mem[51] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[52] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[53] = 32'b000000000000000010000000000000000000010000000111001000;
        
    //    //STRB Reg post ADD
    //     Mem[54] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[55] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[56] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[57] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[58] = 32'b000000000000000010000000000000000000010000000111001000;
        
    //    //STRB Reg post SUB
    //     Mem[59] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[60] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[61] = 32'b000000000000000010000000000000000000010000000111001000;
    //     Mem[63] = 32'b000000000000000010000000000000000000010000000111001000;

    //    //Break
    //     Mem[64] = 32'b000000000000000010000000000000000000010000000111001000;
    end
endmodule

module IncrementerRegister(output reg [6:0] Q, input [6:0] D, input  Clk);

always @(posedge Clk)

Q <= D;
endmodule

module Adder(output reg [6:0] out, input [6:0] in);

always@(in)

out <= in + 1;
endmodule

module InstructionRegister(output reg [31:0] Q, input [31:0] D, input LE, Clk);

always @(posedge Clk)

if(LE) Q <= D;
endmodule

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

// module register32bit(output reg [31:0] Q, input [31:0] D, input Clk, ld);
// initial Q <= 32'd0;
//   always @ (posedge Clk)
//   if(ld) Q <= D;
// endmodule