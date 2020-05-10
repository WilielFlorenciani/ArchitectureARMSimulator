//Testing Condition Tester

module Testing;
wire Cond;
reg Z, C, N, V;
reg [3:0] CC;

wire [3:0] encOut;
reg [31:0] encIn, shifterIn;
wire [31:0] shifterOut;

// ConditionTester condTest (Cond, Z, C, N, V, CC);
MultiRegEncoder encoder (encOut, encIn);
DecInstrShifter shifter (shifterOut, shifterIn);

// initial begin
//   $display("~~~~~ Initiating Encoder Test ~~~~~");
//   $monitor("Out:%b, In:%b", encOut, encIn);
// end

initial begin
  $display("~~~~~ Initiating Shifter Test ~~~~~");
  $monitor("Out:%b, In:%b", shifterOut, shifterIn);
end

initial begin
// encIn = 32'b1000_0000_0000_0000_0000_0000_0000_0000;
shifterIn = {32'b0000_0000_0100_0000, 16'b0};
end

// initial begin
//   $display("~~~~~ Initiating Condition Tester Test ~~~~~");
//   $monitor("Cond:%b, Z:%b, C:%b, N:%b, V:%b, CC:%b", Cond, Z, C, N, V, CC);
// end

// initial begin 
//   CC = 4'b1110;
//   Z = 1'b1;
//   C = 1'b0;
//   N = 1'b1;
//   V = 1'b1;
// end

endmodule

module DecInstrShifter(output reg [31:0] out, input [31:0] in);
always @(in)
    out <= in << 2;
endmodule


////////////// BEGIN ENCODERMULTI
module MultiRegEncoder(output reg [3:0] Out, input [31:0] RegisterBit);
always @(RegisterBit) begin
case(RegisterBit)
    32'h10000: begin
        Out <= 4'h0;
    end
    32'h20000: begin
        Out <= 4'h1;
    end
    32'h40000: begin
        Out <= 4'h2;
    end
    32'h80000: begin
        Out <= 4'h3;
    end
    32'h100000: begin
        Out <= 4'h4;
    end
    32'h200000: begin
        Out <= 4'h5;
    end
    32'h400000: begin
        Out <= 4'h6;
    end
    32'h800000: begin
        Out <= 4'h7;
    end
    32'h1000000: begin
        Out <= 4'h8;
    end
    32'h2000000: begin
        Out <= 4'h9;
    end
    32'h4000000: begin
        Out <= 4'hA;
    end
    32'h8000000: begin
        Out <= 4'hB;
    end
    32'h10000000: begin
        Out <= 4'hC;
    end
    32'h20000000: begin
        Out <= 4'hD;
    end
    32'h40000000: begin
        Out <= 4'hE;
    end
    32'h80000000: begin
        Out <= 4'hF;
    end
endcase
end
endmodule
////////////// END ENCODERMULTI


////////////// BEGIN CONDITION TESTER
// module ConditionTester (output reg Cond, input C, Z, N, V, input [3:0] CC);
// always @ (*) begin
// case(CC)
//     4'h0: begin
//         Cond <= Z; //EQ Equal
//     end
//     4'h1: begin
//         Cond <= ~Z; //NE Not equal
//     end
//     4'h2: begin
//         Cond <= C; //CS/HS Unsigned higher or same
//     end
//     4'h3: begin
//         Cond <= ~C; //CC/LO Unsigned lower
//     end
//     4'h4: begin
//         Cond <= N; //MI Mius
//     end
//     4'h5: begin
//         Cond <= ~N; //PL Positive or Zero
//     end
//     4'h6: begin
//         Cond <= V; //VS Overflow
//     end
//     4'h7: begin
//         Cond <= ~V; //VC No overflow
//     end
//     4'h8: begin
//         Cond <= C & ~Z; //HI Unsigned higher //test this, might be &&?
//     end
//     4'h9: begin
//         Cond <= ~C | Z; //LS Unsigned lower or same
//     end
//     4'hA: begin
//         Cond <= ~(N ^ V); //GE Greater or equal
//     end
//     4'hB: begin
//         Cond <= N ^ V; //LT Less than
//     end
//     4'hC: begin
//         Cond <= ~Z & (~(N ^ V)); //GT Greater than
//     end
//     4'hD: begin
//         Cond <= Z | (~(N ^ ~V)); //LE Less than or eual
//     end
//     4'hE: begin
//         Cond <= 1'b1; //AL Always
//     end
// endcase
// end
// endmodule
////////////// END CONDITION TESTER




// //File that holds the Microstore, Control Register and Mux modules

// module test;

// //Microstore
// reg [5:0] nextState; //output from mux, input into microstore (this is a dummy variable for testing, though)
// reg Clk;
// wire [31:0] microOutput;

// Microstore micro (microOutput, Clk, muxOut); //declaring a microstore instance
// // Microstore micro (microOutput, Clk, nextState);

// //Mux
// reg M1, M0;
// wire [5:0] muxOut;
// reg [5:0] Incr, Encoder, Q, CtrReg;

// Mux4x1 mux (muxOut, M0, M1, Encoder, Q, CtrReg, Incr);

// //Control Register
// wire [31:0] crOut;
// reg [31:0] crIn; //dummy variable for testing

// ControlRegister cr (crOut, Clk, microOutput);


// //Clock
// initial begin
//   Clk <= 1'b0;
//   $display("Initial Clock value: %b, time: %d", Clk, $time);
//   repeat(10) #10 Clk = ~Clk;
// end

// always @ (posedge Clk) begin
// $display("Next Clock value: %b, Incr = %b, muxOut = %b, time: %d", Clk, Incr, muxOut, $time);
// end

// initial begin
//   $display("---- Testing State Cycles ----");
//   M1 <= 1; M0 <= 1;  //4'b0011 = 3 = Incrementer selection from mux
//   Incr <= 1;
//   #2
//   repeat (3) #1 Incr = muxOut + 6'b000001;
// end

// // initial begin
// //   $monitor("Ctrl Reg output: %b \n  Incr = %b, muxOut = %b time: %d", crOut, Incr, muxOut, $time);
// // end

// always @ (crOut) begin
// $display("Ctrl Reg output: %b \n  Incr = %b, muxOut = %b time: %d", crOut, Incr, muxOut, $time);
// end

// // initial begin 
// //     //#2;
// //     $display("----- Testing Microstore -----");
// //     #10 nextState = 2;
// //     #30 nextState = 1; 
// //     #50 nextState = 2;
// //     #70 nextState = 0;

// // end

// // initial begin
// //   $display("---- Testing Mux ----");
// //   M1 = 1; M0 = 1;
// //   $display("Mux output is: %b", muxOut);
// // end

// // initial begin
// //   $display("---- Testing Control Register ----");
// //   #20 crIn = 2;
// //   $display("CR output is: %b", crOut);
// //   #30 crIn = 1;
// //   $display("CR output is: %b", crOut);
// //   #40 crIn = 3;
// //   $display("CR output is: %b", crOut);
// // end
// endmodule

// ///////////////////////////////////////// Ends Test Module ///////////////////////////

// module Microstore (output reg [31:0] Out, input Clk, input wire [5:0] Address);
//     reg [31:0] Mem[0:65];
//     // always @ (posedge Clk, Address) begin
//     always @ (posedge Clk) begin
//         //#2 
//         Out[31:0] = Mem[Address]; 
//         //$display("Microstore output: %b", Out); 
//     end

//     initial begin //n2n1n0 inv s1s0 moore cr(6)

//         Mem[0] = 32'b01100001000000011011001101000000; //reset 
//         Mem[1] = 32'b01100000010001000001010000000000; 
//         Mem[2] = 32'b01100001000111000011010001000000; 
//         Mem[3] = 32'b10110000100110000000000000000011;
//         Mem[4] = 32'b10010100000000000000000000000001; 

//         // //ADD shift
//         // Mem[5] = 32'b010000 01000000001000000000 000001; //done
        
//     //     //ADD R-R
//     //     Mem[6] = 32'b000000000000000000000000100000000000010000000100011000;
        
//     //     //ADD imm
//     //     Mem[7] = 32'b000000000000000001001000001001011000010000000100011100;
        
//     //     //STRB imm offset ADD
//     //     Mem[8] = 32'b000000000000000100100000100000000000010000000100100000;
//     //     Mem[9] = 32'b000000000000000000000100001001001000010000000100100100;
//     //     Mem[10] = 32'b001000010000000100010000100000000100010000000100101000;
//     //     Mem[11] = 32'b000000000000000000000110001001001000010000000100101100;

//     //     //STRB imm offset SUB
//     //     Mem[12] = 32'b000001001000010100100000000000000000010001001000110000;
//     //     Mem[13] = 32'b001000001000100000010010000000000000010001100000110100;
//     //     Mem[14] = 32'b000000000111100000000000010000000000010001001100111000;
//     //     Mem[15] = 32'b0000000001110000000000000010000000010001001100111100;
        
//     //     //STRB reg offset ADD
//     //     Mem[16] = 32'b0000000000110000000000000010000000010001001101000000;
//     //     Mem[17] = 32'b0000000001110000000000000000000000010001001101000100;
//     //     Mem[18] = 32'b000000000011000000000000000000000000011000000001001000;
//     //     Mem[19] = 32'b000000000011000000000000000000000000101101001101001100;
        
//     //     //STRB reg offset SUB
//     //     Mem[20] = 32'b001000010100001000100010100000000000010000000101010000; 
//     //     Mem[21] = 32'b0000000000000000000000000000000000000000000000000; 
//     //     Mem[22] = 32'b000000000101000000000000010000000000010000000101011000;
//     //     Mem[23] = 32'b000000000001000000000000000010000000010001100101011000;
        
//     //     //STRB imm pre ADD
//     //     Mem[24] = 32'b000000000001000000000000000000000000010000000101100000;
//     //     Mem[25] = 32'b000000000001000000000000000000000000101101100101100100;
//     //     Mem[26] = 32'b000000000000000000000000000000000000010000000101101000;
//     //     Mem[27] = 32'b000000000000000000000000001001000010010000000101101100;
//     //     Mem[28] = 32'b000000000000000000000000001001001000010000000101110010;
        
//     //     //STRB imm pre SUB
//     //     Mem[29] = 32'b000000000000000000000000001001001000010000000101110101;
//     //     Mem[30] = 32'b000000000000000000010000000000000000010000000101111000;
//     //     Mem[31] = 32'b000000000000000001000000000000000000010000000101111100;
//     //     Mem[32] = 32'b000000000000000000001000001000001000010010001010000000;
//     //     Mem[33] = 32'b000000000000000000000000001000001000010010001010000100;
       
//     //    //STRB Reg pre ADD
//     //     Mem[34] = 32'b000000000000000000000000001000001000010010010110001000;
//     //     Mem[35] = 32'b001000011000000000000000000001000000101100000110001101;
//     //     Mem[36] = 32'b011000100000000100000000000010000001010000000110010000;
//     //     Mem[37] = 32'b000000000000000000000000000001100110010000000110010100;
//     //     Mem[38] = 32'b001001001000010100100000000000000000010000110110011000;

//     //    //STRB Reg pre SUB 
//     //     Mem[39] = 32'b001111001000000000000000000001000000101100000110011101;
//     //     Mem[40] = 32'b011000100000000100000000000010000001010000000110100000;
//     //     Mem[41] = 32'b001001001000010100100000000000000000010000111010100100; 
//     //     Mem[42] = 32'b001001001000010100100000010000000000011000000010101000; 
//     //     Mem[43] = 32'b001000001000100000010010000000000000010001011010101100; 
       
//     //    //STRB imm post ADD
//     //     Mem[44] = 32'b001000101000000000000000000010000001010000000110110000;
//     //     Mem[45] = 32'b111100110000000000000000100010000000010010010010110100;
//     //     Mem[46] = 32'b000000000000000011000000000000000000010000000110111000;
//     //     Mem[47] = 32'b000000000000000000000000100000010000010000000110111100;
//     //     Mem[48] = 32'b000000000000000000000000100000100000010000000111000000;
        
//     //    //STRB imm post SUB
//     //     Mem[49] = 32'b000000000000000001000000000000000000010000000111000100;
//     //     Mem[51] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[52] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[53] = 32'b000000000000000010000000000000000000010000000111001000;
        
//     //    //STRB Reg post ADD
//     //     Mem[54] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[55] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[56] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[57] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[58] = 32'b000000000000000010000000000000000000010000000111001000;
        
//     //    //STRB Reg post SUB
//     //     Mem[59] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[60] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[61] = 32'b000000000000000010000000000000000000010000000111001000;
//     //     Mem[63] = 32'b000000000000000010000000000000000000010000000111001000;

//     //    //Break
//     //     Mem[64] = 32'b000000000000000010000000000000000000010000000111001000;
//     end
// endmodule

// module Mux4x1 (output reg [5:0] Out, input M0, M1, input [5:0] I0, I1, I2, I3);
//   reg [1:0] selection;
//   always @(M0, M1, I0, I1, I2, I3) begin

//     selection[0] = M0;
//     selection[1] = M1;

//     case (selection)
//       0: begin
//         Out = I0;
//       end
//       1: begin
//         Out = I1;
//       end
//       2: begin 
//         Out = I2;
//       end
//       3: begin
//         Out = I3;
//       end
//     endcase
//     //$display("Selection is: %b, Mux output is: %d", selection, Out);
// end
// endmodule

// module ControlRegister(output reg [31:0] Qs, input Clk, input [31:0] Ds); //32b bus, 20 moore lines, 6 CR and 6 NSAS
//   always @ (posedge Clk) begin
//    Qs <= Ds;
//    //$display("Output of control register is %b", Qs);
// end
// endmodule