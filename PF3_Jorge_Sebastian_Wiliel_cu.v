module main;
//main is used strictly for testing purposes
    
parameter sim_time = 800;


wire [32:0] CRout;
wire [31:0] IR;
reg [31:0] IRin;
reg Cond, MOC, clk, LE, reset;
wire [6:0] State;

InstructionRegister instruc_reg(IR, IRin, LE, clk);

ControlUnit cu(CRout, State, IR, Cond, MOC, reset, clk);

//simulation time
initial #sim_time $finish;

initial begin
    MOC = 1'b1;
    Cond = 1'b1;

    // if(State==3) begin
    // MOC <= 1'b0;
    // #4 MOC <= 1'b1;
    // end

    // MOC = 1'b1;
    // Cond = 1'b1;
end


//manejar clock
initial begin
  clk <= 1'b0;
  repeat(100) #5 clk = ~clk;
end

initial begin
  reset = 1'b1;
  #15 reset = ~reset;
end

initial begin
  LE <= 1'b1;
end

initial begin
  
  IRin <= 32'b1110_000_0100_0_0110_0100_00001110_0100; //estado 5 - ADD R4,R6,R4,ROR #?
  //#50 IRin <= 32'b11100000100001100100000011100100;
  //#100 IRin <= 32'b11100000100001100100000011100100;
 // #150 IRin <= 32'b1110000_0100001100100000011100100;
end


initial begin
  $display("Control Unit Test - Jorge Vega | Sebastian Merced | Wiliel Florenciani \n");
end

initial #1 begin
  $display("Signals to be tested\n");

  $display("CRout                                 IR                                   IRin                               State     Cond    MOC     reset   clk   Time");
  $display("----------------------------------------------------------------------------------------------------------------------------------------------");
  $monitor("%b     %b     %b     %d        %b     %b,     %b,      %b,   %0d",CRout,IR,IRin,State,Cond,MOC,reset,clk,$time);
end

    
endmodule



module ControlUnit(output [32:0] CRout, output reg [6:0] State, input [31:0] IR, input Cond, MOC, reset, clk);

wire [6:0] mux7Out;
wire mux1Out;
wire [6:0] AdderOut;
wire [6:0] EncoderOut;
wire [32:0] CRin;
wire invOut;
wire [6:0] incrementedState;
wire [1:0] M;
wire noValue = 0;

//If CRout[32:30] doesn't work
////////////////////
// reg [2:0] N;
// always@(CRout) begin
//   N[2] <= CRout[32];
//   N[1] <= CRout[31];
//   N[0] <= CRout[30];
// end

//If CRout[6:0] doesn't work
////////////////////
// reg [6:0] creg;
// always@(CRout) begin
//   creg[6] <= CRout[6];
//   creg[5] <= CRout[5];
//   creg[4] <= CRout[4];
//   creg[3] <= CRout[3];
//   creg[2] <= CRout[2];
//   creg[1] <= CRout[1];
//   creg[0] <= CRout[0];
// end

//If CRout[28:27] doesn't work
////////////////////
// reg [1:0] S;
// always@(CRout) begin
//   S[1] <= CRout[28];
//   S[0] <= CRout[27];
// end

always@(mux7Out) begin
    $display("State = %d", mux7Out);
    State = mux7Out;
end

ControlRegister control_register (CRout, clk, CRin);
Microstore microstore (CRin, clk, reset, mux7Out);
NextStateAddressSelector nsas (M, invOut, CRout[32:30]);
Inverter inv (invOut, mux1Out, CRout[29]);
Adder adder (AdderOut, mux7Out);
IncrementerRegister incr_reg (incrementedState, AdderOut, clk);
Multiplexer7_4x2 mux7_4x2 (mux7Out, EncoderOut, mux7Out, CRout[6:0], incrementedState, M, reset);
Multiplexer1_4x2 mux1_4x2 (mux1Out, MOC, Cond, noValue, noValue, CRout[28:27]); //aqui MOC tiene que ir en 0 y Cond en 1
Encoder encoder (EncoderOut, IR);

endmodule

module Inverter(output reg out, input in, inv);
    
    always @ (*)
    begin
        // $display("Inv - before changes ---- out %b,  in %b, inv %b", out,in,inv);
        case(inv)
            1'b0: out <= in;
            1'b1: out <= ~in;
        endcase
        // $display("Inv - after changes ---- out %b,  in %b, inv %b", out,in,inv);
        end
endmodule

// multiplexer4x2
module Multiplexer7_4x2(output reg [6:0] out, input [6:0] I0, I1, I2, I3, input [1:0] S, input reset);
    always @ (S, I3) begin
    //$display("MUX7 - changes ---- out %b,  I0 %b, I1 %b, I2 %b, I3 %b, S %b   time %0d", out, I0, I1, I2, I3, S, $time);
    end

    always @ (*)
    begin
        if(reset) 
        begin
            out = 0;
        end
        else begin
        // $display("Mux7 - before changes ---- out %b,  I0 %b, I1 %b, I2 %b, I3 %b, S %b", out,I0,I1,I2,I3,S);
        case(S)
            2'h0: out <= I0;
            2'h1: out <= I1;
            2'h2: out <= I2;
            2'h3: out <= I3;
        endcase
        // $display("Mux7 - after changes ---- out %b,  I0 %b, I1 %b, I2 %b, I3 %b, S %b", out,I0,I1,I2,I3,S);
        end
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
                    M[1] <= ~Sts;
                    M[0] <= 1'b0;
                  end
            3'o5: begin 
                    M[1] <= 1'b0;
                    M[0] <= ~Sts;
                  end
            3'o6: begin 
                    M[1] <= ~Sts;
                    M[0] <= ~Sts;
                  end
            3'o7: begin 
                    M[1] <= Sts;
                    M[0] <= ~Sts;
                  end
        endcase
        end
endmodule

module IncrementerRegister(output reg [6:0] Q, input [6:0] D, input  Clk);

always @(posedge Clk)

Q <= D;
endmodule
//---------------------------//-----------------------//
module Adder(output reg [6:0] out, input [6:0] in);

always @(in)

out <= in + 1'b1;
endmodule
//---------------------------//-----------------------//
module InstructionRegister(output reg [31:0] Q, input [31:0] D, input LE, Clk);

always @(posedge Clk)

if(LE) Q <= D;
endmodule
//------------------------//--------------------------//
module Encoder(output reg [6:0] Out, input [31:0] Instruction);

always @(Instruction)
begin
// Instruccion ADD immediate
if ((Instruction[27:25]== 3'b001) && (Instruction[24:21] == 4'b0100)) 
    Out <= 7'b0000111;
//Instruccion ADD shift 
else if((Instruction[27:25]== 3'b000) && (Instruction[24:21] == 4'b0100) && (Instruction[4] == 1'b0))
    Out <= 7'b0000101;
//Instruccion ADD R-R
else if ((Instruction[27:25]== 3'b000) && (Instruction[24:21] == 4'b0100) && (Instruction[11:5]== 7'b0000000))
    Out <= 7'b0000110;
//Instruction STRB immediate offset ADD
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b11100))
    Out <= 7'b0001000;
//Instruction STRB immediate offset SUB
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b10100))
    Out <= 7'b0001100;
//Instruction STRB Register offset ADD
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b11100) && (Instruction[11:4]== 8'b00000000))
    Out <= 7'b0010000;
//Instruction STRB Register offset SUB
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b10100) && (Instruction[11:4]== 8'b00000000))
    Out <= 7'b0010100;
//Instruccion STRB immediate pre-index Add
else if ((Instruction[27:25] == 3'b010) && (Instruction[24:20]== 5'b11110))
    Out <= 7'b0011000;
//Instruccion STRB immediate pre-index Sub
else if((Instruction[27:25] == 3'b010) && (Instruction[24:20]== 5'b10110))
    Out <= 7'b0011101;
//Instruccion STRB Register pre-index Add
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b11110) && (Instruction[11:4]== 8'b00000000))
    Out <= 7'b0100010;
//Instruccion STRB Register pre-index Sub
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b10110) && (Instruction[11:4]== 8'b00000000))
    Out <= 7'b0100010;
//Instruccion STRB immediate post-index ADD
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b01100))
    Out <= 7'b0101100;
//Instruction STRB immediate post-index SUB
else if((Instruction[27:25]== 3'b010) && (Instruction[24:20]== 5'b00100))
    Out <= 7'b0110001;
//Instruction STRB Register post-index ADD
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b01100) && (Instruction[11:4]== 8'b00000000))
    Out <= 7'b0110110;
//Instruction STRB Register post-index SUB
else if((Instruction[27:25]== 3'b011) && (Instruction[24:20]== 5'b00100) && (Instruction[11:4]== 8'b00000000))
    Out <= 7'b0111011;
//Instruction Branch
else if(Instruction[27:25]== 3'b101)
    Out <= 7'b1000000;
end
endmodule


module Microstore (output reg [32:0] Out, input Clk, input reset, input wire [6:0] Address);
    reg [32:0] Mem[0:64];
    // always @ (posedge Clk, reset, Address) begin
    always @ (*) begin
        //#2 
        //$display("Microstore - before changes ---- out %b,  Clk %b, reset %b, Address %b", Out, Clk, reset, Address);
        if (reset) begin
        Out <=  Mem[0]; 
        end
        else begin
        Out[32:0] <= Mem[Address]; 
        //$display("Microstore output: %b", Out);
        end
        //$display("Microstore - after changes ---- out %b,  Clk %b, reset %b, Address %b", Out, Clk, reset, Address); 
    end

    initial begin //n2n1n0 inv s1s0 moore cr(6)

        Mem[0] <= 33'b011000010000000110110011010000000; //reset 
        Mem[1] <= 33'b011000000100010000010100000000000; 
        Mem[2] <= 33'b011000010001110000110100010000000; 
        Mem[3] <= 33'b101100001001100000000000000000011;
        Mem[4] <= 33'b100001000000000000000000000000001; //100 inv Cond

        //ADD shift
        Mem[5] <= 33'b010000010000000010000000000000001; 
        
        //ADD R-R
        Mem[6] <= 33'b010000010000000000000000000000001;
        
        //ADD imm
        Mem[7] <= 33'b010000010000000010000000000000001;
        
        //STRB imm offset ADD
        Mem[8] <= 33'b011000000100000010010001000000000;
        Mem[9] <= 33'b011000000010101000011100000000000;
        Mem[10] <= 33'b011000000000100000000000000000000;
        Mem[11] <= 33'b111000000000100000000000000000001; //this state checks if MOC 1, it goes to CR, else it goes to same state

        //STRB imm offset SUB
        Mem[12] <= 33'b011000000100000010010000100000000;
        Mem[13] <= 33'b011000000010101000011100000000000;
        Mem[14] <= 33'b011000000000100000000000000000000;
        Mem[15] <= 33'b111000000000100000000000000000001;
        
        //STRB reg offset ADD
        Mem[16] <= 33'b011000_00010000000001000100_0000000;
        Mem[17] <= 33'b011000_00001010100001110000_0000000;
        Mem[18] <= 33'b011000_00000010000000000000_0000000;
        Mem[19] <= 33'b111000_00000010000000000000_0000001;
        
        //STRB reg offset SUB
        Mem[20] <= 33'b011000_00010000000001000010_0000000;
        Mem[21] <= 33'b011000_00001010100001110000_0000000;
        Mem[22] <= 33'b011000_00000010000000000000_0000000;
        Mem[23] <= 33'b111000_00000010000000000000_0000001;
        
    //     //STRB imm pre ADD
        Mem[24] <= 33'b011000_01000000001001000100_0000000;
        Mem[25] <= 33'b011000_00010000000001010000_0000000;
        Mem[26] <= 33'b011000_00001000100001110000_0000000;
        Mem[27] <= 33'b011000_00000010000000000000_0000000;
        Mem[28] <= 33'b111000_00000010000000000000_0000001;
        
    //     //STRB imm pre SUB
        Mem[29] <= 33'b011000_01000000001001000010_0000000;
        Mem[30] <= 33'b011000_00010000000001010000_0000000;
        Mem[31] <= 33'b011000_00001000100001110000_0000000;
        Mem[32] <= 33'b011000_00000010000000000000_0000000;
        Mem[33] <= 33'b111000_00000010000000000000_0000001;
       
    //    //STRB Reg pre ADD
        Mem[34] <= 33'b011000_01000000000001000100_0000000;
        Mem[35] <= 33'b011000_00010000000001010000_0000000;
        Mem[36] <= 33'b011000_00001000100001110000_0000000;
        Mem[37] <= 33'b011000_00000010000000000000_0000000;
        Mem[38] <= 33'b111000_00000010000000000000_0000001;

    //    //STRB Reg pre SUB 
        Mem[39] <= 33'b011000_01000000000001000010_0000000;
        Mem[40] <= 33'b011000_00010000000001010000_0000000;
        Mem[41] <= 33'b011000_00001000100001110000_0000000;
        Mem[42] <= 33'b011000_00000010000000000000_0000000; 
        Mem[43] <= 33'b111000_00000010000000000000_0000001;
       
    //    //STRB imm post ADD
        Mem[44] <= 33'b011000_00010000000001010000_0000000;
        Mem[45] <= 33'b011000_01000000001001000100_0000000;
        Mem[46] <= 33'b011000_00001000100001110000_0000000;
        Mem[47] <= 33'b011000_00000010000000000000_0000000;
        Mem[48] <= 33'b111000_00000010000000000000_0000001;
        
    //    //STRB imm post SUB
        Mem[49] <= 33'b011000_00010000000001010000_0000000;
        Mem[50] <= 33'b011000_01000000001001000010_0000000;
        Mem[51] <= 33'b011000_00001000100001110000_0000000;
        Mem[52] <= 33'b011000_00000010000000000000_0000000;
        Mem[53] <= 33'b111000_00000010000000000000_0000001;
        
    //    //STRB Reg post ADD
        Mem[54] <= 33'b011000_00010000000001010000_0000000;
        Mem[55] <= 33'b011000_01000000000001000100_0000000;
        Mem[56] <= 33'b011000_00001000100001110000_0000000;
        Mem[57] <= 33'b011000_00000010000000000000_0000000;
        Mem[58] <= 33'b111000_00000010000000000000_0000001;
        
    //    //STRB Reg post SUB
        Mem[59] <= 33'b011000_00010000000001010000_0000000;
        Mem[60] <= 33'b011000_01000000000001000010_0000000;
        Mem[61] <= 33'b011000_00001000100001110000_0000000;
        Mem[62] <= 33'b011000_00000010000000000000_0000000;
        Mem[63] <= 33'b111000_00000010000000000000_0000001;

       //Branch
        Mem[64] <= 33'b010000010000010011010100100000001;
    end
endmodule

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

module ControlRegister(output reg [32:0] Qs, input Clk, input [32:0] Ds); //32b bus, 20 moore lines, 6 CR and 6 NSAS
  always @ (posedge Clk) begin
   Qs <= Ds;
   //$display("Output of control register is %b", Qs);
end
endmodule

// module register32bit(output reg [31:0] Q, input [31:0] D, input clk, ld);
// initial Q <= 32'd0;
//   always @ (posedge Clk)
//   if(ld) Q <= D;
// endmodule