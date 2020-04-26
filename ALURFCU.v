module LetsGo;
/////// BEGIN VARIABLES AND OBJECTS
reg Clk, reset;

parameter number15 = 4'b1111;
parameter noValue_4 = 4'b0000;
parameter noValue_1 = 1'b0;
parameter noValue_32 = 32'b0;

//wires de Control Unit
wire FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MD, ME;
wire [1:0] MA, MB, MC;
wire [4:0] OP4OP0;
wire [6:0] current_state;
wire [31:0] IRBus;
// wire Cond, MOC;
reg Cond, MOC; //registers to simulate the signals

//wires de Register File y ALU
wire [3:0] ALU_flags; // 3-Carry, 2-Zero, 1-Negative, 0-Vflow
wire [31:0] AluB;
wire [31:0] PA;
wire [31:0] PB;
wire [31:0] aluOut;
wire [3:0] A;
// wire [3:0] B = IRBus[3:0];
wire [3:0] C;
wire [4:0] OP; //muxD output
reg Cin; //wire Cin; --> for when we figure out Cin
//wire [31:0] SASExtender;

//wires del RAM 
wire [31:0] Address;
//wire [31:0] MDRout;
wire [31:0] DataOut;
wire [31:0] DataIn;
reg [1:0] OpCode;


ControlUnit CU(FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MA, MB, MC, MD, ME, OP4OP0, current_state, IRBus, Cond, MOC, reset, Clk);
RegisterFile RF(PA, PB, aluOut, A, IRBus[3:0], C, Clk, RFld);
alu_32 ALU(aluOut, ALU_flags[3], ALU_flags[2], ALU_flags[1], ALU_flags[0], PA, PB, OP[4:0], Cin);

Multiplexer4x2_4 MuxA(A,IRBus[19:16],IRBus[15:12],number15,noValue_4, MA);
Multiplexer4x2_32 MuxB(AluB, PB, noValue_32, noValue_32, noValue_32, MB );
Multiplexer4x2_4 MuxC(C,IRBus[15:12],number15,IRBus[19:16],noValue_4, MC); //check this, any states that used IR19-16 now have to point to 10 instead of 00
Multiplexer2x1_5 MuxD(OP,1'b0 + IRBus[24:21], OP4OP0, MD);

MAR Mar(Address, aluOut, MARld, Clk);
/////// END

/////// BEGIN INITIALS

initial begin
// #50 //so that clock starts when precharge tasks are done 
  Clk <= 1'b0;
  repeat(20) #5 Clk = ~Clk;
end

initial begin
// #50 //so that reset starts when precharge tasks are done 
  reset = 1'b1;
#5 reset = ~reset;
end

initial begin //for signal simulations
Cin <= 0;
Cond <= 0; //making it 0 so that it loops back to 1
MOC <= 1; 
end

initial begin
// #50 //delay to wait for precharge things
$display("\n~~~~~~~~Initiating MicroSan simulation~~~~~~~~\n");
// $monitor("%h    %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b",IR,aluOut,OP,current_state,FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MD, ME, MA, MB, MC,Clk,reset, $time); 
    $monitor("IR:%x, Dout:%x, alO:%b, State:%d, RFld:%b, MA:%b, MB:%b, MC:%b, OP:%b, IRld:%b, MARld:%b, R_W:%b, MOV:%b, MOC:%b, Cond:%b, Clk:%b, reset:%b, t:%0d", IRBus, DataOut, aluOut, current_state, RFld, MA, MB, MC, OP4OP0, IRld, MARld, R_W, MOV, MOC, Cond, Clk, reset, $time);
end

/////// END INITIALS
endmodule

/////////////// BEGIN CONTROL UNIT
module ControlUnit(output reg FRld, RFld, IRld, MARld, MDRld, R_W , MOV, output reg [1:0] MA, MB, MC, output reg MD, ME, output reg [4:0] OP4OP0, output reg [6:0] current_state, input [31:0] IR, input Cond, MOC, reset, clk);

wire [6:0] mux7Out;
wire mux1Out;
wire [6:0] AdderOut;
wire [6:0] EncoderOut;
wire [32:0] CRin;
wire [32:0] CRout;
wire invOut;
wire [6:0] incrementedState;
wire [1:0] M;
wire noValue = 0;
wire [6:0] val1 = 1;
wire [6:0] new_state, curr_state;

always @ (*) begin
    // Cin <= CR[34];
    FRld <= CRout[26];
    RFld <= CRout[25];
    IRld <= CRout[24];
    MARld <= CRout[23];
    MDRld <= CRout[22];
    R_W <= CRout[21];
    MOV <= CRout[20];
    MA <= CRout[19:18];
    MB <= CRout[17:16];
    MC <= CRout[15:14];
    MD <= CRout[13];
    ME <= CRout[12];
    OP4OP0 <= CRout[11:7];
    current_state <= curr_state;
end


Multiplexer7_4x2 mux7_4x2 (mux7Out, EncoderOut, val1, CRout[6:0], incrementedState, M, reset); //CRout?
Adder adder (AdderOut, mux7Out);
Microstore microstore (CRin, new_state, reset, mux7Out);
ControlRegister control_register (CRout, curr_state, clk, CRin, new_state);//CRout?
NextStateAddressSelector nsas (M, invOut, CRout[32:30]);//CRout??
Inverter inv (invOut, mux1Out, CRout[29]);
IncrementerRegister incr_reg (incrementedState, AdderOut, clk);
Multiplexer1_4x2 mux1_4x2 (mux1Out, MOC, Cond, noValue, noValue, CRout[28:27]); //aqui MOC tiene que ir en 0 y Cond en 1//CRout?
Encoder encoder (EncoderOut, IR);

endmodule

module Inverter(output reg out, input in, inv);
    
    always @(in, inv)
        out = inv ? !in: in;
        
endmodule

// multiplexer4x2
module Multiplexer7_4x2(output reg [6:0] out, input [6:0] I0, I1, I2, I3, input [1:0] S, input reset);

    always @ (*) begin
        if(reset) 
        begin
            out <= 0;
            // $display("~~~ mux is outputting 0 because of reset ~~~");
        end
        else begin
        case(S)
            2'h0: out <= I0;
            2'h1: out <= I1;
            2'h2: out <= I2;
            2'h3: out <= I3;
        endcase
        // $display("__Mux7 - out:%d,  enc:%d, same:%d, cr:%d, inc:%d, S:%b,       t:%0d", out,I0,I1,I2,I3,S,$time); 
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
    always @ (Sts, N) begin
        case(N)
            3'o0: M <= 2'b00; //Encoder
            3'o1: M <= 2'b01; // Mux7output
            3'o2: M <= 2'b10; //Control Register
            3'o3: M <= 2'b11; // Incrementer
            3'o4: begin 
                    M[1] <= ~Sts;
                    M[0] <= 1'b0;
                  end
            3'o5: begin 
                    M[1] <= 1'b1;
                    M[0] <= ~Sts;
                  end
            3'o6: begin 
                    M[1] <= ~Sts;
                    M[0] <= ~Sts;
                  end
            3'o7: begin 
                    M[1] <= ~Sts;
                    M[0] <= Sts;
                  end
        endcase

        // $display("__NSAS - out:%b, Ns:%b, Sts:%b                                     t:%0d", M, N, Sts, $time); 
        end
endmodule

module IncrementerRegister(output reg [6:0] Q, input [6:0] D, input  Clk);
always @(posedge Clk)
    Q <= D;
endmodule

module Adder(output reg [6:0] out, input [6:0] in);
always @(in)
    out <= in + 1'b1;
// $display("__Adder input: %d, Adder output: %d", in, out); ---
endmodule

module InstructionRegister(output reg [31:0] Q, input [31:0] D, input LE, Clk);
always @(posedge Clk)
    if(LE) Q <= D;
endmodule

module Encoder(output reg [6:0] Out, input [31:0] Instruction);
always @(Instruction) begin
case(Instruction[27:25])
    3'b001: begin
            if(Instruction[24:21] == 4'b0100)
                 Out = 7'b0000111;
            end
    3'b000: begin
            if((Instruction[24:21] == 4'b0100) && (Instruction[4] == 1'b0))
                Out = 7'b0000101;
            else if((Instruction[24:21] == 4'b0100) && (Instruction[11:5]== 7'b0000000))
                Out = 7'b0000110;
            end
    3'b010: begin
            if(Instruction[24:20]== 5'b11100)
                 Out = 7'b0001000;
            else if(Instruction[24:20]== 5'b10100)
                Out = 7'b0001100;
            end
    3'b011: begin
            if(Instruction[11:4]== 8'b00000000)
                begin
                case(Instruction[24:20])
                    5'b11100:   Out = 7'b0010000;
                    5'b10100:   Out = 7'b0010100; 
                    5'b11110:   Out = 7'b0100010;
                    5'b10110:   Out = 7'b0100010;
                    5'b01100:   Out = 7'b0110110;
                    5'b00100:   Out = 7'b0111011;
                endcase
                end
            end
    3'b010: begin
            case(Instruction[24:20]) 
              5'b11110: Out = 7'b0011000;
              5'b10110: Out = 7'b0011101; 
              5'b01100: Out = 7'b0101100;
              5'b00100: Out = 7'b0110001;
            endcase
            end
    3'b101: Out = 7'b1000000;

    default:    Out = 7'b0000001;
endcase
end
endmodule

module Microstore (output reg [32:0] out, output reg [6:0] current_state, input reset, input [6:0] next_state);
    //n2n1n0 inv s1s0 moore cr(6)
        parameter[0:33 * 65 - 1] CR_states = {
        33'b011000010000000110110011010000000, //0
        33'b011000000100010000010100000000000, //1
        33'b011000010001110000110100010000000, //2
        33'b101100001001100000000000000000011, //3
        33'b100001000000000000000000000000001, //4
        33'b010000010000000010000000000000001, //5
        33'b010000010000000000000000000000001, //6
        33'b010000010000000010000000000000001, //7
        33'b011000000100000010010001000000000, //8
        33'b011000000010101000011100000000000, //9
        33'b011000000000100000000000000000000, //10
        33'b111000000000100000000000000001011, //11
        33'b011000000100000010010000100000000, //12
        33'b011000000010101000011100000000000, //13
        33'b011000000000100000000000000000000, //14
        33'b111000000000100000000000000001111, //15
        33'b011000_00010000000001000100_0000000, //16
        33'b011000_00001010100001110000_0000000, //17
        33'b011000_00000010000000000000_0000000, //18
        33'b111000_00000010000000000000_0010011, //19
        33'b011000_00010000000001000010_0000000, //20
        33'b011000_00001010100001110000_0000000, //21
        33'b011000_00000010000000000000_0000000, //22
        33'b111000_00000010000000000000_0010111, //23
        33'b011000_01000000001001000100_0000000, //24
        33'b011000_00010000000001010000_0000000, //25
        33'b011000_00001000100001110000_0000000, //26
        33'b011000_00000010000000000000_0000000, //27
        33'b111000_00000010000000000000_0011100, //28
        33'b011000_01000000001001000010_0000000, //29
        33'b011000_00010000000001010000_0000000, //30
        33'b011000_00001000100001110000_0000000, //31
        33'b011000_00000010000000000000_0000000, //32
        33'b111000_00000010000000000000_0100001, //33
        33'b011000_01000000000001000100_0000000, //34
        33'b011000_00010000000001010000_0000000, //35
        33'b011000_00001000100001110000_0000000, //36
        33'b011000_00000010000000000000_0000000, //37
        33'b111000_00000010000000000000_0100110, //38
        33'b011000_01000000000001000010_0000000, //39
        33'b011000_00010000000001010000_0000000, //40
        33'b011000_00001000100001110000_0000000, //41
        33'b011000_00000010000000000000_0000000, //42
        33'b111000_00000010000000000000_0101011, //43
        33'b011000_00010000000001010000_0000000, //44
        33'b011000_01000000001001000100_0000000, //45
        33'b011000_00001000100001110000_0000000, //46
        33'b011000_00000010000000000000_0000000, //47
        33'b111000_00000010000000000000_0110000, //48
        33'b011000_00010000000001010000_0000000, //49
        33'b011000_01000000001001000010_0000000, //50
        33'b011000_00001000100001110000_0000000, //51
        33'b011000_00000010000000000000_0000000, //52
        33'b111000_00000010000000000000_0110101, //53
        33'b011000_00010000000001010000_0000000, //54
        33'b011000_01000000000001000100_0000000, //55
        33'b011000_00001000100001110000_0000000, //56
        33'b011000_00000010000000000000_0000000,//57
        33'b111000_00000010000000000000_0111010,//58
        33'b011000_00010000000001010000_0000000,//59
        33'b011000_01000000000001000010_0000000,//60
        33'b011000_00001000100001110000_0000000,//61
        33'b011000_00000010000000000000_0000000,//62
        33'b111000_00000010000000000000_0111111,//63
        33'b010000010000010011010100100000001 //64
       };

always @(next_state, reset)
begin
    if (reset) begin
        out           <= CR_states[0+:33];
        current_state <= 10'd0;
    end
    else begin
        out           <= CR_states[33*next_state+:33];
        current_state <= next_state;
    end
end
endmodule

module ControlRegister(output reg [32:0] Qs, output reg [6:0] current_state, input Clk, input [32:0] Ds, input [6:0] next_state); //32b bus, 20 moore lines, 6 CR and 6 NSAS
  always @ (posedge Clk) begin
   Qs <= Ds;
   current_state <= next_state; 
end
endmodule
///////////////// END CONTROL UNIT

///////////////// BEGIN ALU
module alu_32 (output reg [31:0] Out, output reg Carry,Zero,Neg,Vflow, input [31:0] A,B, input [4:0] Sel,input Cin);

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
    5'b01011:  {Carry,Out} = A + B;
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
///////////////// END ALU

///////////////// BEGIN REGISTER FILE
module RegisterFile(output wire [31:0] PA, PB, input [31:0] PC, input [3:0] A, B, C, input clk, rfLd);

wire [15:0] BDselect;

wire [31:0] I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15;

always@(I15) begin
    $display("__R15: %b, Clock:%b, t:%0d", I15, clk, $time);
end

binaryDecoder16bit decoder (BDselect, C, rfLd);

register32bit r0 (I0, PC, clk, BDselect[0]);
register32bit r1 (I1, PC, clk, BDselect[1]);
register32bit r2 (I2, PC, clk, BDselect[2]);
register32bit r3 (I3, PC, clk, BDselect[3]);
register32bit r4 (I4, PC, clk, BDselect[4]);
register32bit r5 (I5, PC, clk, BDselect[5]);
register32bit r6 (I6, PC, clk, BDselect[6]);
register32bit r7 (I7, PC, clk, BDselect[7]);
register32bit r8 (I8, PC, clk, BDselect[8]);
register32bit r9 (I9, PC, clk, BDselect[9]);
register32bit r10 (I10, PC, clk, BDselect[10]);
register32bit r11 (I11, PC, clk, BDselect[11]);
register32bit r12 (I12, PC, clk, BDselect[12]);
register32bit r13 (I13, PC, clk, BDselect[13]);
register32bit r14 (I14, PC, clk, BDselect[14]);
register32bit r15 (I15, PC, clk, BDselect[15]); // Program Counter


Multiplexer16x4 muxA (PA, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, A);

Multiplexer16x4 muxB (PB, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, B);

endmodule


// multiplexer16x4
module Multiplexer16x4(output reg [31:0] Q, input [31:0] I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, input [3:0] S);
    
    always @ (*)
    begin
        case(S)
            4'h0: Q <= I0;
            4'h1: Q <= I1;
            4'h2: Q <= I2;
            4'h3: Q <= I3;
            4'h4: Q <= I4;
            4'h5: Q <= I5;
            4'h6: Q <= I6;
            4'h7: Q <= I7;
            4'h8: Q <= I8;
            4'h9: Q <= I9;
            4'ha: Q <= I10;
            4'hb: Q <= I11;
            4'hc: Q <= I12;
            4'hd: Q <= I13;
            4'he: Q <= I14;
            4'hf: Q <= I15;
        endcase
        end
    
endmodule 

module binaryDecoder16bit(output wire [15:0]E, input wire [3:0] d, input Ld);
    
    wire nd3, nd2, nd1, nd0;
    
    not(nd3, d[3]);
    not(nd2, d[2]);
    not(nd1, d[1]);
    not(nd0, d[0]);
                                                //Output wires
    and(E[0], nd3, nd2, nd1, nd0, Ld);          // wire 0
    and(E[1], nd3, nd2, nd1, d[0], Ld);         // wire 1
    and(E[2], nd3, nd2, d[1], nd0, Ld);         // wire 2
    and(E[3], nd3, nd2, d[1], d[0], Ld);        // wire 3
    and(E[4], nd3, d[2], nd1, nd0, Ld);         // wire 4
    and(E[5], nd3, d[2], nd1, d[0], Ld);        // wire 5
    and(E[6], nd3, d[2], d[1], nd0, Ld);        // wire 6
    and(E[7], nd3, d[2], d[1], d[0], Ld);       // wire 7
    and(E[8], d[3], nd2, nd1, nd0, Ld);         // wire 8
    and(E[9], d[3], nd2, nd1, d[0], Ld);        // wire 9
    and(E[10], d[3], nd2, d[1], nd0, Ld);       // wire 10
    and(E[11], d[3], nd2, d[1], d[0], Ld);      // wire 11
    and(E[12], d[3], d[2], nd1, nd0, Ld);       // wire 12
    and(E[13], d[3], d[2], nd1, d[0], Ld);      // wire 13
    and(E[14], d[3], d[2], d[1], nd0, Ld);      // wire 14
    and(E[15], d[3], d[2], d[1], d[0], Ld);     // wire 15
        
endmodule


module register32bit(output reg [31:0] Q, input [31:0] D, input clk, ld);
initial Q <= 32'd0;
  always @ (posedge clk)
  if(ld) Q <= D;
endmodule
///////////////// END REGISTER FILE

///////////////// BEGIN MUXES
//this one's used for MuxB --v
module Multiplexer4x2_32(output reg [31:0] Q, input [31:0] I0, I1, I2, I3, input [1:0] S);
    
    always @ (*)
    begin
        case(S)
            4'h0: Q <= I0;
            4'h1: Q <= I1;
            4'h2: Q <= I2;
            4'h3: Q <= I3;
        endcase
        end
    
endmodule 

//this one's used for MuxA and MuxC
module Multiplexer4x2_4(output reg [3:0] Q, input [3:0] I0, I1, I2, I3, input [1:0] S);
    
    always @ (*)
    begin
        case(S)
            4'h0: Q <= I0;
            4'h1: Q <= I1;
            4'h2: Q <= I2;
            4'h3: Q <= I3;
        endcase
        end
    
endmodule

//this one's used for MuxD
module Multiplexer2x1_5(output reg [4:0] Q, input [4:0] I0, I1, input S);
    
    always @ (*)
    begin
        case(S)
            1'b0: Q <= I0;
            1'b1: Q <= I1;
        endcase
        end
    
endmodule
///////////////// END MUXES

///////////////// BEGIN REGISTERS
module MAR(output reg [31:0] Q, input [31:0] D, input LE, Clk);
always @(posedge Clk) begin
    if(LE) begin 
        Q <= D;
    end
    $display("__MAR: marOut:%b", Q);
end
endmodule
///////////////// END REGISTERS
