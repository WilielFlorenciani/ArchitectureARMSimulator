module LetsGo;

////////// Instructions
// E5 C6 40 E4 - 32'b1110_010_11100_0110_0100_000011100100; //estado 8 STRB R4,[R6,#+?]
// E0 86 40 E4 - 32'b1110_000_0100_0_0110_0100_00001110_0100; //estado 5 - ADD R4,R6,R4,ROR #?
// <hex for instr> - 32'b<bits for instruction>; //estado 7 - <syntax for add imm>
// E7 C6 40 04 - 32'b1110_011_11100_0110_0100_000000000100;// estado 16 STRB register offset ADD
// EA C6 40 E4 - 32'b1110_101_01100_0110_0100_000011100100; // estado 64 Branch instruction
//////////
// 32'b1110_001_0100_0_1111_0001_0000_00001001 //instruccion estado 7 ADD R1, R15, #0d9 E2801009
//32'b1110_001_0100_0_1111_0010_0000_00000011 // instruction estado 7 ADD R2, R15, #d3  E2802003

/////// BEGIN VARIABLES AND OBJECTS
reg Clk, reset;

parameter number15 = 4'b1111;
parameter noValue_4 = 4'b0000;
parameter noValue_1 = 1'b0;
parameter noValue_32 = 32'b0;

//wires de Control Unit
wire MF, FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MD, ME;
wire [1:0] MA, MB, MC, sizeOP;
wire [4:0] OP4OP0;
wire [9:0] current_state;
wire [31:0] IRBus;
wire MOC, Cond;
// reg Cond, MOC; //registers to simulate the signals
// reg [1:0] OpCode;

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

//wires del Shifter Sign Extender
wire [31:0] saseOut;

//wire del Flag Register
wire [3:0] FROut;

//wires del RAM 
wire [31:0] Address;
wire [31:0] DataOut;
// wire [31:0] DataIn; --> got replaced with mdrOut


//MDR, MuxE, MuxF
wire [31:0] mdrOut;
wire [31:0] muxEOut;
wire [3:0] muxFOut;

integer fi, code, i; reg [7:0] data; reg [31:0] Adr, EfAdr; //variables to handle file info


ControlUnit CU(MF, FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MA, MB, MC, sizeOP, MD, ME, OP4OP0, current_state, IRBus, Cond, MOC, reset, Clk);
RegisterFile RF(PA, PB, aluOut, A, muxFOut, C, Clk, RFld);
alu_32 ALU(aluOut, ALU_flags[3], ALU_flags[2], ALU_flags[1], ALU_flags[0], PA, AluB, OP[4:0], Cin);
ram512x8 RAM(DataOut, MOC, MOV, R_W, Address, mdrOut, sizeOP);
ConditionTester condition_tester(Cond, FROut[3], FROut[2], FROut[1], FROut[0], IRBus[31:28]); //use this one cuando vayas a usar FR
// ConditionTester condition_tester(Cond, ALU_flags[3], ALU_flags[2], ALU_flags[1], ALU_flags[0], IRBus[31:28]); 
shift_sign_extender SASExtender(saseOut, ALU_flags[3], IRBus, PB, FROut[3]);

Multiplexer4x2_4 MuxA(A,IRBus[19:16],IRBus[15:12],number15,noValue_4, MA);
Multiplexer4x2_32 MuxB(AluB, PB, saseOut, mdrOut, noValue_32, MB );
Multiplexer4x2_4 MuxC(C,IRBus[19:16],IRBus[15:12],number15,noValue_4, MC);
Multiplexer2x1_5 MuxD(OP,{1'b0, IRBus[24:21]}, OP4OP0, MD);
Multiplexer2x1_32 MuxE(muxEOut, DataOut, aluOut, ME);
Multiplexer2x1_4 MuxF(muxFOut,IRBus[3:0],IRBus[19:16], MF);

MAR Mar(Address, aluOut, MARld, Clk);
MDR Mdr(mdrOut, muxEOut, MDRld, Clk);
FlagRegister FR(FROut, ALU_flags, FRld, Clk); 
InstructionRegister IR(IRBus, DataOut, IRld, Clk);
/////// END

/////// BEGIN INITIALS

initial begin //initial to precharge memory with the file
    $display("----- Initiating Precharge -----");
    fi = $fopen("PF1_Vega_Rodriguez_Jorge_ramdata.txt","r");
    // Adr = 9'b000000000;
    Adr = 0;
    // OpCode = 2'b10;
    while (!$feof(fi)) begin
        code = $fscanf(fi, "%x", data);
        RAM.Mem[Adr] = data;
        Adr = Adr + 1;
    end
    $fclose(fi);
    $display("----- Finished Precharge ----- time:%0d", $time);
end

initial begin //initial to read content of memory after precharging
#1
    $display("----- Memory contents after precharging ----- time:%0d", $time);                       
    Adr = 7'b0000000;
    repeat (16) begin
        #1;
        $display("__RAM_Precharge: data in address %d = %x, time: %0d", Adr, RAM.Mem[Adr], $time);
        #1;
        Adr = Adr + 1;
        #1;
    end                                     
    $display("----- END PRECHARGE INFO ----- time:%0d", $time);                                               
end 

initial begin
#50 //so that clock starts when precharge tasks are done 
  Clk <= 1'b0;
  repeat(30) #5 Clk = ~Clk;
end

initial begin
#50 //so that reset starts when precharge tasks are done 
  reset = 1'b1;
#5 reset = ~reset;
end

initial begin //for signal simulations
Cin <= 0;
// Cond <= 1; //making it 0 so that it loops back to 1
// MOC <= 1; 
end

initial begin //BEGIN PRINT
#50 //delay to wait for precharge things
$display("\n~~~~~~~~Initiating ALURFCU simulation~~~~~~~~\n");
// $monitor("%h    %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b  %b",IR,aluOut,OP,current_state,FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MD, ME, MA, MB, MC,Clk,reset, $time); 
    $monitor("IR:%x, Dout:%x, sizeOP:%b, State:%d, RFld:%b, MA:%b, MB:%b, MC:%b, MD:%b, OP:%b, IRld:%b, MARld:%b, R_W:%b, MOV:%b, MOC:%b, Cond:%b, Clk:%b, reset:%b, t:%0d", IRBus, DataOut, sizeOP, current_state, RFld, MA, MB, MC, MD, OP4OP0, IRld, MARld, R_W, MOV, MOC, Cond, Clk, reset, $time);
end

initial begin //initial test instructions
#551
    Adr = 7'b0000000; //Address of instruction being tested 
    EfAdr = 5 + 228;
    $display("----- Memory contents after running: %h ----- time:%0d",{RAM.Mem[Adr], RAM.Mem[Adr+1], RAM.Mem[Adr+2], RAM.Mem[Adr+3]}, $time);                       

    repeat (4) begin //each address is a byte, so this tells amount of bytes to show 
        #1;
        $display("__RAM_After_Testing: data in address %d = %x, time: %0d", EfAdr, RAM.Mem[EfAdr], $time);
        #1;
        EfAdr = EfAdr + 1;
        #1;
    end                                     
    $display("----- END TESTING REPORT ----- time:%0d", $time);                                               
end

/////// END INITIALS
endmodule

/////////////// BEGIN CONTROL UNIT
module ControlUnit(output reg MF, FRld, RFld, IRld, MARld, MDRld, R_W , MOV, output reg [1:0] MA, MB, MC, sizeOP, output reg MD, ME, output reg [4:0] OP4OP0, output reg [9:0] current_state, input [31:0] IR, input Cond, MOC, reset, clk);

wire [9:0] mux7Out;
wire mux1Out;
wire [9:0] AdderOut;
wire [9:0] EncoderOut;
wire [38:0] CRin;
wire [38:0] CRout;
wire invOut;
wire [9:0] incrementedState;
wire [1:0] M;
wire noValue = 0;
wire [9:0] val1 = 1;
wire [9:0] new_state, curr_state;

always @ (*) begin
    // Cin <= CRout[?];
    MF <= CRout[38];
    FRld <= CRout[37];
    RFld <= CRout[36];
    IRld <= CRout[35];
    MARld <= CRout[34];
    MDRld <= CRout[33];
    R_W <= CRout[32];
    MOV <= CRout[31];
    MA <= CRout[30:29];
    MB <= CRout[28:27];
    MC <= CRout[26:25];
    MD <= CRout[24];
    ME <= CRout[23];
    OP4OP0 <= CRout[22:18];
    sizeOP <= CRout[17:16];
    current_state <= curr_state;
end
// moore        size n2n1n0 inv  s1s0   CR
//             17:16  15:13  12  11:10  9-0
Multiplexer7_4x2 mux7_4x2 (mux7Out, EncoderOut, val1, CRout[9:0], incrementedState, M, reset); 
Adder adder (AdderOut, mux7Out);
Microstore microstore (CRin, new_state, reset, mux7Out);
ControlRegister control_register (CRout, curr_state, clk, CRin, new_state);
NextStateAddressSelector nsas (M, invOut, CRout[15:13]);
Inverter inv (invOut, mux1Out, CRout[12]);
IncrementerRegister incr_reg (incrementedState, AdderOut, clk);
Multiplexer1_4x2 mux1_4x2 (mux1Out, MOC, Cond, noValue, noValue, CRout[11:10]); //aqui MOC tiene que ir en 0 y Cond en 1//CRout?
Encoder encoder (EncoderOut, IR);

endmodule

module Inverter(output reg out, input in, inv);
    
    always @(in, inv)
        out = inv ? !in: in;
        
endmodule

// multiplexer4x2
module Multiplexer7_4x2(output reg [9:0] out, input [9:0] I0, I1, I2, I3, input [1:0] S, input reset);

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

module IncrementerRegister(output reg [9:0] Q, input [9:0] D, input  Clk);
always @(posedge Clk)
    Q <= D;
endmodule

module Adder(output reg [9:0] out, input [9:0] in);
always @(in)
    out <= in + 1'b1;
// $display("__Adder input: %d, Adder output: %d", in, out); ---
endmodule

module Encoder(output reg [9:0] Out, input [31:0] Instruction);
always @(Instruction) begin
case(Instruction[27:25])
    3'b001: begin
            case(Instruction[24:21])
            //case de opcode de la instruccion
            4'b0100: Out = 10'b0000000111;
            endcase
            end
    3'b000: begin
            //case de opcode de la instruccion
            case(Instruction[24:21])
                4'b0100: begin 
                        if(Instruction[4] == 1'b0)
                                begin
                                case(Instruction[6:5]) 
                                    // logical shift left
                                2'b00:  Out = 10'b0000000101;
                                endcase
                                end
                        else if(Instruction[11:5]== 7'b0000000)
                             Out = 10'b0000000110;
                        end
            endcase
            
            end
    3'b010: begin
            case(Instruction[24:20])
                //strb immed
                5'b11100:   Out = 10'b0000001000;
                5'b10100:   Out = 10'b0000001100;
                5'b11110:   Out = 10'b0000011000;
                5'b10110:   Out = 10'b0000011101;
                5'b01100:   Out = 10'b0000101100;
                5'b00100:   Out = 10'b0000110001;
                //str immed
                5'b11000:   Out = 10'b0001000001;
                5'b10000:   Out = 10'b0001000101;
                5'b11010:   Out = 10'b0001010001;
                5'b10010:   Out = 10'b0001010110;
                5'b01000:   Out = 10'b0001100101;
                5'b00000:   Out = 10'b0001101010;
            endcase
            end
    3'b011: begin
            if(Instruction[11:4]== 8'b00000000)
                begin
                case(Instruction[24:20])
                    //strb register
                    5'b11100:   Out = 10'b0000010000;
                    5'b10100:   Out = 10'b0000010100; 
                    5'b11110:   Out = 10'b0000100010;
                    5'b10110:   Out = 10'b0000100010;
                    5'b01100:   Out = 10'b0000110110;
                    5'b00100:   Out = 10'b0000111011;
                    //str register
                    5'b11000:   Out = 10'b0001001001;
                    5'b10000:   Out = 10'b0001001101;
                    5'b11010:   Out = 10'b0001011011;
                    5'b10010:   Out = 10'b0001100000;
                    5'b01000:   Out = 10'b0001101111;
                    5'b00000:   Out = 10'b0001110100;
                endcase
                end
            end
    3'b101: Out = 10'b0001000000;

    default:    Out = 10'b0000000001;
endcase
end
endmodule

module Microstore (output reg [38:0] out, output reg [9:0] current_state, input reset, input [9:0] next_state);
    //n2n1n0 inv s1s0 moore cr(6)
        parameter[0:39 * 65 - 1] CR_states = { //cambiar aqui 65 por el numero de estados que hay 
        39'b001000000011101001101000110000000000000, //0
        39'b000010001000001010000000110000000000000, //1
        39'b001000111000101010001_10_0110000000000000, //2
        39'b000100110000000000000001011000000000011, //3
        39'b000000000000000000000001000010000000001, //4
        39'b010000010000000010000000000000001, //5
        39'b010000010000000000000000000000001, //6
        39'b001000000001000000000100100000000000001, //7
        39'b000010000001001000100000110000000000000, //8
        39'b000001010100001110000000110000000000000, //9
        39'b000000010000000000000000110000000000000, //10
        39'b000000010000000000000001110000000001011, //11
        39'b011000000100000010010000100000000, //12
        39'b011000000010101000011100000000000, //13
        39'b011000000000100000000000000000000, //14
        39'b111000000000100000000000000001111, //15
        39'b011000_00010000000001000100_0000000, //16
        39'b011000_00001010100001110000_0000000, //17
        39'b011000_00000010000000000000_0000000, //18
        39'b111000_00000010000000000000_0010011, //19
        39'b011000_00010000000001000010_0000000, //20
        39'b011000_00001010100001110000_0000000, //21
        39'b011000_00000010000000000000_0000000, //22
        39'b111000_00000010000000000000_0010111, //23
        39'b011000_01000000001001000100_0000000, //24
        39'b011000_00010000000001010000_0000000, //25
        39'b011000_00001000100001110000_0000000, //26
        39'b011000_00000010000000000000_0000000, //27
        39'b111000_00000010000000000000_0011100, //28
        39'b011000_01000000001001000010_0000000, //29
        39'b011000_00010000000001010000_0000000, //30
        39'b011000_00001000100001110000_0000000, //31
        39'b011000_00000010000000000000_0000000, //32
        39'b111000_00000010000000000000_0100001, //39
        39'b011000_01000000000001000100_0000000, //34
        39'b011000_00010000000001010000_0000000, //35
        39'b011000_00001000100001110000_0000000, //36
        39'b011000_00000010000000000000_0000000, //37
        39'b111000_00000010000000000000_0100110, //38
        39'b011000_01000000000001000010_0000000, //39
        39'b011000_00010000000001010000_0000000, //40
        39'b011000_00001000100001110000_0000000, //41
        39'b011000_00000010000000000000_0000000, //42
        39'b111000_00000010000000000000_0101011, //43
        39'b011000_00010000000001010000_0000000, //44
        39'b011000_01000000001001000100_0000000, //45
        39'b011000_00001000100001110000_0000000, //46
        39'b011000_00000010000000000000_0000000, //47
        39'b111000_00000010000000000000_0110000, //48
        39'b011000_00010000000001010000_0000000, //49
        39'b011000_01000000001001000010_0000000, //50
        39'b011000_00001000100001110000_0000000, //51
        39'b011000_00000010000000000000_0000000, //52
        39'b111000_00000010000000000000_0110101, //53
        39'b011000_00010000000001010000_0000000, //54
        39'b011000_01000000000001000100_0000000, //55
        39'b011000_00001000100001110000_0000000, //56
        39'b011000_00000010000000000000_0000000,//57
        39'b111000_00000010000000000000_0111010,//58
        39'b011000_00010000000001010000_0000000,//59
        39'b011000_01000000000001000010_0000000,//60
        39'b011000_00001000100001110000_0000000,//61
        39'b011000_00000010000000000000_0000000,//62
        39'b111000_00000010000000000000_0111111,//63
        39'b010000010000010011010100100000001 //64
       };

always @(next_state, reset)
begin
    if (reset) begin
        out           <= CR_states[0+:39];
        current_state <= 10'd0;
    end
    else begin
        out           <= CR_states[39*next_state+:39];
        current_state <= next_state;
    end
end
endmodule

module ControlRegister(output reg [38:0] Qs, output reg [9:0] current_state, input Clk, input [38:0] Ds, input [9:0] next_state); 
  always @ (posedge Clk) begin
   Qs <= Ds;
   current_state <= next_state; 
end
endmodule
///////////////// END CONTROL UNIT

///////////////// BEGIN ALU
module alu_32 (output reg [31:0] Out, output reg Carry,Zero,Neg,Vflow, input [31:0] A,B, input [4:0] Sel,input Cin);

always @(*) begin
 $display("__ALU: A:%b, B:%b, Sel:%b, aluOut:%b, t:%0d", A, B, Sel, Out, $time);
    // Out = 32'b0;
    // Carry = 1'b0;
    // Zero = 1'b0;
    // Neg = 1'b0;
    // Vflow = 1'b0;
    
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

always@(I15, rfLd) begin
    $display("__R15: %b, Clock:%b, t:%0d\n__R4: %b, Clock:%b, t:%0d", I15, clk, $time, I4, clk, $time);
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
// initial Q <= 32'd0;
// initial begin
// Q <= 32'b00000000000000000000000000000101;
// end
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

//this one is for MuxF
module Multiplexer2x1_4(output reg [3:0] Q, input [3:0] I0, I1, input S);
    
    always @ (*)
    begin
        case(S)
            1'b0: Q <= I0;
            1'b1: Q <= I1;
        endcase
        end
    
endmodule

module Multiplexer2x1_32(output reg [31:0] Q, input [31:0] I0, I1, input S);
    
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
    $display("__MAR: marOut:%b, t:%0d", Q, $time);
end
endmodule

module MDR(output reg [31:0] Q, input [31:0] D, input LE, Clk);
always @(posedge Clk) begin
    if(LE) begin 
        Q <= D;
    end
    $display("__MDR: mdrOut:%b, t:%0d", Q, $time);
end
endmodule

module FlagRegister(output reg [3:0] Q, input [3:0] D, input LE, Clk);
always @(posedge Clk) begin
    if(LE) begin 
        Q <= D;
    end
    $display("__FR: FROut:%b, t:%0d", Q, $time);
end
endmodule

module InstructionRegister(output reg [31:0] Q, input [31:0] D, input LE, Clk);
always @(posedge Clk)
    if(LE) Q <= D;
endmodule
///////////////// END REGISTERS

///////////////// BEGIN RAM
module ram512x8(output reg [31:0] DataOut, output reg MOC, input Enable, input ReadWrite, input [31:0] Address, input [31:0] DataIn, input [1:0] OpCode);

  reg [7:0] Mem[0:511]; //512 localizaciones de 8 bits
  always @ (Enable, ReadWrite) begin
    MOC <= 0; //MOC <= 0;
    $display("__RAM: entered the always, MOC:%b", MOC);
    if (Enable) begin
        case (OpCode) 
            2'b00: begin //opcode for byte operations 
                if(ReadWrite) begin //read
                    DataOut[7:0] = Mem[Address];
                    DataOut[31:8] = 24'h000000;
                    MOC <= 1; 
                    $display("__RAM: read a byte, DataOut:%b", DataOut);
                end else begin  //write
                    Mem[Address] <= DataIn[7:0];
                    MOC <= 1;
                    $display("__RAM: wrote a byte Adr:%b, Mem[Adr]:%b", Address, Mem[Address]);
                end
            end
            2'b01: begin //opcode for halfword operations
                if(ReadWrite) begin //read
                    DataOut[31:16] <= 16'h0000;
                    DataOut[15:8] <= Mem[Address];
                    DataOut[7:0] <= Mem[Address + 1];
                    MOC <= 1;
                end else begin  //write
                    Mem[Address] <= DataIn[15:8];
                    Mem[Address+1] <= DataIn[7:0];
                    MOC <= 1;
                end
            end
            2'b10: begin //opcode for word operations
                if(ReadWrite) begin //read
                    DataOut[31:24] <= Mem[Address];
                    DataOut[23:16] <= Mem[Address+1];
                    DataOut[15:8] <= Mem[Address+2];
                    DataOut[7:0] <= Mem[Address+3];
                    MOC <= 1;
                end
                else begin //write
                    Mem[Address] <= DataIn[31:24];
                    Mem[Address+1] <= DataIn[23:16];
                    Mem[Address+2] <= DataIn[15:8];
                    Mem[Address+3] <= DataIn[7:0];
                    MOC <= 1;
                end
            end
            default: begin //default to doubleword if its none of the others
                if(ReadWrite) begin //read
                    DataOut[31:24] <= Mem[Address];
                    DataOut[23:16] <= Mem[Address+1];
                    DataOut[15:8] <= Mem[Address+2];
                    DataOut[7:0] <= Mem[Address+3];
                    // #2
                    // DataOut[31:24] <= Mem[Address+4];
                    // DataOut[23:16] <= Mem[Address+5];
                    // DataOut[15:8] <= Mem[Address+6];
                    // DataOut[7:0] <= Mem[Address+7];
                    MOC <= 1;
                end
                else begin //write 
                    Mem[Address] <= DataIn[31:24];
                    Mem[Address+1] <= DataIn[23:16];
                    Mem[Address+2] <= DataIn[15:8];
                    Mem[Address+3] <= DataIn[7:0];
                    // #2
                    // Mem[Address+4] <= DataIn[31:24];
                    // Mem[Address+5] <= DataIn[23:16];
                    // Mem[Address+6] <= DataIn[15:8];
                    // Mem[Address+7] <= DataIn[7:0];
                    MOC <= 1;
                end
            end
        endcase 
    end
end
endmodule
////////////// END RAM

////////////// BEGIN CONDITION TESTER
module ConditionTester (output reg Cond, input C, Z, N, V, input [3:0] CC);
always @ (*) begin
case(CC)
    4'h0: begin
        Cond <= Z; //EQ Equal
    end
    4'h1: begin
        Cond <= ~Z; //NE Not equal
    end
    4'h2: begin
        Cond <= C; //CS/HS Unsigned higher or same
    end
    4'h3: begin
        Cond <= ~C; //CC/LO Unsigned lower
    end
    4'h4: begin
        Cond <= N; //MI Mius
    end
    4'h5: begin
        Cond <= ~N; //PL Positive or Zero
    end
    4'h6: begin
        Cond <= V; //VS Overflow
    end
    4'h7: begin
        Cond <= ~V; //VC No overflow
    end
    4'h8: begin
        Cond <= C & ~Z; //HI Unsigned higher //test this, might be &&?
    end
    4'h9: begin
        Cond <= ~C | Z; //LS Unsigned lower or same
    end
    4'hA: begin
        Cond <= ~(N ^ V); //GE Greater or equal
    end
    4'hB: begin
        Cond <= N ^ V; //LT Less than
    end
    4'hC: begin
        Cond <= ~Z & (~(N ^ V)); //GT Greater than
    end
    4'hD: begin
        Cond <= Z | (~(N ^ ~V)); //LE Less than or eual
    end
    4'hE: begin
        Cond <= 1'b1; //AL Always
    end
endcase

$display("__CT: Cond:%b, Z:%b, C:%b, N:%b, V:%b, CC:%b", Cond, Z, C, N, V, CC);

end
endmodule
////////////// END CONDITION TESTER

////////////// BEGIN SHIFTER SIGN EXTENDER
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
    3'b101:  begin
        {extender_out} = {{6{instruction[23]}},instruction[23:0]} <<2;
        

        end
    endcase
        

end           

//end 

endmodule
////////////// END SHIFTER SIGN EXTENDER

