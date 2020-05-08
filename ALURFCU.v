module LetsGo;

////////// Instructions 
// E5 C1 20 E4 - 32'b1110_010_11100_0001_0010_000011100100; //estado 8 STRB R2,[R1,#228] --> 11 in 241 // it works 
// E08120E2 - 32'b1110_000_0100_0_0110_0100_00001110_0100; //estado 5 - ADD R2,R1,R2,ROR #1 -> 001011 --> ROR #1: 000101; 5 + 13 = 18 --> 10010 in r2 
// E2 8F 10 09 - 32'b<bits for instruction>; //estado 7 - ADD R1, R15, #0d9 //temp --> R1 = 13        // it works 
// E2 8F 20 03 - 32'b<bits for instruction>; //estado 7 - ADD R2, R15, #d3 //temp --> R2 = 11         // it works 
// E7 C6 40 04 - 32'b1110_011_11100_0110_0100_000000000100;// estado 16 STRB register offset ADD
// EA C6 40 E4 - 32'b1110_101_01100_0110_0100_000011100100; // estado 64 Branch instruction
// E8 A1 00 0C - STMIA R1!, {R2, R3} //state 715?
// E8 81 00 0C - STMIA R1, {R2, R3} //state 724
// E2 82 30 04 -> state 7, add r3, r2, #4 --> R3 = 15
//  testing: store r2=11 in 13 and r3=15 in 17, r1 ends with 21
//  whats happening: 13 in 16, 15 in 20
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
wire MK, MultiRegld, MJ, MI, MG, MF, FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MD, ME;
wire [2:0] MA, MB, MC;
wire [1:0] sizeOP;
wire [4:0] OP4OP0;
wire [9:0] current_state;
wire [31:0] IRBus;
wire MOC, Cond;
wire [3:0] CondTestOp;
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

//Adder_4 wires
wire [3:0] adder4Out;

//multiencoder wires
wire [3:0] multiencOut;

//multireg wires
wire [31:0] multiregOut;

//MDR, MuxE, MuxF, MuxG, MuxH, MuxI, MuxJ, MuxK
wire [31:0] mdrOut, muxEOut, muxGOut, muxIOut, muxKOut;
wire [3:0] muxFOut, muxJOut;

//BaseRegister wires 
wire [31:0] baseregOut;
wire BaseRegld;

integer fi, code, i; reg [7:0] data; reg [31:0] Adr, EfAdr; //variables to handle file info


ControlUnit CU(CondTestOp, BaseRegld, MultiRegld, FRld, RFld, IRld, MARld, MDRld, R_W, MOV, MA, MC, MB, sizeOP, MD, ME, MF, MG, MI, MJ, MK, OP4OP0, current_state, IRBus, Cond, MOC, reset, Clk);
RegisterFile RF(PA, PB, muxKOut, A, muxFOut, C, Clk, RFld);
alu_32 ALU(aluOut, ALU_flags[3], ALU_flags[2], ALU_flags[1], ALU_flags[0], muxGOut, AluB, OP[4:0], Cin);
ram512x8 RAM(DataOut, MOC, MOV, R_W, Address, mdrOut, sizeOP);
ConditionTester condition_tester(Cond, FROut[3], FROut[2], FROut[1], FROut[0], muxJOut); //use this one cuando vayas a usar FR
// ConditionTester condition_tester(Cond, ALU_flags[3], ALU_flags[2], ALU_flags[1], ALU_flags[0], IRBus[31:28]); 
shift_sign_extender SASExtender(saseOut, ALU_flags[3], IRBus, PB, FROut[3]);
Adder_4 adder4(adder4Out, IRBus[15:12]);
MultiRegEncoder multireg_encoder(multiencOut, multiregOut);

Multiplexer8x3_4 MuxA(A, IRBus[19:16], IRBus[15:12], number15, adder4Out, multiencOut, MA);
Multiplexer8x3_32 MuxB(AluB, PB, saseOut, mdrOut, noValue_32, multiregOut, MB); 
Multiplexer8x3_4 MuxC(C, IRBus[19:16], IRBus[15:12], number15, adder4Out, multiencOut, MC);
Multiplexer2x1_5 MuxD(OP,{1'b0, IRBus[24:21]}, OP4OP0, MD);
Multiplexer2x1_32 MuxE(muxEOut, DataOut, aluOut, ME);
Multiplexer2x1_4 MuxF(muxFOut, IRBus[3:0],IRBus[19:16], MF);
Multiplexer2x1_32 MuxG(muxGOut, PA, {IRBus[15:0], 16'b0}, MG); //feeds alu input A
// Multiplexer2x1_32 MuxH(muxHOut, IRBus, multiregOut, MH); //feeds sase
Multiplexer2x1_32 MuxI(muxIOut, 32'h10000, aluOut, MI); //feeds multireg
Multiplexer2x1_4 MuxJ(muxJOut, IRBus[31:28], CondTestOp, MJ); //feeds condition tester 
Multiplexer2x1_32 MuxK(muxKOut, aluOut, baseregOut, MK); //feeds PC
// Multiplexer2x1_32 MuxK(muxKOut, baseregOut, aluOut, MK); //feeds PC

MAR Mar(Address, aluOut, MARld, Clk);
MDR Mdr(mdrOut, muxEOut, MDRld, Clk);
FlagRegister FR(FROut, ALU_flags, FRld, Clk); 
InstructionRegister IR(IRBus, DataOut, IRld, Clk);
MultiRegister multireg(multiregOut, muxIOut, MultiRegld, Clk); 
BaseRegister basereg(baseregOut, aluOut, BaseRegld, Clk);
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
  repeat(200) #5 Clk = ~Clk;
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
    // $monitor("IR:%x, Dout:%x, alu:%x, sizeOP:%b, State:%0d, RFld:%b, MA:%b, MB:%b, MC:%b, MD:%b, ME:%b, OP:%b, IRld:%b, MARld:%b, MDRld:%b, RW:%b, MOV:%b, MOC:%b, Cond:%b, Clk:%b, rst:%b, t:%0d", IRBus, DataOut, aluOut, sizeOP, current_state, RFld, MA, MB, MC, MD, ME, OP4OP0, IRld, MARld, MDRld, R_W, MOV, MOC, Cond, Clk, reset, $time);
    $monitor("IR:%x, Dout:%x, alu:%x, sOP:%b, State:%0d, RFld:%b, MA:%b, MB:%b, MC:%b, MD:%b, ME:%b, MJ:%b, OP:%b, MARld:%b, MDRld:%b, RW:%b, MOV:%b, MOC:%b, Cond:%b, Clk:%b, t:%0d", IRBus, DataOut, aluOut, sizeOP, current_state, RFld, MA, MB, MC, MD, ME, MJ, OP4OP0, MARld, MDRld, R_W, MOV, MOC, Cond, Clk, $time);
end

initial begin //initial test instructions
// #551
#1055
    Adr = 7'b0000000; //Address of instruction being tested 
    EfAdr = 13;
    $display("----- Memory contents after running: %h ----- time:%0d",{RAM.Mem[Adr], RAM.Mem[Adr+1], RAM.Mem[Adr+2], RAM.Mem[Adr+3]}, $time);                       

    repeat (16) begin //each address is a byte, so this tells amount of bytes to show 
        #1;
        $display("__RAM_After_Testing: data in address %0d = %x, time: %0d", EfAdr, RAM.Mem[EfAdr], $time);
        #1;
        EfAdr = EfAdr + 1;
        #1;
    end                                     
    $display("----- END TESTING REPORT ----- time:%0d", $time);                                               
end

/////// END INITIALS
endmodule

/////////////// BEGIN CONTROL UNIT
module ControlUnit(output reg [3:0] CondTestOp, output reg BaseRegld, MultiRegld, FRld, RFld, IRld, MARld, MDRld, R_W , MOV, output reg [2:0] MA, MC, MB, output reg [1:0] sizeOP, output reg MD, ME, MF, MG, MI, MJ, MK, output reg [4:0] OP4OP0, output reg [9:0] current_state, input [31:0] IR, input Cond, MOC, reset, clk);

wire [9:0] mux7Out;
wire mux1Out;
wire [9:0] AdderOut;
wire [9:0] EncoderOut;
wire [51:0] CRin;
wire [51:0] CRout;
wire invOut;
wire [9:0] incrementedState;
wire [1:0] M;
wire noValue = 0;
wire [9:0] val1 = 1;
wire [9:0] new_state, curr_state;

always @ (*) begin
    // Cin <= CRout[?];
    BaseRegld <= CRout[51];
    MG <= CRout[50];
    MI <= CRout[49];
    MK <= CRout[48];
    MJ <= CRout[47];
    MultiRegld <= CRout[46];
    CondTestOp <= CRout[45:42];
    MF <= CRout[41];
    FRld <= CRout[40];
    RFld <= CRout[39];
    IRld <= CRout[38];
    MARld <= CRout[37];
    MDRld <= CRout[36];
    R_W <= CRout[35];
    MOV <= CRout[34];
    MA <= CRout[33:31];
    MB <= CRout[30:28];//
    MC <= CRout[27:25];
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
    3'b001: begin // data-processing 32-bit immediate
                if(Instruction[20]==1'b0) //Bit S
                    begin
                        case(Instruction[24:21])
                        //AND
                        4'b0000:    Out = 10'b0111110100;
                        //EOR
                        4'b0001:    Out = 10'b0111111000;
                        //SUB
                        4'b0010:    Out = 10'b0111101001;
                        //RSB
                        4'b0011:    Out = 10'b0111101100;
                        //ADD
                        4'b0100:    Out = 10'b0000000111;
                        //ADC
                        4'b0101:    Out = 10'b0111101110;
                        //SBC
                        4'b0110:    Out = 10'b0111110000;
                        //RSC
                        4'b0111:    Out = 10'b0111110010;
                        //TST
                        4'b1000:    Out = 10'b0111100111;
                        //TEQ
                        4'b1001:    Out = 10'b0111101000;
                        //CMP
                        4'b1010:    Out = 10'b0111100101;
                        //CMN
                        4'b1011:    Out = 10'b0111100110;
                        //ORR
                        4'b1100:    Out = 10'b0111111010;
                        //MOV
                        4'b1101:    Out = 10'b0111100010;
                        //BIC
                        4'b1110:    Out = 10'b0111110110;
                        //MVN
                        4'b1111:    Out = 10'b0111100100;
                        endcase
                    end
                else
                    begin
                        case(Instruction[24:21])
                        //ANDS
                        4'b0000:    Out = 10'b0111110011;
                        //EORS
                        4'b0001:    Out = 10'b0111110111;
                        //SUBS
                        4'b0010:    Out = 10'b0111101010;
                        //RSBS
                        4'b0011:    Out = 10'b0111101011;
                        //ADDS
                        4'b0100:    Out = 10'b0000000101;
                        //ADCS
                        4'b0101:    Out = 10'b0111101101;
                        //SBCS
                        4'b0110:    Out = 10'b0111101111;
                        //RSCS
                        4'b0111:    Out = 10'b0111110001;
                        //TST
                        4'b1000:    Out = 10'b0111100111;
                        //TEQ
                        4'b1001:    Out = 10'b0111101000;
                        //CMP
                        4'b1010:    Out = 10'b0111100101;
                        //CMN
                        4'b1011:    Out = 10'b0111100110;
                        //ORRS
                        4'b1100:    Out = 10'b0111111001;
                        //MOVS
                        4'b1101:    Out = 10'b0111100001;
                        //BICS
                        4'b1110:    Out = 10'b0111110101;
                        //MVNS
                        4'b1111:    Out = 10'b0111100011;
                        endcase
                        
                    end
            end
    3'b000: begin
            //addressing mode 3
            if(Instruction[24]==1'b1 && Instruction[7]==1'b1 && Instruction[4]==1'b1)
                begin
                //strh
                if(Instruction[20]==1'b0 && Instruction[6:5]==2'b01)
                    begin
                        case(Instruction[23:21])
                        // immed add
                            3'b110: Out = 10'b0001111001;
                        //immed sub
                            3'b010: Out = 10'b0001111101;
                        //immed pre-index add
                            3'b111: Out = 10'b0010001001;
                        //immed pre-index sub
                            3'b011: Out = 10'b0010001110;
                        //register offset add
                            3'b100: Out = 10'b0010000001;
                        //register offset sub
                            3'b000: Out = 10'b0010000101;
                        //register pre-index add
                            3'b101: Out = 10'b0010010011;
                        //register pre-index sub
                            3'b001: Out = 10'b0010011000;
                        endcase
                    end
                //STRD
                else if(Instruction[20]==1'b0 && Instruction[6:5]==2'b11)
                    begin
                        case(Instruction[23:21])
                        //immed add
                             3'b110:    Out = 10'b0111111011;
                        //immed sub
                             3'b010:    Out = 10'b1000000011;
                         //immed pre-index add
                            3'b111:     Out = 10'b1000011011;
                        //immed pre-index sub
                            3'b011:     Out = 10'b1000100100;
                        //register offset add
                            3'b100:     Out = 10'b1000001011;
                        //register offset sub
                            3'b000:     Out = 10'b1000010011;
                        //register pre-index add
                            3'b101:     Out = 10'b1000101101;
                        //register pre-index sub
                            3'b001:     Out = 10'b1000110110;
                        endcase
                    end
                //LDRD
                else if(Instruction[20]==1'b0 && Instruction[6:5]==2'b10)
                    begin
                        case(Instruction[23:21])
                        //immed add
                             3'b110:    Out = 10'b1001100011;
                        //immed sub
                             3'b010:    Out = 10'b1001101011;
                         //immed pre-index add
                            3'b111:     Out = 10'b1010000011;
                        //immed pre-index sub
                            3'b011:     Out = 10'b1010001100;
                        //register offset add
                            3'b100:     Out = 10'b1001110011;
                        //register offset sub
                            3'b000:     Out = 10'b1001111011;
                        //register pre-index add
                            3'b101:     Out = 10'b1010010101;
                        //register pre-index sub
                            3'b001:     Out = 10'b1010011110;
                        endcase
                    end
                //ldrh
                else if(Instruction[20]==1'b1 && Instruction[6:5]==2'b01)
                    begin
                        case(Instruction[23:21])
                        //immed add
                             3'b110:    Out = 10'b0011101001;
                        //immed sub
                             3'b010:    Out = 10'b0011101101;
                         //immed pre-index add
                            3'b111:     Out = 10'b0011111001;
                        //immed pre-index sub
                            3'b011:     Out = 10'b0011111110;
                        //register offset add
                            3'b100:     Out = 10'b0011110001;
                        //register offset sub
                            3'b000:     Out = 10'b0011110101;
                        //register pre-index add
                            3'b101:     Out = 10'b0100000011;
                        //register pre-index sub
                            3'b001:     Out = 10'b0100001000;
                        endcase
                    end
                //ldrsb
                else if(Instruction[20]==1'b1 && Instruction[6:5]==2'b10)
                    begin
                        case(Instruction[23:21])
                            //immed add
                                3'b110:     Out = 10'b0101011001;
                            //immed sub
                                3'b010:     Out = 10'b0101011110;
                            //immed pre-index add
                                3'b111:     Out = 10'b0101101101;
                            //immed pre-index sub
                                3'b011:     Out = 10'b0101110011;
                            //register offset add
                                3'b100:     Out = 10'b0101100011;
                            //register offset sub
                                3'b000:     Out = 10'b0101101000;
                            //register pre-index add
                                3'b101:     Out = 10'b0101111001;
                            //register pre-index sub
                                3'b001:     Out = 10'b0101111111;
                        endcase
                    end
                //ldrsh    
                else if(Instruction[20]==1'b1 && Instruction[6:5]==2'b11)
                    begin
                        case(Instruction[23:21])
                            //immed add
                                3'b110:     Out = 10'b0110011101;
                            //immed sub
                                3'b010:     Out = 10'b0110100010;
                            //immed pre-index add
                                3'b111:     Out = 10'b0110110001;
                            //immed pre-index sub
                                3'b011:     Out = 10'b0110110111;
                            //register offset add
                                3'b100:     Out = 10'b0110100111;
                            //register offset sub
                                3'b000:     Out = 10'b0110101100;
                            //register pre-index add
                                3'b101:     Out = 10'b0110111101;
                            //register pre-index sub
                                3'b001:     Out = 10'b0111000011;
                        endcase
                    end
                end
            else if(Instruction[24]==1'b0 && Instruction[7]==1'b1 && Instruction[4]==1'b1)
                begin
                    //strh
                    if(Instruction[20]==1'b0 && Instruction[6:5]==2'b01)
                        begin
                            case(Instruction[23:21])
                                //strh immed post-index add
                                3'b110:     Out = 10'b0010011101;
                                //strh immed post-index sub
                                3'b010:     Out = 10'b0010100010;
                                //strh register post-index add
                                3'b100:     Out = 10'b0010100111;
                                //strh register post-index sub
                                3'b000:     Out = 10'b0010101100;
                            endcase
                        end
                    //STRD
                    else if(Instruction[20]==1'b0 && Instruction[6:5]==2'b11)
                        begin
                            case(Instruction[23:21])
                                //strh immed post-index add
                                3'b110:     Out = 10'b1000111111;
                                //strh immed post-index sub
                                3'b010:     Out = 10'b1001001000;
                                //strh register post-index add
                                3'b100:     Out = 10'b1001010001;
                                //strh register post-index sub
                                3'b000:     Out = 10'b1001011010;
                            endcase
                        end
                    //ldrh
                    else if(Instruction[20]==1'b1 && Instruction[6:5]==2'b01)
                        begin
                           case(Instruction[23:21])
                                //immed post-index add
                                3'b110:     Out = 10'b0100001101;
                                //immed post-index sub
                                3'b010:     Out = 10'b0100010010;
                                //register post-index add
                                3'b100:     Out = 10'b0100010111;
                                //register post-index sub
                                3'b000:     Out = 10'b0100011100;
                            endcase 
                        end
                    //ldrsb
                    else if(Instruction[20]==1'b1 && Instruction[6:5]==2'b10)
                        begin
                            case(Instruction[23:21])
                                //immed post-index add
                                3'b110:     Out = 10'b0110000101;
                                //immed post-index sub
                                3'b010:     Out = 10'b0110001011;
                                //register post-index add
                                3'b100:     Out = 10'b0110010001;
                                //register post-index sub
                                3'b000:     Out = 10'b0110010111;
                            endcase 
                        end
                    //ldrsh
                    else if(Instruction[20]==1'b1 && Instruction[6:5]==2'b11)
                        begin
                            case(Instruction[23:21])
                                //immed post-index add
                                3'b110:     Out = 10'b0111001001;
                                //immed post-index sub
                                3'b010:     Out = 10'b0111001111;
                                //register post-index add
                                3'b100:     Out = 10'b0111010101;
                                //register post-index sub
                                3'b000:  Out = 10'b0111011011;
                            endcase 
                        end
                    //LDRD
                    else if(Instruction[20]==1'b0 && Instruction[6:5]==2'b10)
                        begin
                            case(Instruction[23:21])
                                //immed post-index add
                                3'b110:     Out = 10'b1010100111;
                                //immed post-index sub
                                3'b010:     Out = 10'b1010110000;
                                //register post-index add
                                3'b100:     Out = 10'b1010111001;
                                //register post-index sub
                                3'b000:     Out = 10'b1011000010;
                            endcase 
                        end
                end   
            end

                     
    3'b010: begin   //adressing mode 2 immediate
            case(Instruction[24:20])
                //strb immed
                //immed offset add
                5'b11100:   Out = 10'b0000001000;
                //immed offset sub
                5'b10100:   Out = 10'b0000001100;
                //immed pre-index add
                5'b11110:   Out = 10'b0000011000;
                //immed pre-index sub
                5'b10110:   Out = 10'b0000011101;
                //immed post-index add
                5'b01100:   Out = 10'b0000101100;
                //immed post-index sub
                5'b00100:   Out = 10'b0000110001;
                
                //str immed
                //immed add
                5'b11000:   Out = 10'b0001000001;
                //immed sub
                5'b10000:   Out = 10'b0001000101;
                //pre-index add
                5'b11010:   Out = 10'b0001010001;
                //pre-index sub
                5'b10010:   Out = 10'b0001010110;
                //post-index add
                5'b01000:   Out = 10'b0001100101;
                //post-index sub
                5'b00000:   Out = 10'b0001101010;

                //ldrb immed
                //immed offset add
                5'b11101:   Out = 10'b0010110001;
                //immed offset sub
                5'b10101:   Out = 10'b0010110101;
                //immed pre-index add
                5'b11111:   Out = 10'b0011000001;
                //immed pre-index sub
                5'b10111:   Out = 10'b0011000110;
                //immed post-index add
                5'b01101:   Out = 10'b0011010101;
                //immed post-index sub
                5'b00101:   Out = 10'b0011011010;
                
                //ldr immed
                //immed offset add
                 5'b11001:  Out = 10'b0100100001;
                //immed offset sub
                 5'b10001:  Out = 10'b0100100101;
                 //immed pre-index add
                 5'b11011:  Out = 10'b0100110001;
                 //immed pre-index sub
                5'b10011:   Out = 10'b0100110110;
                  //immed post-index add
                5'b01001:   Out = 10'b0101000101;
                //immed post-index sub
                5'b00001:   Out = 10'b0101001010;


            endcase
            end
    3'b011: begin   //addressing mode 2 register offset
            if(Instruction[11:4]== 8'b00000000)
                begin
                case(Instruction[24:20])
                    //strb register
                    //register add
                    5'b11100:   Out = 10'b0000010000;
                    //register sub
                    5'b10100:   Out = 10'b0000010100; 
                    //register pre-index add
                    5'b11110:   Out = 10'b0000100010;
                    //pre-index sub
                    5'b10110:   Out = 10'b0000100010;
                    //post-index add
                    5'b01100:   Out = 10'b0000110110;
                    //post-index sub 
                    5'b00100:   Out = 10'b0000111011;
                   
                    //str register
                    //register add
                    5'b11000:   Out = 10'b0001001001;
                    //register sub
                    5'b10000:   Out = 10'b0001001101;
                    //pre-index add
                    5'b11010:   Out = 10'b0001011011;
                    //pre-index sub
                    5'b10010:   Out = 10'b0001100000;
                    //post-index add
                    5'b01000:   Out = 10'b0001101111;
                    //post-index sub
                    5'b00000:   Out = 10'b0001110100;

                    //ldrb register
                    //register add
                    5'b11101:   Out = 10'b0010111001;
                    //register sub
                    5'b10101:   Out = 10'b0010111101;
                    //pre-index add
                    5'b11111:   Out = 10'b0011001011;
                    //pre-index sub
                    5'b10111:   Out = 10'b0011010000;
                    //post-index add
                    5'b01101:   Out = 10'b0011011111;
                    //post-index sub
                    5'b00101:   Out = 10'b0011100100;

                    //ldr register
                    //register add
                    5'b11001:   Out = 10'b0100101001;
                    //register sub
                    5'b10001:   Out = 10'b0100101101;
                    //pre-index add
                    5'b11011:   Out = 10'b0100111011;
                    //pre-index sub
                    5'b10011:   Out = 10'b0101000000;
                    //post-index add
                    5'b01001:   Out = 10'b0101001111;
                    //post-index sub
                    5'b00001:   Out = 10'b101010100;
                endcase
                end
            end

    3'b101: Out = 10'b0001000000; //branch instruction
    
    //load/store multiple
    3'b100: 
        begin
            if(Instruction[20]==1'b0) //STR
                begin
                    case (Instruction[24:23])
                        //Increment after
                        2'b01:  if(Instruction[21]==1'b1) //Bit W-permite update a Rn
                                    Out = 10'b1011001011; //estado de permititr update Rn
                                else
                                    Out = 10'b1011010100; //estado de no update a Rn
                        //Increment before            
                        2'b11:  if(Instruction[21]==1'b1)
                                    Out = 10'b0; //estado de update Rn
                                else
                                    Out = 10'b0; //estado de no update Rn
                        //Decrement after
                        2'b00:   if(Instruction[21]==1'b1)
                                    Out = 10'b0; //estado de permitir update Rn
                                else
                                    Out = 10'b0; //estado de no update Rn
                        //Decrement before
                        2'b10:  if(Instruction[21]==1'b1)
                                    Out = 10'b0; //estado de update Rn
                                else
                                    Out = 10'b0; //estado de no update Rn  
                    endcase
                end
            else
                begin
                   //LDR 
                   case (Instruction[24:23])
                        //Increment after
                        2'b01:  if(Instruction[21]==1'b1) //Bit W-permite update a Rn
                                    Out = 10'b0; //estado de permititr update Rn
                                else
                                    Out = 10'b0; //estado de no update a Rn
                        //Increment before            
                        2'b11:  if(Instruction[21]==1'b1)
                                    Out = 10'b0; //estado de update Rn
                                else
                                    Out = 10'b0; //estado de no update Rn
                        //Decrement after
                        2'b00:   if(Instruction[21]==1'b1)
                                    Out = 10'b0; //estado de permitir update Rn
                                else
                                    Out = 10'b0; //estado de no update Rn
                        //Decrement before
                        2'b10:  if(Instruction[21]==1'b1)
                                    Out = 10'b0; //estado de update Rn
                                else
                                    Out = 10'b0; //estado de no update Rn  
                    endcase
                   
                end

        end

    default:    Out = 10'b0000000001;
endcase
end
endmodule

module Microstore (output reg [51:0] out, output reg [9:0] current_state, input reset, input [9:0] next_state);
    //n2n1n0 inv s1s0 moore cr(6)
        parameter[0:52 * 734 - 1] CR_states = { //  cambiar aqui 724 por el numero total de estados que hay 
        52'b0000000000001000000000110101001101000110000000000000, //0
        52'b0000000000000010000100000001010000000110000000000000, //1
        52'b0000000000001000110100000101010001100110000000000000, //2
        52'b0000000000000100110000000000000000001011000000000011, //3
        52'b0000000000000000000000000000000000001000010000000001, //4
        52'b0000000000011000000000010010000100000100000000000001, //5
        52'b0000000000001000000000000000000000000100000000000001, //6
        52'b0000000000001000000000010010000100000100000000000001, //7
        52'b0000000000000010000000010001000100000110000000000000, //8
        52'b0000000000000001000010000001110000000110000000000000, //9
        52'b0000000000000000010000000000000000000110000000000000, //10
        52'b0000000000000000010000000000000000001110000000001011, //11
        52'b0000000000000010000000010001000010000110000000000000, //12
        52'b0000000000000001000010000001110000000110000000000000, //13
        52'b0000000000000000010000000000000000000110000000000000, //14
        52'b0000000000000000010000000000000000001110000000001011, //15
        52'b0000000000000010000000000001000100000110000000000000, //16
        52'b0000000000000001000010000001110000000110000000000000, //17
        52'b0000000000000000010000000000000000000110000000000000, //18
        52'b0000000000000000010000000000000000001110000000010011, //19
        52'b0000000000000010000000000001000010000110000000000000, //20
        52'b0000000000000001000010000001110000000110000000000000, //21
        52'b0000000000000000010000000000000000000110000000000000, //22
        52'b0000000000000000010000000000000000001110000000010111, //23
        52'b0000000000001000000000010001000100000110000000000000, //24
        52'b0000000000000010000000000001010000000110000000000000, //25
        52'b0000000000000001000010000001110000000110000000000000, //26
        52'b0000000000000000010000000000000000000110000000000000, //27
        52'b0000000000000000010000000000000000001110000000011100, //28
        52'b0000000000001000000000010001000010000110000000000000, //29
        52'b0000000000000010000000000001010000000110000000000000, //30
        52'b0000000000000001000010000001110000000110000000000000, //31
        52'b0000000000000000010000000000000000000110000000000000, //32
        52'b0000000000000000010000000000000000001110000000100001, //33
        52'b0000000000001000000000000001000100000110000000000000, //34
        52'b0000000000000010000000000001010000000110000000000000, //35
        52'b0000000000000001000010000001110000000110000000000000, //36
        52'b0000000000000000010000000000000000000110000000000000, //37
        52'b0000000000000000010000000000000000001110000000100110, //38
        52'b0000000000001000000000000001000010000110000000000000, //39
        52'b0000000000000010000000000001010000000110000000000000, //40
        52'b0000000000000001000010000001110000000110000000000000, //41
        52'b0000000000000000010000000000000000000110000000000000, //42
        52'b0000000000000000010000000000000000001110000000101011, //43
        52'b0000000000000010000000000001010000000110000000000000, //44
        52'b0000000000001000000000010001000100000110000000000000, //45
        52'b0000000000000001000010000001110000000110000000000000, //46
        52'b0000000000000000010000000000000000000110000000000000, //47
        52'b0000000000000000010000000000000000001110000000110000, //48
        52'b0000000000000010000000000001010000000110000000000000, //49
        52'b0000000000001000000000010001000010000110000000000000, //50
        52'b0000000000000001000010000001110000000110000000000000, //51
        52'b0000000000000000010000000000000000000110000000000000, //52
        52'b0000000000000000010000000000000000001110000000110101, //53
        52'b0000000000000010000000000001010000000110000000000000, //54
        52'b0000000000001000000000000001000100000110000000000000, //55
        52'b0000000000000001000010000001110000000110000000000000, //56
        52'b0000000000000000010000000000000000000110000000000000, //57
        52'b0000000000000000010000000000000000001110000000111010, //58
        52'b0000000000000010000000000001010000000110000000000000, //59
        52'b0000000000001000000000000001000010000110000000000000, //60
        52'b0000000000000001000010000001110000000110000000000000, //61
        52'b0000000000000000010000000000000000000110000000000000, //62
        52'b0000000000000000010000000000000000001110000000111111, //63
        52'b0000000000001000000100010101010010000100000000000001, //64
        52'b0000000000000010000000010001000100100110000000000000, //65
        52'b0000000000000001010010000001110000100110000000000000, //66
        52'b0000000000000000010000000000000000100110000000000000, //67
        52'b0000000000000000010000000000000000101110000001000100, //68
        52'b0000000000000010000000010001000010100110000000000000, //69
        52'b0000000000000001010010000001110000100110000000000000, //70
        52'b0000000000000000010000000000000000100110000000000000, //71
        52'b0000000000000000010000000000000000101110000001001000, //72
        52'b0000000000000010000000000001000100100110000000000000, //73
        52'b0000000000000001010010000001110000100110000000000000, //74
        52'b0000000000000000010000000000000000100110000000000000, //75
        52'b0000000000000000010000000000000000101110000001001100, //76
        52'b0000000000000010000000000001000010100110000000000000, //77
        52'b0000000000000001010010000001110000100110000000000000, //78
        52'b0000000000000000010000000000000000100110000000000000, //79
        52'b0000000000000000010000000000000000101110000001010000, //80
        52'b0000000000001000000000010001000100100110000000000000, //81
        52'b0000000000000010000000000001010000100110000000000000, //82
        52'b0000000000000001000010000001110000100110000000000000, //83
        52'b0000000000000000010000000000000000100110000000000000, //84
        52'b0000000000000000010000000000000000101110000001010101, //85
        52'b0000000000001000000000010001000010100110000000000000, //86
        52'b0000000000000010000000000001010000100110000000000000, //87
        52'b0000000000000001000010000001110000100110000000000000, //88
        52'b0000000000000000010000000000000000100110000000000000, //89
        52'b0000000000000000010000000000000000101110000001011010, //90
        52'b0000000000001000000000000001000100100110000000000000, //91
        52'b0000000000000010000000000001010000100110000000000000, //92
        52'b0000000000000001000010000001110000100110000000000000, //93
        52'b0000000000000000010000000000000000100110000000000000, //94
        52'b0000000000000000010000000000000000101110000001011111, //95
        52'b0000000000001000000000000001000010100110000000000000, //96
        52'b0000000000000010000000000001010000100110000000000000, //97
        52'b0000000000000001000010000001110000100110000000000000, //98
        52'b0000000000000000010000000000000000100110000000000000, //99
        52'b0000000000000000010000000000000000101110000001100100, //100
        52'b0000000000000010000000000001010000100110000000000000, //101
        52'b0000000000001000000000010001000100100110000000000000, //102
        52'b0000000000000001000010000001110000100110000000000000, //103
        52'b0000000000000000010000000000000000100110000000000000, //104
        52'b0000000000000000010000000000000000101110000001101001, //105
        52'b0000000000000010000000000001010000100110000000000000, //106
        52'b0000000000001000000000010001000010100110000000000000, //107
        52'b0000000000000001000010000001110000100110000000000000, //108
        52'b0000000000000000010000000000000000100110000000000000, //109
        52'b0000000000000000010000000000000000101110000001101110, //110
        52'b0000000000000010000000000001010000100110000000000000, //111
        52'b0000000000001000000000000001000100100110000000000000, //112
        52'b0000000000000001000010000001110000100110000000000000, //113
        52'b0000000000000000010000000000000000100110000000000000, //114
        52'b0000000000000000010000000000000000101110000001110011, //115
        52'b0000000000000010000000000001010000100110000000000000, //116
        52'b0000000000001000000000000001000010100110000000000000, //117
        52'b0000000000000001000010000001110000100110000000000000, //118
        52'b0000000000000000010000000000000000100110000000000000, //119
        52'b0000000000000000010000000000000000101110000001111000, //120
        52'b0000000000000010000000010001000100010110000000000000, //121
        52'b0000000000000001010010000001110000010110000000000000, //122
        52'b0000000000000000010000000000000000010110000000000000, //123
        52'b0000000000000000010000000000000000011110000001111100, //124
        52'b0000000000000010000000010001000010010110000000000000, //125
        52'b0000000000000001010010000001110000010110000000000000, //126
        52'b0000000000000000010000000000000000010110000000000000, //127
        52'b0000000000000000010000000000000000011110000010000000, //128
        52'b0000000000000010000000000001000100010110000000000000, //129
        52'b0000000000000001010010000001110000010110000000000000, //130
        52'b0000000000000000010000000000000000010110000000000000, //131
        52'b0000000000000000010000000000000000011110000010000100, //132
        52'b0000000000000010000000000001000010010110000000000000, //133
        52'b0000000000000001010010000001110000010110000000000000, //134
        52'b0000000000000000010000000000000000010110000000000000, //135
        52'b0000000000000000010000000000000000011110000010001000, //136
        52'b0000000000001000000000010001000100010110000000000000, //137
        52'b0000000000000010000000000001010000010110000000000000, //138
        52'b0000000000000001000010000001110000010110000000000000, //139
        52'b0000000000000000010000000000000000010110000000000000, //140
        52'b0000000000000000010000000000000000011110000010001101, //141
        52'b0000000000001000000000010001000010010110000000000000, //142
        52'b0000000000000010000000000001010000010110000000000000, //143
        52'b0000000000000001000010000001110000010110000000000000, //144
        52'b0000000000000000010000000000000000010110000000000000, //145
        52'b0000000000000000010000000000000000011110000010010010, //146
        52'b0000000000001000000000000001000100010110000000000000, //147
        52'b0000000000000010000000000001010000010110000000000000, //148
        52'b0000000000000001000010000001110000010110000000000000, //149
        52'b0000000000000000010000000000000000010110000000000000, //150
        52'b0000000000000000010000000000000000011110000010010111, //151
        52'b0000000000001000000000000001000010010110000000000000, //152
        52'b0000000000000010000000000001010000010110000000000000, //153
        52'b0000000000000001000010000001110000010110000000000000, //154
        52'b0000000000000000010000000000000000010110000000000000, //155
        52'b0000000000000000010000000000000000011110000010011100, //156
        52'b0000000000000010000000000001010000010110000000000000, //157
        52'b0000000000001000000000010001000100010110000000000000, //158
        52'b0000000000000001000010000001110000010110000000000000, //159
        52'b0000000000000000010000000000000000010110000000000000, //160
        52'b0000000000000000010000000000000000011110000010100001, //161
        52'b0000000000000010000000000001010000010110000000000000, //162
        52'b0000000000001000000000010001000010010110000000000000, //163
        52'b0000000000000001000010000001110000010110000000000000, //164
        52'b0000000000000000010000000000000000010110000000000000, //165
        52'b0000000000000000010000000000000000011110000010100110, //166
        52'b0000000000000010000000000001010000010110000000000000, //167
        52'b0000000000001000000000000001000100010110000000000000, //168
        52'b0000000000000001000010000001110000010110000000000000, //169
        52'b0000000000000000010000000000000000010110000000000000, //170
        52'b0000000000000000010000000000000000011110000010101011, //171
        52'b0000000000000010000000000001010000010110000000000000, //172
        52'b0000000000001000000000000001000010010110000000000000, //173
        52'b0000000000000001000010000001110000010110000000000000, //174
        52'b0000000000000000010000000000000000010110000000000000, //175
        52'b0000000000000000010000000000000000011110000010110000, //176
        52'b0000000000000010000000010001000100000110000000000000, //177
        52'b0000000000000000110000000000000000000110000000000000, //178
        52'b0000000000000001110000000000000000001011000010110011, //179
        52'b0000000000001000000000100011001101000010000000000000, //180
        52'b0000000000000010000000010001000010000110000000000000, //181
        52'b0000000000000000110000000000000000000110000000000000, //182
        52'b0000000000000001110000000000000000001011000010110111, //183
        52'b0000000000001000000000100011001101000010000000000000, //184
        52'b0000000000000010000000000001000100000110000000000000, //185
        52'b0000000000000000110000000000000000000110000000000000, //186
        52'b0000000000000001110000000000000000001011000010111011, //187
        52'b0000000000001000000000100011001101000010000000000000, //188
        52'b0000000000000010000000000001000010000110000000000000, //189
        52'b0000000000000000110000000000000000000110000000000000, //190
        52'b0000000000000001110000000000000000001011000010111111, //191
        52'b0000000000001000000000100011001101000010000000000000, //192
        52'b0000000000001000000000010001000100000110000000000000, //193
        52'b0000000000000010000000000001010000000110000000000000, //194
        52'b0000000000000000110000000000000000000110000000000000, //195
        52'b0000000000000001110000000000000000001011000011000100, //196
        52'b0000000000001000000000100011001101000010000000000000, //197
        52'b0000000000001000000000010001000010000110000000000000, //198
        52'b0000000000000010000000000001010000000110000000000000, //199
        52'b0000000000000000110000000000000000000110000000000000, //200
        52'b0000000000000001110000000000000000001011000011001001, //201
        52'b0000000000001000000000100011001101000010000000000000, //202
        52'b0000000000001000000000000001000100000110000000000000, //203
        52'b0000000000000010000000000001010000000110000000000000, //204
        52'b0000000000000000110000000000000000000110000000000000, //205
        52'b0000000000000001110000000000000000001011000011001110, //206
        52'b0000000000001000000000100011001101000010000000000000, //207
        52'b0000000000001000000000000001000010000110000000000000, //208
        52'b0000000000000010000000000001010000000110000000000000, //209
        52'b0000000000000000110000000000000000000110000000000000, //210
        52'b0000000000000001110000000000000000001011000011010011, //211
        52'b0000000000001000000000100011001101000010000000000000, //212
        52'b0000000000000010000000000001010000000110000000000000, //213
        52'b0000000000001000000000010001000100000110000000000000, //214
        52'b0000000000000000110000000000000000000110000000000000, //215
        52'b0000000000000001110000000000000000001011000011011000, //216
        52'b0000000000001000000000100011001101000010000000000000, //217
        52'b0000000000000010000000000001010000000110000000000000, //218
        52'b0000000000001000000000010001000010000110000000000000, //219
        52'b0000000000000000110000000000000000000110000000000000, //220
        52'b0000000000000001110000000000000000001011000011011101, //221
        52'b0000000000001000000000100011001101000010000000000000, //222
        52'b0000000000000010000000000001010000000110000000000000, //223
        52'b0000000000001000000000000001000100000110000000000000, //224
        52'b0000000000000000110000000000000000000110000000000000, //225
        52'b0000000000000001110000000000000000001011000011100010, //226
        52'b0000000000001000000000100011001101000010000000000000, //227
        52'b0000000000000010000000000001010000000110000000000000, //228
        52'b0000000000001000000000000001000010000110000000000000, //229
        52'b0000000000000000110000000000000000000110000000000000, //230
        52'b0000000000000001110000000000000000001011000011100111, //231
        52'b0000000000001000000000100011001101000010000000000000, //232
        52'b0000000000000010000000010001000100010110000000000000, //233
        52'b0000000000000000110000000000000000010110000000000000, //234
        52'b0000000000000001110000000000000000011011000011101011, //235
        52'b0000000000001000000000100011001101010010000000000000, //236
        52'b0000000000000010000000010001000010010110000000000000, //237
        52'b0000000000000000110000000000000000010110000000000000, //238
        52'b0000000000000001110000000000000000011011000011101111, //239
        52'b0000000000001000000000100011001101010010000000000000, //240
        52'b0000000000000010000000000001000100010110000000000000, //241
        52'b0000000000000000110000000000000000010110000000000000, //242
        52'b0000000000000001110000000000000000011011000011110011, //243
        52'b0000000000001000000000100011001101010010000000000000, //244
        52'b0000000000000010000000000001000010010110000000000000, //245
        52'b0000000000000000110000000000000000010110000000000000, //246
        52'b0000000000000001110000000000000000011011000011110111, //247
        52'b0000000000001000000000100011001101010010000000000000, //248
        52'b0000000000001000000000010001000100010110000000000000, //249
        52'b0000000000000010000000000001010000010110000000000000, //250
        52'b0000000000000000110000000000000000010110000000000000, //251
        52'b0000000000000001110000000000000000011011000011111100, //252
        52'b0000000000001000000000100011001101010010000000000000, //253
        52'b0000000000001000000000010001000010010110000000000000, //254
        52'b0000000000000010000000000001010000010110000000000000, //255
        52'b0000000000000000110000000000000000010110000000000000, //256
        52'b0000000000000001110000000000000000011011000100000001, //257
        52'b0000000000001000000000100011001101010010000000000000, //258
        52'b0000000000001000000000000001000100010110000000000000, //259
        52'b0000000000000010000000000001010000010110000000000000, //260
        52'b0000000000000000110000000000000000010110000000000000, //261
        52'b0000000000000001110000000000000000011011000100000110, //262
        52'b0000000000001000000000100011001101010010000000000000, //263
        52'b0000000000001000000000000001000010010110000000000000, //264
        52'b0000000000000010000000000001010000010110000000000000, //265
        52'b0000000000000000110000000000000000010110000000000000, //266
        52'b0000000000000001110000000000000000011011000100001011, //267
        52'b0000000000001000000000100011001101010010000000000000, //268
        52'b0000000000000010000000000001010000010110000000000000, //269
        52'b0000000000001000000000010001000100010110000000000000, //270
        52'b0000000000000000110000000000000000010110000000000000, //271
        52'b0000000000000001110000000000000000011011000100010000, //272
        52'b0000000000001000000000100011001101010010000000000000, //273
        52'b0000000000000010000000000001010000010110000000000000, //274
        52'b0000000000001000000000010001000010010110000000000000, //275
        52'b0000000000000000110000000000000000010110000000000000, //276
        52'b0000000000000001110000000000000000011011000100010101, //277
        52'b0000000000001000000000100011001101010010000000000000, //278
        52'b0000000000000010000000000001010000010110000000000000, //279
        52'b0000000000001000000000000001000100010110000000000000, //280
        52'b0000000000000000110000000000000000010110000000000000, //281
        52'b0000000000000001110000000000000000011011000100011010, //282
        52'b0000000000001000000000100011001101010010000000000000, //283
        52'b0000000000000010000000000001010000010110000000000000, //284
        52'b0000000000001000000000000001000010010110000000000000, //285
        52'b0000000000000000110000000000000000010110000000000000, //286
        52'b0000000000000001110000000000000000011011000100011111, //287
        52'b0000000000001000000000100011001101010010000000000000, //288
        52'b0000000000000010000000010001000100100110000000000000, //289
        52'b0000000000000000110000000000000000100110000000000000, //290
        52'b0000000000000001110000000000000000101011000100100011, //291
        52'b0000000000001000000000100011001101100010000000000000, //292
        52'b0000000000000010000000010001000010100110000000000000, //293
        52'b0000000000000000110000000000000000100110000000000000, //294
        52'b0000000000000001110000000000000000101011000100100111, //295
        52'b0000000000001000000000100011001101100010000000000000, //296
        52'b0000000000000010000000000001000100100110000000000000, //297
        52'b0000000000000000110000000000000000100110000000000000, //298
        52'b0000000000000001110000000000000000101011000100101011, //299
        52'b0000000000001000000000100011001101100010000000000000, //300
        52'b0000000000000010000000000001000010100110000000000000, //301
        52'b0000000000000000110000000000000000100110000000000000, //302
        52'b0000000000000001110000000000000000101011000100101111, //303
        52'b0000000000001000000000100011001101100010000000000000, //304
        52'b0000000000001000000000010001000100100110000000000000, //305
        52'b0000000000000010000000000001010000100110000000000000, //306
        52'b0000000000000000110000000000000000100110000000000000, //307
        52'b0000000000000001110000000000000000101011000100110100, //308
        52'b0000000000001000000000100011001101100010000000000000, //309
        52'b0000000000001000000000010001000010100110000000000000, //310
        52'b0000000000000010000000000001010000100110000000000000, //311
        52'b0000000000000000110000000000000000100110000000000000, //312
        52'b0000000000000001110000000000000000101011000100111001, //313
        52'b0000000000001000000000100011001101100010000000000000, //314
        52'b0000000000001000000000000001000100100110000000000000, //315
        52'b0000000000000010000000000001010000100110000000000000, //316
        52'b0000000000000000110000000000000000100110000000000000, //317
        52'b0000000000000001110000000000000000101011000100111110, //318
        52'b0000000000001000000000100011001101100010000000000000, //319
        52'b0000000000001000000000000001000010100110000000000000, //320
        52'b0000000000000010000000000001010000100110000000000000, //321
        52'b0000000000000000110000000000000000100110000000000000, //322
        52'b0000000000000001110000000000000000101011000101000011, //323
        52'b0000000000001000000000100011001101100010000000000000, //324
        52'b0000000000000010000000000001010000100110000000000000, //325
        52'b0000000000001000000000010001000100100110000000000000, //326
        52'b0000000000000000110000000000000000100110000000000000, //327
        52'b0000000000000001110000000000000000101011000101001000, //328
        52'b0000000000001000000000100011001101100010000000000000, //329
        52'b0000000000000010000000000001010000100110000000000000, //330
        52'b0000000000001000000000010001000010100110000000000000, //331
        52'b0000000000000000110000000000000000100110000000000000, //332
        52'b0000000000000001110000000000000000101011000101001101, //333
        52'b0000000000001000000000100011001101100010000000000000, //334
        52'b0000000000000010000000000001010000100110000000000000, //335
        52'b0000000000001000000000000001000100100110000000000000, //336
        52'b0000000000000000110000000000000000100110000000000000, //337
        52'b0000000000000001110000000000000000101011000101010010, //338
        52'b0000000000001000000000100011001101100010000000000000, //339
        52'b0000000000000010000000000001010000100110000000000000, //340
        52'b0000000000001000000000000001000010100110000000000000, //341
        52'b0000000000000000110000000000000000100110000000000000, //342
        52'b0000000000000001110000000000000000101011000101010111, //343
        52'b0000000000001000000000100011001101100010000000000000, //344
        52'b0000000000000010000000010001000100000110000000000000, //345
        52'b0000000000000000110000000000000000000110000000000000, //346
        52'b0000000000000001110000000000000000001011000101011011, //347
        52'b0000000000001000000000100011001101000110000000000000, //348
        52'b0000000000101000000000010001001101000010000000000000, //349
        52'b0000000000000010000000010001000010000110000000000000, //350
        52'b0000000000000000110000000000000000000110000000000000, //351
        52'b0000000000000001110000000000000000001011000101100000, //352
        52'b0000000000001000000000100011001101000110000000000000, //353
        52'b0000000000101000000000010001001101000010000000000000, //354
        52'b0000000000000010000000000001000100000110000000000000, //355
        52'b0000000000000000110000000000000000000110000000000000, //356
        52'b0000000000000001110000000000000000001011000101100101, //357
        52'b0000000000001000000000100011001101000110000000000000, //358
        52'b0000000000101000000000010001001101000010000000000000, //359
        52'b0000000000000010000000000001000010000110000000000000, //360
        52'b0000000000000000110000000000000000000110000000000000, //361
        52'b0000000000000001110000000000000000001011000101101010, //362
        52'b0000000000001000000000100011001101000110000000000000, //363
        52'b0000000000101000000000010001001101000010000000000000, //364
        52'b0000000000001000000000010001000100000110000000000000, //365
        52'b0000000000000010000000000001010000000110000000000000, //366
        52'b0000000000000000110000000000000000000110000000000000, //367
        52'b0000000000000001110000000000000000001011000101110000, //368
        52'b0000000000001000000000100011001101000110000000000000, //369
        52'b0000000000101000000000010001001101000010000000000000, //370
        52'b0000000000001000000000010001000010000110000000000000, //371
        52'b0000000000000010000000000001010000000110000000000000, //372
        52'b0000000000000000110000000000000000000110000000000000, //373
        52'b0000000000000001110000000000000000001011000101110110, //374
        52'b0000000000001000000000100011001101000110000000000000, //375
        52'b0000000000101000000000010001001101000010000000000000, //376
        52'b0000000000001000000000000001000100000110000000000000, //377
        52'b0000000000000010000000000001010000000110000000000000, //378
        52'b0000000000000000110000000000000000000110000000000000, //379
        52'b0000000000000001110000000000000000001011000101111100, //380
        52'b0000000000001000000000100011001101000110000000000000, //381
        52'b0000000000101000000000010001001101000010000000000000, //382
        52'b0000000000001000000000000001000010000110000000000000, //383
        52'b0000000000000010000000000001010000000110000000000000, //384
        52'b0000000000000000110000000000000000000110000000000000, //385
        52'b0000000000000001110000000000000000001011000110000010, //386
        52'b0000000000001000000000100011001101000110000000000000, //387
        52'b0000000000101000000000010001001101000010000000000000, //388
        52'b0000000000000010000000000001010000000110000000000000, //389
        52'b0000000000001000000000010001000100000110000000000000, //390
        52'b0000000000000000110000000000000000000110000000000000, //391
        52'b0000000000000001110000000000000000001011000110001000, //392
        52'b0000000000001000000000100011001101000110000000000000, //393
        52'b0000000000101000000000010001001101000010000000000000, //394
        52'b0000000000000010000000000001010000000110000000000000, //395
        52'b0000000000001000000000010001000010000110000000000000, //396
        52'b0000000000000000110000000000000000000110000000000000, //397
        52'b0000000000000001110000000000000000001011000110001110, //398
        52'b0000000000001000000000100011001101000110000000000000, //399
        52'b0000000000101000000000010001001101000010000000000000, //400
        52'b0000000000000010000000000001010000000110000000000000, //401
        52'b0000000000001000000000000001000100000110000000000000, //402
        52'b0000000000000000110000000000000000000110000000000000, //403
        52'b0000000000000001110000000000000000001011000110010100, //404
        52'b0000000000001000000000100011001101000110000000000000, //405
        52'b0000000000101000000000010001001101000010000000000000, //406
        52'b0000000000000010000000000001010000000110000000000000, //407
        52'b0000000000001000000000000001000010000110000000000000, //408
        52'b0000000000000000110000000000000000000110000000000000, //409
        52'b0000000000000001110000000000000000001011000110011010, //410
        52'b0000000000001000000000100011001101000110000000000000, //411
        52'b0000000000101000000000010001001101000010000000000000, //412
        52'b0000000000000010000000010001000100010110000000000000, //413
        52'b0000000000000000110000000000000000010110000000000000, //414
        52'b0000000000000001110000000000000000011011000110011111, //415
        52'b0000000000001000000000100011001101010110000000000000, //416
        52'b0000000000101000000000010001001101010010000000000000, //417
        52'b0000000000000010000000010001000010010110000000000000, //418
        52'b0000000000000000110000000000000000010110000000000000, //419
        52'b0000000000000001110000000000000000011011000110100100, //420
        52'b0000000000001000000000100011001101010110000000000000, //421
        52'b0000000000101000000000010001001101010010000000000000, //422
        52'b0000000000000010000000000001000100010110000000000000, //423
        52'b0000000000000000110000000000000000010110000000000000, //424
        52'b0000000000000001110000000000000000011011000110101001, //425
        52'b0000000000001000000000100011001101010110000000000000, //426
        52'b0000000000101000000000010001001101010010000000000000, //427
        52'b0000000000000010000000000001000010010110000000000000, //428
        52'b0000000000000000110000000000000000010110000000000000, //429
        52'b0000000000000001110000000000000000011011000110101110, //430
        52'b0000000000001000000000100011001101010110000000000000, //431
        52'b0000000000101000000000010001001101010010000000000000, //432
        52'b0000000000001000000000010001000100010110000000000000, //433
        52'b0000000000000010000000000001010000010110000000000000, //434
        52'b0000000000000000110000000000000000010110000000000000, //435
        52'b0000000000000001110000000000000000011011000110110100, //436
        52'b0000000000001000000000100011001101010110000000000000, //437
        52'b0000000000101000000000010001001101010010000000000000, //438
        52'b0000000000001000000000010001000010010110000000000000, //439
        52'b0000000000000010000000000001010000010110000000000000, //440
        52'b0000000000000000110000000000000000010110000000000000, //441
        52'b0000000000000001110000000000000000011011000110111010, //442
        52'b0000000000001000000000100011001101010110000000000000, //443
        52'b0000000000101000000000010001001101010010000000000000, //444
        52'b0000000000001000000000000001000100010110000000000000, //445
        52'b0000000000000010000000000001010000010110000000000000, //446
        52'b0000000000000000110000000000000000010110000000000000, //447
        52'b0000000000000001110000000000000000011011000111000000, //448
        52'b0000000000001000000000100011001101010110000000000000, //449
        52'b0000000000101000000000010001001101010010000000000000, //450
        52'b0000000000001000000000000001000010010110000000000000, //451
        52'b0000000000000010000000000001010000010110000000000000, //452
        52'b0000000000000000110000000000000000010110000000000000, //453
        52'b0000000000000001110000000000000000011011000111000110, //454
        52'b0000000000001000000000100011001101010110000000000000, //455
        52'b0000000000101000000000010001001101010010000000000000, //456
        52'b0000000000000010000000000001010000010110000000000000, //457
        52'b0000000000001000000000010001000100010110000000000000, //458
        52'b0000000000000000110000000000000000010110000000000000, //459
        52'b0000000000000001110000000000000000011011000111001100, //460
        52'b0000000000001000000000100011001101010110000000000000, //461
        52'b0000000000101000000000010001001101010010000000000000, //462
        52'b0000000000000010000000000001010000010110000000000000, //463
        52'b0000000000001000000000010001000010010110000000000000, //464
        52'b0000000000000000110000000000000000010110000000000000, //465
        52'b0000000000000001110000000000000000011011000111010010, //466
        52'b0000000000001000000000100011001101010110000000000000, //467
        52'b0000000000101000000000010001001101010010000000000000, //468
        52'b0000000000000010000000000001010000010110000000000000, //469
        52'b0000000000001000000000000001000100010110000000000000, //470
        52'b0000000000000000110000000000000000010110000000000000, //471
        52'b0000000000000001110000000000000000011011000111011000, //472
        52'b0000000000001000000000100011001101010110000000000000, //473
        52'b0000000000101000000000010001001101010010000000000000, //474
        52'b0000000000000010000000000001010000010110000000000000, //475
        52'b0000000000001000000000000001000010010110000000000000, //476
        52'b0000000000000000110000000000000000010110000000000000, //477
        52'b0000000000000001110000000000000000011011000111011110, //478
        52'b0000000000001000000000100011001101010110000000000000, //479
        52'b0000000000101000000000010001001101010010000000000000, //480
        52'b0000000000011000000000010011001101000010000000000000, //481
        52'b0000000000001000000000010011001101000010000000000000, //482
        52'b0000000000011000000000010011001111000010000000000000, //483
        52'b0000000000001000000000010011001111000010000000000000, //484
        52'b0000000000010000000000010001000010000010000000000000, //485
        52'b0000000000010000000000010001000100000010000000000000, //486
        52'b0000000000010000000000010001000000000010000000000000, //487
        52'b0000000000010000000000010001000001000010000000000000, //488
        52'b0000000000001000000000010011000010000010000000000000, //489
        52'b0000000000011000000000010011000010000010000000000000, //490
        52'b0000000000011000000000010011000011000010000000000000, //491
        52'b0000000000001000000000010011000011000010000000000000, //492
        52'b0000000000011000000000010011000101000010000000000000, //493
        52'b0000000000001000000000010011000101000010000000000000, //494
        52'b0000000000011000000000010011000110000010000000000000, //495
        52'b0000000000001000000000010011000110000010000000000000, //496
        52'b0000000000011000000000010011000111000010000000000000, //497
        52'b0000000000001000000000010011000111000010000000000000, //498
        52'b0000000000011000000000010011000000000010000000000000, //499
        52'b0000000000001000000000010011000000000010000000000000, //500
        52'b0000000000011000000000010011001110000010000000000000, //501
        52'b0000000000001000000000010011001110000010000000000000, //502
        52'b0000000000011000000000010011000001000010000000000000, //503
        52'b0000000000001000000000010011000001000010000000000000, //504
        52'b0000000000011000000000010011001100000010000000000000, //505
        52'b0000000000001000000000010011001100000010000000000000, //506
        52'b0000000000000010000000010001000100110110000000000000, //507
        52'b0000000000000001000010000001110000110110000000000000, //508
        52'b0000000000000000010000000000000000110110000000000000, //509
        52'b0000000000000000010000000000000000111011000111111110, //510
        52'b0000000000000010000000010001010010110110000000000000, //511
        52'b0000000000000001000110000001110000110110000000000000, //512
        52'b0000000000000000010000000000000000110110000000000000, //513
        52'b0000000000000000010000000000000000111110001000000010, //514
        52'b0000000000000010000000010001000010110110000000000000, //515
        52'b0000000000000001000010000001110000110110000000000000, //516
        52'b0000000000000000010000000000000000110110000000000000, //517
        52'b0000000000000000010000000000000000111011001000000110, //518
        52'b0000000000000010000000010001010101110110000000000000, //519
        52'b0000000000000001000110000001110000110110000000000000, //520
        52'b0000000000000000010000000000000000110110000000000000, //521
        52'b0000000000000000010000000000000000111110001000001010, //522
        52'b0000000000000010000000000001000100110110000000000000, //523
        52'b0000000000000001000010000001110000110110000000000000, //524
        52'b0000000000000000010000000000000000110110000000000000, //525
        52'b0000000000000000010000000000000000111011001000001110, //526
        52'b0000000000000010000000000001010010110110000000000000, //527
        52'b0000000000000001000110000001110000110110000000000000, //528
        52'b0000000000000000010000000000000000110110000000000000, //529
        52'b0000000000000000010000000000000000111110001000010010, //530
        52'b0000000000000010000000000001000010110110000000000000, //531
        52'b0000000000000001000010000001110000110110000000000000, //532
        52'b0000000000000000010000000000000000110110000000000000, //533
        52'b0000000000000000010000000000000000111011001000010110, //534
        52'b0000000000000010000000000001010101110110000000000000, //535
        52'b0000000000000001000110000001110000110110000000000000, //536
        52'b0000000000000000010000000000000000110110000000000000, //537
        52'b0000000000000000010000000000000000111110001000011010, //538
        52'b0000000000001000000000010001000100110110000000000000, //539
        52'b0000000000000010000000000001010000110110000000000000, //540
        52'b0000000000000001000010000001110000110110000000000000, //541
        52'b0000000000000000010000000000000000110110000000000000, //542
        52'b0000000000000000010000000000000000111011001000011111, //543
        52'b0000000000000010000000000001010001110110000000000000, //544
        52'b0000000000000001000110000001110000110110000000000000, //545
        52'b0000000000000000010000000000000000110110000000000000, //546
        52'b0000000000000000010000000000000000111110001000100011, //547
        52'b0000000000001000000000010001000010110110000000000000, //548
        52'b0000000000000010000000000001010000110110000000000000, //549
        52'b0000000000000001000010000001110000110110000000000000, //550
        52'b0000000000000000010000000000000000110110000000000000, //551
        52'b0000000000000000010000000000000000111011001000101000, //552
        52'b0000000000000010000000000001010001110110000000000000, //553
        52'b0000000000000001000110000001110000110110000000000000, //554
        52'b0000000000000000010000000000000000110110000000000000, //555
        52'b0000000000000000010000000000000000111110001000101011, //556
        52'b0000000000001000000000000001000100110110000000000000, //557
        52'b0000000000000010000000000001010000110110000000000000, //558
        52'b0000000000000001000010000001110000110110000000000000, //559
        52'b0000000000000000010000000000000000110110000000000000, //560
        52'b0000000000000000010000000000000000111011001000110001, //561
        52'b0000000000000010000000000001010001110110000000000000, //562
        52'b0000000000000001000110000001110000110110000000000000, //563
        52'b0000000000000000010000000000000000110110000000000000, //564
        52'b0000000000000000010000000000000000111110001000110101, //565
        52'b0000000000001000000000000001000010110110000000000000, //566
        52'b0000000000000010000000000001010000110110000000000000, //567
        52'b0000000000000001000010000001110000110110000000000000, //568
        52'b0000000000000000010000000000000000110110000000000000, //569
        52'b0000000000000000010000000000000000111011001000111010, //570
        52'b0000000000000010000000000001010001110110000000000000, //571
        52'b0000000000000001000110000001110000110110000000000000, //572
        52'b0000000000000000010000000000000000110110000000000000, //573
        52'b0000000000000000010000000000000000111110001000111110, //574
        52'b0000000000000010000000000001010000110110000000000000, //575
        52'b0000000000000001000010000001110000110110000000000000, //576
        52'b0000000000000000010000000000000000110110000000000000, //577
        52'b0000000000000000010000000000000000111011001001000010, //578
        52'b0000000000000010000000000001010001110110000000000000, //579
        52'b0000000000000001000110000001110000110110000000000000, //580
        52'b0000000000000000010000000000000000110110000000000000, //581
        52'b0000000000000000010000000000000000111011001001000110, //582
        52'b0000000000001000000010010011000100110010000000000000, //583
        52'b0000000000000010000000000001010000110110000000000000, //584
        52'b0000000000000001000010000001110000110110000000000000, //585
        52'b0000000000000000010000000000000000110110000000000000, //586
        52'b0000000000000000010000000000000000111011001001001011, //587
        52'b0000000000000010000000000001010001110110000000000000, //588
        52'b0000000000000001000110000001110000110110000000000000, //589
        52'b0000000000000000010000000000000000110110000000000000, //590
        52'b0000000000000000010000000000000000111011001001001111, //591
        52'b0000000000001000000010010011000010110010000000000000, //592
        52'b0000000000000010000000000001010000110110000000000000, //593
        52'b0000000000000001000010000001110000110110000000000000, //594
        52'b0000000000000000010000000000000000110110000000000000, //595
        52'b0000000000000000010000000000000000111011001001010100, //596
        52'b0000000000000010000000000001010001110110000000000000, //597
        52'b0000000000000001000110000001110000110110000000000000, //598
        52'b0000000000000000010000000000000000110110000000000000, //599
        52'b0000000000000000010000000000000000111011001001011000, //600
        52'b0000000000001000000010000011000100110010000000000000, //601
        52'b0000000000000010000000000001010000110110000000000000, //602
        52'b0000000000000001000010000001110000110110000000000000, //603
        52'b0000000000000000010000000000000000110110000000000000, //604
        52'b0000000000000000010000000000000000111011001001011101, //605
        52'b0000000000000010000000000001010001110110000000000000, //606
        52'b0000000000000001000110000001110000110110000000000000, //607
        52'b0000000000000000010000000000000000110110000000000000, //608
        52'b0000000000000000010000000000000000111011001001100001, //609
        52'b0000000000001000000010000011000010110010000000000000, //610
        52'b0000000000000010000000010001000100110110000000000000, //611
        52'b0000000000000000110000000000000000110110000000000000, //612
        52'b0000000000000001110000000000000000111011001001100101, //613
        52'b0000000000001000000000100011001101110110000000000000, //614
        52'b0000000000000010000000010001010010110110000000000000, //615
        52'b0000000000000000110000000000000000110110000000000000, //616
        52'b0000000000000001110000000000000000111011001001101001, //617
        52'b0000000000001000000000100111001101110010000000000000, //618
        52'b0000000000000010000000010001000010110110000000000000, //619
        52'b0000000000000000110000000000000000110110000000000000, //620
        52'b0000000000000001110000000000000000111011001001101101, //621
        52'b0000000000001000000000100011001101110110000000000000, //622
        52'b0000000000000010000000010001010101110110000000000000, //623
        52'b0000000000000000110000000000000000110110000000000000, //624
        52'b0000000000000001110000000000000000111011001001110001, //625
        52'b0000000000001000000000100111001101110010000000000000, //626
        52'b0000000000000010000000000001000100110110000000000000, //627
        52'b0000000000000000110000000000000000110110000000000000, //628
        52'b0000000000000001110000000000000000111011001001110101, //629
        52'b0000000000001000000000100011001101110110000000000000, //630
        52'b0000000000000010000000000001010010110110000000000000, //631
        52'b0000000000000000110000000000000000110110000000000000, //632
        52'b0000000000000001110000000000000000111011001001111001, //633
        52'b0000000000001000000000100111001101110010000000000000, //634
        52'b0000000000000010000000000001000010110110000000000000, //635
        52'b0000000000000000110000000000000000110110000000000000, //636
        52'b0000000000000001110000000000000000111011001001111101, //637
        52'b0000000000001000000000100011001101110110000000000000, //638
        52'b0000000000000010000000000001010101110110000000000000, //639
        52'b0000000000000000110000000000000000110110000000000000, //640
        52'b0000000000000001110000000000000000111011001010000001, //641
        52'b0000000000001000000000100111001101110010000000000000, //642
        52'b0000000000001000000000010001000100110110000000000000, //643
        52'b0000000000000010000000000001010000110110000000000000, //644
        52'b0000000000000000110000000000000000110110000000000000, //645
        52'b0000000000000001110000000000000000111011001010000110, //646
        52'b0000000000001000000000100011001101110110000000000000, //647
        52'b0000000000000010000000000001010001110110000000000000, //648
        52'b0000000000000000110000000000000000110110000000000000, //649
        52'b0000000000000001110000000000000000111011001010001010, //650
        52'b0000000000001000000000100111001101110010000000000000, //651
        52'b0000000000001000000000010001000010110110000000000000, //652
        52'b0000000000000010000000000001010000110110000000000000, //653
        52'b0000000000000000110000000000000000110110000000000000, //654
        52'b0000000000000001110000000000000000111011001010001111, //655
        52'b0000000000001000000000100011001101110110000000000000, //656
        52'b0000000000000010000000000001010001110110000000000000, //657
        52'b0000000000000000110000000000000000110110000000000000, //658
        52'b0000000000000001110000000000000000111011001010010011, //659
        52'b0000000000001000000000100111001101110010000000000000, //660
        52'b0000000000001000000000000001000100110110000000000000, //661
        52'b0000000000000010000000000001010000110110000000000000, //662
        52'b0000000000000000110000000000000000110110000000000000, //663
        52'b0000000000000001110000000000000000111011001010011000, //664
        52'b0000000000001000000000100011001101110110000000000000, //665
        52'b0000000000000010000000000001010001110110000000000000, //666
        52'b0000000000000000110000000000000000110110000000000000, //667
        52'b0000000000000001110000000000000000111011001010011100, //668
        52'b0000000000001000000000100111001101110010000000000000, //669
        52'b0000000000001000000000000001000010110110000000000000, //670
        52'b0000000000000010000000000001010000110110000000000000, //671
        52'b0000000000000000110000000000000000110110000000000000, //672
        52'b0000000000000001110000000000000000111011001010100001, //673
        52'b0000000000001000000000100011001101110110000000000000, //674
        52'b0000000000000010000000000001010001110110000000000000, //675
        52'b0000000000000000110000000000000000110110000000000000, //676
        52'b0000000000000001110000000000000000111011001010100101, //677
        52'b0000000000001000000000100111001101110010000000000000, //678
        52'b0000000000000010000000000001010000110110000000000000, //679
        52'b0000000000000000110000000000000000110110000000000000, //680
        52'b0000000000000001110000000000000000111011001010101001, //681
        52'b0000000000001000000000100011001101110110000000000000, //682
        52'b0000000000000010000000000001010001110110000000000000, //683
        52'b0000000000000000110000000000000000110110000000000000, //684
        52'b0000000000000001110000000000000000111011001010101101, //685
        52'b0000000000001000000000100111001101110110000000000000, //686
        52'b0000000000001000000000010001000100110010000000000000, //687
        52'b0000000000000010000000000001010000110110000000000000, //688
        52'b0000000000000000110000000000000000110110000000000000, //689
        52'b0000000000000001110000000000000000111011001010110010, //690
        52'b0000000000001000000000100011001101110110000000000000, //691
        52'b0000000000000010000000000001010001110110000000000000, //692
        52'b0000000000000000110000000000000000110110000000000000, //693
        52'b0000000000000001110000000000000000111011001010110110, //694
        52'b0000000000001000000000100111001101110110000000000000, //695
        52'b0000000000001000000000010001000010110010000000000000, //696
        52'b0000000000000010000000000001010000110110000000000000, //697
        52'b0000000000000000110000000000000000110110000000000000, //698
        52'b0000000000000001110000000000000000111011001010111011, //699
        52'b0000000000001000000000100011001101110110000000000000, //700
        52'b0000000000000010000000000001010001110110000000000000, //701
        52'b0000000000000000110000000000000000110110000000000000, //702
        52'b0000000000000001110000000000000000111011001010111111, //703
        52'b0000000000001000000000100111001101110110000000000000, //704
        52'b0000000000001000000000000001000100110010000000000000, //705
        52'b0000000000000010000000000001010000110110000000000000, //706
        52'b0000000000000000110000000000000000110110000000000000, //707
        52'b0000000000000001110000000000000000111011001011000100, //708
        52'b0000000000001000000000100011001101110110000000000000, //709
        52'b0000000000000010000000000001010001110110000000000000, //710
        52'b0000000000000000110000000000000000110110000000000000, //711
        52'b0000000000000001110000000000000000111011001011001000, //712
        52'b0000000000001000000000100111001101110110000000000000, //713
        52'b0000000000001000000000000001000010110010000000000000, //714
        52'b0000010001000010000000000001010000100110000000000000, //715
        52'b0100000000010000000001000001000000100110000000000000, //716
        52'b0000100000000000000000000000000000101011011011010000, //717
        52'b0010110000010000000001000001010110100110000000000000, //718
        52'b0000100000000000000000000000000000101110011011001100, //719
        52'b0000000000000001001000000001110000100110000000000000, //720
        52'b0000000000000000010000000000000000100110000000000000, //721
        52'b0000000000000000010000000000000000101011001011010010, //722
        52'b0000000000001010000000000001010001100100001011001110, //723
        52'b1000010000000010000000000001010000100110000000000000, //724
        52'b0100000000010000000001000001000000100110000000000000, //725
        52'b0000100000000000000000000000000000101010011011011011, //726
        52'b0000000000000001001000000001010000100110000000000000, //727
        52'b0000000000000000010000000000000000100110000000000000, //728
        52'b0000000000000000010000000000000000101011001011011001, //729
        52'b0000000000001010000000000001010001100110000000000000, //730
        52'b0010110000010000000001000001010110100110000000000000, //731
        52'b0000100000000000000000000000000000001011011011010101, //732
        52'b0001000000001000000000000000000000000010000000000000 //733

       };

always @(next_state, reset)
begin
    if (reset) begin
        out           <= CR_states[0+:52];
        current_state <= 10'd0;
    end
    else begin
        out           <= CR_states[52*next_state+:52];
        current_state <= next_state;
    end
end
endmodule

module ControlRegister(output reg [51:0] Qs, output reg [9:0] current_state, input Clk, input [51:0] Ds, input [9:0] next_state); 
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
    5'b10010:   Out = A + B + 4;
    5'b10011:  Out = A >>1; // right shift
    5'b10100:  Out = A <<1; // left shift
    5'b10101:   Out = A - B + 4;
    5'b10110: Out = B << 1;
    endcase
    
     Zero = (~|Out); //bitwise or
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
    $display("__R15: %b, Clock:%b, t:%0d\n__R1: %b, Clock:%b, t:%0d\n__R2: %b, Clock:%b, t:%0d\n__R3: %b, Clock:%b, t:%0d", I15, clk, $time, I1, clk, $time, I2, clk, $time, I3, clk, $time);
end

initial begin 
#1060
    $display("------- Testing Report: Register File Contents ----------- t:%0d", $time);
    $display("__R15: %b, Clock:%b, t:%0d\n__R1: %b, Clock:%b, t:%0d\n__R2: %b, Clock:%b, t:%0d\n__R3: %b, Clock:%b, t:%0d", I15, clk, $time, I1, clk, $time, I2, clk, $time, I3, clk, $time);
    $display("------- Ends Testing Report: Register File Contents ------ t:%0d", $time);
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
// module Multiplexer4x2_32(output reg [31:0] Q, input [31:0] I0, I1, I2, I3, input [1:0] S);
    
//     always @ (*)
//     begin
//         case(S)
//             4'h0: Q <= I0;
//             4'h1: Q <= I1;
//             4'h2: Q <= I2;
//             4'h3: Q <= I3;
//         endcase
//         end
    
// endmodule 

//this one's used for MuxA and MuxC
// module Multiplexer8x3_4(output reg [3:0] Q, input [3:0] I0, I1, I2, I3, I4, I5, I6, I7, input [1:0] S);
module Multiplexer8x3_4(output reg [3:0] Q, input [3:0] I0, I1, I2, I3, I4, input [2:0] S); //using this one hasta que tengamos que usar las otras entradas
    
    always @ (*)
    begin
        case(S)
            4'h0: Q <= I0;
            4'h1: Q <= I1;
            4'h2: Q <= I2;
            4'h3: Q <= I3;
            4'h4: Q <= I4;
        endcase
        end
    
endmodule

//this one's used for MuxB
// module Multiplexer8x3_32(output reg [31:0] Q, input [31:0] I0, I1, I2, I3, I4, I5, I6, I7, input [1:0] S);
module Multiplexer8x3_32(output reg [31:0] Q, input [31:0] I0, I1, I2, I3, I4, input [2:0] S); //using this one hasta que tengamos que usar las otras entradas
    
    always @ (*)
    begin
        case(S)
            4'h0: Q <= I0;
            4'h1: Q <= I1;
            4'h2: Q <= I2;
            4'h3: Q <= I3;
            4'h4: Q <= I4;
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

//this one is for MuxF, MuxJ
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
        $display("__MuxE: out:%b, in0:%b, in1:%b", Q, I0, I1);
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

module FlagRegister(output reg [3:0] Q, input [3:0] D, input LE, Clk); //add Cin here
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

module MultiRegister(output reg [31:0] Q, input [31:0] D, input LE, Clk);
always @(posedge Clk) begin
    if(LE) begin 
        Q <= D;
    end
    $display("__MultiRegister: multiregOut:%b, t:%0d", Q, $time);
end
endmodule

module BaseRegister(output reg [31:0] Q, input [31:0] D, input LE, Clk);
always @(posedge Clk) begin
    if(LE) begin 
        Q <= D;
    end
    $display("__BaseRegister: baseregOut:%b, t:%0d", Q, $time);
end
endmodule
///////////////// END REGISTERS

///////////////// BEGIN RAM
module ram512x8(output reg [31:0] DataOut, output reg MOC, input Enable, input ReadWrite, input [31:0] Address, input [31:0] DataIn, input [1:0] OpCode);

  reg [7:0] Mem[0:511]; //512 localizaciones de 8 bits
  always @ (Enable, ReadWrite) begin
    MOC <= 0; //MOC <= 0;
    $display("__RAM: entered the always, MOC:%b, MOV:%b, RW:%b, Adr:%b, DataIn:%b, t:0%d", MOC, Enable, ReadWrite, Address, DataIn, $time);
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
                    $display("__RAM: wrote a byte Adr:%0d, Mem[%0d]:%b", Address, Address, Mem[Address]);
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
    //immediate shifter operand
    case (instruction[27:25])
    3'b001: 
        begin
            if(instruction[11:8]==4'b0000)
                begin
                extender_out = instruction[7:0];
                carry = Cin;
                end
            else
                begin
                    temp = instruction[7:0];
                    extender_out = {temp,temp} >>(2*{instruction[11:8]});
                    carry = extender_out[31];
                end
        end 
    //shift by immediate shifter opperand
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
                         extender_out =  B >> instruction[11:7];
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
    //addressing mode 2
    //immediate offset/pre/post
    3'b010: begin
            if((instruction[24]==1'b1 && instruction[21]==1'b0) || (instruction[24]==1'b1 && instruction[21]==1'b1) || (instruction[24]==1'b0 && instruction[21]==1'b0))
                extender_out = instruction[11:0];
            end
    //register offset/pre/post
    3'b011: begin
                if(instruction[11:4]==8'b00000000)
                    begin
                       if((instruction[24]==1'b1 && instruction[21]==1'b0) || (instruction[24]==1'b1 && instruction[21]==1'b1) || (instruction[24]==1'b0 && instruction[21]==1'b0))
                            extender_out = B; 
                    end
            end
    endcase
        

end           
endmodule
////////////// END SHIFTER SIGN EXTENDER
////////////// BEGIN ADDER
module Adder_4(output reg [3:0] out, input [3:0] in);
always @(in)
    out <= in + 1'b1;
// $display("__Adder input: %d, Adder output: %d", in, out); ---
endmodule
////////////// END ADDER
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

