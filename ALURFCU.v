module LetsGo;

////////// Instructions 
// E5 C1 20 E4 - 32'b1110_010_11100_0001_0010_000011100100; //estado 8 STRB R2,[R1,#228] --> 11 in 241 // it works 
// E08120E2 - 32'b1110_000_0100_0_0110_0100_00001110_0100; //estado 5 - ADD R2,R1,R2,ROR #1 -> 001011 --> ROR #1: 000101; 5 + 13 = 18 --> 10010 in r2 
// E2 8F 10 09 - 32'b<bits for instruction>; //estado 7 - ADD R1, R15, #0d9 //temp --> R1 = 13        // it works 
// E2 8F 20 03 - 32'b<bits for instruction>; //estado 7 - ADD R2, R15, #d3 //temp --> R2 = 11         // it works 
// E2 81 30 02 - /////////////////////////// //estado 7 - ADD R3, R1, #2 --> R3 = 15
// E7 C6 40 04 - 32'b1110_011_11100_0110_0100_000000000100;// estado 16 STRB register offset ADD
// EA C6 40 E4 - 32'b1110_101_01100_0110_0100_000011100100; // estado 64 Branch instruction
//  E5 81 20 07 --> 507, STRD r2, [r1, #7]
//  testing: (r2)11 --> 23, (r3)15 --> 27
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

//Adder_4 wires
wire [3:0] adder4Out;

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
Adder_4 adder4(adder4Out, IRBus[15:12]);

Multiplexer4x2_4 MuxA(A,IRBus[19:16],IRBus[15:12],number15,adder4Out, MA);
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
  repeat(60) #5 Clk = ~Clk;
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
    $monitor("IR:%x, Dout:%x, alu:%x, sizeOP:%b, State:%0d, RFld:%b, MA:%b, MB:%b, MC:%b, MD:%b, ME:%b, OP:%b, IRld:%b, MARld:%b, MDRld:%b, RW:%b, MOV:%b, MOC:%b, Cond:%b, Clk:%b, rst:%b, t:%0d", IRBus, DataOut, aluOut, sizeOP, current_state, RFld, MA, MB, MC, MD, ME, OP4OP0, IRld, MARld, MDRld, R_W, MOV, MOC, Cond, Clk, reset, $time);
end

initial begin //initial test instructions
#551
    Adr = 7'b0000000; //Address of instruction being tested 
    EfAdr = 13 + 7;
    $display("----- Memory contents after running: %h ----- time:%0d",{RAM.Mem[Adr], RAM.Mem[Adr+1], RAM.Mem[Adr+2], RAM.Mem[Adr+3]}, $time);                       

    repeat (8) begin //each address is a byte, so this tells amount of bytes to show 
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
$display("InstructionEncoder: %b", Out, $time);
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

module Microstore (output reg [38:0] out, output reg [9:0] current_state, input reset, input [9:0] next_state);
    //n2n1n0 inv s1s0 moore cr(6)
        parameter[0:39 * 539 - 1] CR_states = { //cambiar aqui 345 por el numero de estados que hay 
        39'b001000000011101001101000110000000000000, //0
        39'b000010001000001010000000110000000000000, //1
        39'b001000111000101010001100110000000000000, //2
        39'b000100110000000000000001011000000000011, //3
        39'b000000000000000000000001000010000000001, //4
        39'b001000000001010000100000100000000000001, //5
        39'b001000000000000000000000100000000000001, //6
        39'b001000000001010000100100100000000000001, //7
        39'b000010000001001000100000110000000000000, //8
        39'b000001000100001110000000110000000000000, //9
        39'b000000010000000000000000110000000000000, //10
        39'b000000010000000000000001110000000001011, //11
        39'b000010000001001000010000110000000000000, //12
        39'b000001000100001110000000110000000000000, //13
        39'b000000010000000000000000110000000000000, //14
        39'b000000010000000000000001110000000001011, //15
        39'b000010000000001000100000110000000000000, //16
        39'b000001000100001110000000110000000000000, //17
        39'b000000010000000000000000110000000000000, //18
        39'b000000010000000000000001110000000010011, //19
        39'b000010000000001000010000110000000000000, //20
        39'b000001000100001110000000110000000000000, //21
        39'b000000010000000000000000110000000000000, //22
        39'b000000010000000000000001110000000010111, //23
        39'b001000000001001000100000110000000000000, //24
        39'b000010000000001010000000110000000000000, //25
        39'b000001000100001110000000110000000000000, //26
        39'b000000010000000000000000110000000000000, //27
        39'b000000010000000000000001110000000011100, //28
        39'b001000000001001000010000110000000000000, //29
        39'b000010000000001010000000110000000000000, //30
        39'b000001000100001110000000110000000000000, //31
        39'b000000010000000000000000110000000000000, //32
        39'b000000010000000000000001110000000100001, //33
        39'b001000000000001000100000110000000000000, //34
        39'b000010000000001010000000110000000000000, //35
        39'b000001000100001110000000110000000000000, //36
        39'b000000010000000000000000110000000000000, //37
        39'b000000010000000000000001110000000100110, //38
        39'b001000000000001000010000110000000000000, //39
        39'b000010000000001010000000110000000000000, //40
        39'b000001000100001110000000110000000000000, //41
        39'b000000010000000000000000110000000000000, //42
        39'b000000010000000000000001110000000101011, //43
        39'b000010000000001010000000110000000000000, //44
        39'b001000000001001000100000110000000000000, //45
        39'b000001000100001110000000110000000000000, //46
        39'b000000010000000000000000110000000000000, //47
        39'b000000010000000000000001110000000110000, //48
        39'b000010000000001010000000110000000000000, //49
        39'b001000000001001000010000110000000000000, //50
        39'b000001000100001110000000110000000000000, //51
        39'b000000010000000000000000110000000000000, //52
        39'b000000010000000000000001110000000110101, //53
        39'b000010000000001010000000110000000000000, //54
        39'b001000000000001000100000110000000000000, //55
        39'b000001000100001110000000110000000000000, //56
        39'b000000010000000000000000110000000000000, //57
        39'b000000010000000000000001110000000111010, //58
        39'b000010000000001010000000110000000000000, //59
        39'b001000000000001000010000110000000000000, //60
        39'b000001000100001110000000110000000000000, //61
        39'b000000010000000000000000110000000000000, //62
        39'b000000010000000000000001110000000111111, //63
        39'b001000001001101010010000100000000000001, //64
        39'b000010000001001000100100110000000000000, //65
        39'b000001010100001110000100110000000000000, //66
        39'b000000010000000000000100110000000000000, //67
        39'b000000010000000000000101110000001000100, //68
        39'b000010000001001000010100110000000000000, //69
        39'b000001010100001110000100110000000000000, //70
        39'b000000010000000000000100110000000000000, //71
        39'b000000010000000000000101110000001001000, //72
        39'b000010000000001000100100110000000000000, //73
        39'b000001010100001110000100110000000000000, //74
        39'b000000010000000000000100110000000000000, //75
        39'b000000010000000000000101110000001001100, //76
        39'b000010000000001000010100110000000000000, //77
        39'b000001010100001110000100110000000000000, //78
        39'b000000010000000000000100110000000000000, //79
        39'b000000010000000000000101110000001010000, //80
        39'b001000000001001000100100110000000000000, //81
        39'b000010000000001010000100110000000000000, //82
        39'b000001000100001110000100110000000000000, //83
        39'b000000010000000000000100110000000000000, //84
        39'b000000010000000000000101110000001010101, //85
        39'b001000000001001000010100110000000000000, //86
        39'b000010000000001010000100110000000000000, //87
        39'b000001000100001110000100110000000000000, //88
        39'b000000010000000000000100110000000000000, //89
        39'b000000010000000000000101110000001011010, //90
        39'b001000000000001000100100110000000000000, //91
        39'b000010000000001010000100110000000000000, //92
        39'b000001000100001110000100110000000000000, //93
        39'b000000010000000000000100110000000000000, //94
        39'b000000010000000000000101110000001011111, //95
        39'b001000000000001000010100110000000000000, //96
        39'b000010000000001010000100110000000000000, //97
        39'b000001000100001110000100110000000000000, //98
        39'b000000010000000000000100110000000000000, //99
        39'b000000010000000000000101110000001100100, //100
        39'b000010000000001010000100110000000000000, //101
        39'b001000000001001000100100110000000000000, //102
        39'b000001000100001110000100110000000000000, //103
        39'b000000010000000000000100110000000000000, //104
        39'b000000010000000000000101110000001101001, //105
        39'b000010000000001010000100110000000000000, //106
        39'b001000000001001000010100110000000000000, //107
        39'b000001000100001110000100110000000000000, //108
        39'b000000010000000000000100110000000000000, //109
        39'b000000010000000000000101110000001101110, //110
        39'b000010000000001010000100110000000000000, //111
        39'b001000000000001000100100110000000000000, //112
        39'b000001000100001110000100110000000000000, //113
        39'b000000010000000000000100110000000000000, //114
        39'b000000010000000000000101110000001110011, //115
        39'b000010000000001010000100110000000000000, //116
        39'b001000000000001000010100110000000000000, //117
        39'b000001000100001110000100110000000000000, //118
        39'b000000010000000000000100110000000000000, //119
        39'b000000010000000000000101110000001111000, //120
        39'b000010000001001000100010110000000000000, //121
        39'b000001010100001110000010110000000000000, //122
        39'b000000010000000000000010110000000000000, //123
        39'b000000010000000000000011110000001111100, //124
        39'b000010000001001000010010110000000000000, //125
        39'b000001010100001110000010110000000000000, //126
        39'b000000010000000000000010110000000000000, //127
        39'b000000010000000000000011110000010000000, //128
        39'b000010000000001000100010110000000000000, //129
        39'b000001010100001110000010110000000000000, //130
        39'b000000010000000000000010110000000000000, //131
        39'b000000010000000000000011110000010000100, //132
        39'b000010000000001000010010110000000000000, //133
        39'b000001010100001110000010110000000000000, //134
        39'b000000010000000000000010110000000000000, //135
        39'b000000010000000000000011110000010001000, //136
        39'b001000000001001000100010110000000000000, //137
        39'b000010000000001010000010110000000000000, //138
        39'b000001000100001110000010110000000000000, //139
        39'b000000010000000000000010110000000000000, //140
        39'b000000010000000000000011110000010001101, //141
        39'b001000000001001000010010110000000000000, //142
        39'b000010000000001010000010110000000000000, //143
        39'b000001000100001110000010110000000000000, //144
        39'b000000010000000000000010110000000000000, //145
        39'b000000010000000000000011110000010010010, //146
        39'b001000000000001000100010110000000000000, //147
        39'b000010000000001010000010110000000000000, //148
        39'b000001000100001110000010110000000000000, //149
        39'b000000010000000000000010110000000000000, //150
        39'b000000010000000000000011110000010010111, //151
        39'b001000000000001000010010110000000000000, //152
        39'b000010000000001010000010110000000000000, //153
        39'b000001000100001110000010110000000000000, //154
        39'b000000010000000000000010110000000000000, //155
        39'b000000010000000000000011110000010011100, //156
        39'b000010000000001010000010110000000000000, //157
        39'b001000000001001000100010110000000000000, //158
        39'b000001000100001110000010110000000000000, //159
        39'b000000010000000000000010110000000000000, //160
        39'b000000010000000000000011110000010100001, //161
        39'b000010000000001010000010110000000000000, //162
        39'b001000000001001000010010110000000000000, //163
        39'b000001000100001110000010110000000000000, //164
        39'b000000010000000000000010110000000000000, //165
        39'b000000010000000000000011110000010100110, //166
        39'b000010000000001010000010110000000000000, //167
        39'b001000000000001000100010110000000000000, //168
        39'b000001000100001110000010110000000000000, //169
        39'b000000010000000000000010110000000000000, //170
        39'b000000010000000000000011110000010101011, //171
        39'b000010000000001010000010110000000000000, //172
        39'b001000000000001000010010110000000000000, //173
        39'b000001000100001110000010110000000000000, //174
        39'b000000010000000000000010110000000000000, //175
        39'b000000010000000000000011110000010110000, //176
        39'b000010000001001000100000110000000000000, //177
        39'b000000110000000000000000110000000000000, //178
        39'b000001110000000000000001011000010110011, //179
        39'b001000000010011001101000010000000000000, //180
        39'b000010000001001000010000110000000000000, //181
        39'b000000110000000000000000110000000000000, //182
        39'b000001110000000000000001011000010110111, //183
        39'b001000000010011001101000010000000000000, //184
        39'b000010000000001000100000110000000000000, //185
        39'b000000110000000000000000110000000000000, //186
        39'b000001110000000000000001011000010111011, //187
        39'b001000000010011001101000010000000000000, //188
        39'b000010000000001000010000110000000000000, //189
        39'b000000110000000000000000110000000000000, //190
        39'b000001110000000000000001011000010111111, //191
        39'b001000000010011001101000010000000000000, //192
        39'b001000000001001000100000110000000000000, //193
        39'b000010000000001010000000110000000000000, //194
        39'b000000110000000000000000110000000000000, //195
        39'b000001110000000000000001011000011000100, //196
        39'b001000000010011001101000010000000000000, //197
        39'b001000000001001000010000110000000000000, //198
        39'b000010000000001010000000110000000000000, //199
        39'b000000110000000000000000110000000000000, //200
        39'b000001110000000000000001011000011001001, //201
        39'b001000000010011001101000010000000000000, //202
        39'b001000000000001000100000110000000000000, //203
        39'b000010000000001010000000110000000000000, //204
        39'b000000110000000000000000110000000000000, //205
        39'b000001110000000000000001011000011001110, //206
        39'b001000000010011001101000010000000000000, //207
        39'b001000000000001000010000110000000000000, //208
        39'b000010000000001010000000110000000000000, //209
        39'b000000110000000000000000110000000000000, //210
        39'b000001110000000000000001011000011010011, //211
        39'b001000000010011001101000010000000000000, //212
        39'b000010000000001010000000110000000000000, //213
        39'b001000000001001000100000110000000000000, //214
        39'b000000110000000000000000110000000000000, //215
        39'b000001110000000000000001011000011011000, //216
        39'b001000000010011001101000010000000000000, //217
        39'b000010000000001010000000110000000000000, //218
        39'b001000000001001000010000110000000000000, //219
        39'b000000110000000000000000110000000000000, //220
        39'b000001110000000000000001011000011011101, //221
        39'b001000000010011001101000010000000000000, //222
        39'b000010000000001010000000110000000000000, //223
        39'b001000000000001000100000110000000000000, //224
        39'b000000110000000000000000110000000000000, //225
        39'b000001110000000000000001011000011100010, //226
        39'b001000000010011001101000010000000000000, //227
        39'b000010000000001010000000110000000000000, //228
        39'b001000000000001000010000110000000000000, //229
        39'b000000110000000000000000110000000000000, //230
        39'b000001110000000000000001011000011100111, //231
        39'b001000000010011001101000010000000000000, //232
        39'b000010000001001000100000110000000000000, //233
        39'b000000110000000000000000110000000000000, //234
        39'b000001110000000000000001011000011101011, //235
        39'b001000000010011001101000010000000000000, //236
        39'b000010000001001000010000110000000000000, //237
        39'b000000110000000000000010110000000000000, //238
        39'b000001110000000000000001011000011101111, //239
        39'b001000000010011001101000010000000000000, //240
        39'b000010000000001000100000110000000000000, //241
        39'b000000110000000000000010110000000000000, //242
        39'b000001110000000000000001011000011110011, //243
        39'b001000000010011001101000010000000000000, //244
        39'b000010000000001000010000110000000000000, //245
        39'b000000110000000000000000110000000000000, //246
        39'b000001110000000000000001011000011110111, //247
        39'b001000000010011001101000010000000000000, //248
        39'b001000000001001000100000110000000000000, //249
        39'b000010000000001010000000110000000000000, //250
        39'b000000110000000000000000110000000000000, //251
        39'b000001110000000000000001011000011111100, //252
        39'b001000000010011001101000010000000000000, //253
        39'b001000000001001000010000110000000000000, //254
        39'b000010000000001010000000110000000000000, //255
        39'b000000110000000000000000110000000000000, //256
        39'b000001110000000000000001011000100000001, //257
        39'b001000000010011001101000010000000000000, //258
        39'b001000000000001000100000110000000000000, //259
        39'b000010000000001010000000110000000000000, //260
        39'b000000110000000000000000110000000000000, //261
        39'b000001110000000000000001011000100000110, //262
        39'b001000000010011001101000010000000000000, //263
        39'b001000000000001000010000110000000000000, //264
        39'b000010000000001010000000110000000000000, //265
        39'b000000110000000000000000110000000000000, //266
        39'b000001110000000000000001011000100001011, //267
        39'b001000000010011001101000010000000000000, //268
        39'b000010000000001010000000110000000000000, //269
        39'b001000000001001000100000110000000000000, //270
        39'b000000110000000000000000110000000000000, //271
        39'b000001110000000000000001011000100010000, //272
        39'b001000000010011001101000010000000000000, //273
        39'b000010000000001010000000110000000000000, //274
        39'b001000000001001000010000110000000000000, //275
        39'b000000110000000000000000110000000000000, //276
        39'b000001110000000000000001011000100010101, //277
        39'b001000000010011001101000010000000000000, //278
        39'b000010000000001010000000110000000000000, //279
        39'b001000000000001000100000110000000000000, //280
        39'b000000110000000000000000110000000000000, //281
        39'b000001110000000000000001011000100011010, //282
        39'b001000000010011001101000010000000000000, //283
        39'b000010000000001010000000110000000000000, //284
        39'b001000000000001000010000110000000000000, //285
        39'b000000110000000000000000110000000000000, //286
        39'b000001110000000000000001011000100011111, //287
        39'b001000000010011001101000010000000000000, //288
        39'b000010000001001000100000110000000000000, //289
        39'b000000110000000000000000110000000000000, //290
        39'b000001110000000000000001011000100100011, //291
        39'b001000000010011001101000010000000000000, //292
        39'b000010000001001000010000110000000000000, //293
        39'b000000110000000000000000110000000000000, //294
        39'b000001110000000000000001011000100100111, //295
        39'b001000000010011001101000010000000000000, //296
        39'b000010000000001000100000110000000000000, //297
        39'b000000110000000000000000110000000000000, //298
        39'b000001110000000000000001011000100101011, //299
        39'b001000000010011001101000010000000000000, //300
        39'b000010000000001000010000110000000000000, //301
        39'b000000110000000000000000110000000000000, //302
        39'b000001110000000000000001011000100101111, //303
        39'b001000000010011001101000010000000000000, //304
        39'b001000000001001000100000110000000000000, //305
        39'b000010000000001010000000110000000000000, //306
        39'b000000110000000000000000110000000000000, //307
        39'b000001110000000000000001011000100110100, //308
        39'b001000000010011001101000010000000000000, //309
        39'b001000000001001000010000110000000000000, //310
        39'b000010000000001010000000110000000000000, //311
        39'b000000110000000000000000110000000000000, //312
        39'b000001110000000000000001011000100111001, //313
        39'b001000000010011001101000010000000000000, //314
        39'b001000000000001000100000110000000000000, //315
        39'b000010000000001010000000110000000000000, //316
        39'b000000110000000000000000110000000000000, //317
        39'b000001110000000000000001011000100111110, //318
        39'b001000000010011001101000010000000000000, //319
        39'b001000000000001000010000110000000000000, //320
        39'b000010000000001010000000110000000000000, //321
        39'b000000110000000000000000110000000000000, //322
        39'b000001110000000000000001011000101000011, //323
        39'b001000000010011001101000010000000000000, //324
        39'b000010000000001010000000110000000000000, //325
        39'b001000000001001000100000110000000000000, //326
        39'b000000110000000000000000110000000000000, //327
        39'b000001110000000000000001011000101001000, //328
        39'b001000000010011001101000010000000000000, //329
        39'b000010000000001010000000110000000000000, //330
        39'b001000000001001000010000110000000000000, //331
        39'b000000110000000000000000110000000000000, //332
        39'b000001110000000000000001011000101001101, //333
        39'b001000000010011001101000010000000000000, //334
        39'b000010000000001010000000110000000000000, //335
        39'b001000000000001000100000110000000000000, //336
        39'b000000110000000000000000110000000000000, //337
        39'b000001110000000000000001011000101010010, //338
        39'b001000000010011001101000010000000000000, //339
        39'b000010000000001010000000110000000000000, //340
        39'b001000000000001000010000110000000000000, //341
        39'b000000110000000000000000110000000000000, //342
        39'b000001110000000000000001011000101010111, //343
        39'b001000000010011001101000010000000000000, //344
        39'b000010000001001000100000110000000000000, //345
        39'b000000110000000000000000110000000000000, //346
        39'b000001110000000000000001011000101011011, //347
        39'b001000000010011001101000110000000000000, //348
        39'b101000000001001001101000010000000000000, //349
        39'b000010000001001000010000110000000000000, //350
        39'b000000110000000000000000110000000000000, //351
        39'b000001110000000000000001011000101100000, //352
        39'b001000000010011001101000110000000000000, //353
        39'b101000000001001001101000010000000000000, //354
        39'b000010000000001000100000110000000000000, //355
        39'b000000110000000000000000110000000000000, //356
        39'b000001110000000000000001011000101100101, //357
        39'b001000000010011001101000110000000000000, //358
        39'b101000000001001001101000010000000000000, //359
        39'b000010000000001000010000110000000000000, //360
        39'b000000110000000000000000110000000000000, //361
        39'b000001110000000000000001011000101101010, //362
        39'b001000000010011001101000110000000000000, //363
        39'b101000000001001001101000010000000000000, //364
        39'b001000000001001000100000110000000000000, //365
        39'b000010000000001010000000110000000000000, //366
        39'b000000110000000000000000110000000000000, //367
        39'b000001110000000000000001011000101110000, //368
        39'b001000000010011001101000110000000000000, //369
        39'b101000000001001001101000010000000000000, //370
        39'b001000000001001000010000110000000000000, //371
        39'b000010000000001010000000110000000000000, //372
        39'b000000110000000000000000110000000000000, //373
        39'b000001110000000000000001011000101110110, //374
        39'b001000000010011001101000110000000000000, //375
        39'b101000000001001001101000010000000000000, //376
        39'b001000000000001000100000110000000000000, //377
        39'b000010000000001010000000110000000000000, //378
        39'b000000110000000000000000110000000000000, //379
        39'b000001110000000000000001011000101111100, //380
        39'b001000000010011001101000110000000000000, //381
        39'b101000000001001001101000010000000000000, //382
        39'b001000000000001000010000110000000000000, //383
        39'b000010000000001010000000110000000000000, //384
        39'b000000110000000000000000110000000000000, //385
        39'b000001110000000000000001011000110000010, //386
        39'b001000000010011001101000110000000000000, //387
        39'b101000000001001001101000010000000000000, //388
        39'b000010000000001010000000110000000000000, //389
        39'b001000000001001000100000110000000000000, //390
        39'b000000110000000000000000110000000000000, //391
        39'b000001110000000000000001011000110001000, //392
        39'b001000000010011001101000110000000000000, //393
        39'b101000000001001001101000010000000000000, //394
        39'b000010000000001010000000110000000000000, //395
        39'b001000000001001000010000110000000000000, //396
        39'b000000110000000000000000110000000000000, //397
        39'b000001110000000000000001011000110001110, //398
        39'b001000000010011001101000110000000000000, //399
        39'b101000000001001001101000010000000000000, //400
        39'b000010000000001010000000110000000000000, //401
        39'b001000000000001000100000110000000000000, //402
        39'b000000110000000000000000110000000000000, //403
        39'b000001110000000000000001011000110010100, //404
        39'b001000000010011001101000110000000000000, //405
        39'b101000000001001001101000010000000000000, //406
        39'b000010000000001010000000110000000000000, //407
        39'b001000000000001000010000110000000000000, //408
        39'b000000110000000000000000110000000000000, //409
        39'b000001110000000000000001011000110011010, //410
        39'b001000000010011001101000110000000000000, //411
        39'b101000000001001001101000010000000000000, //412
        39'b000010000001001000100010110000000000000, //413
        39'b000000110000000000000010110000000000000, //414
        39'b000001110000000000000011011000110011111, //415
        39'b001000000010011001101010110000000000000, //416
        39'b101000000001001001101010010000000000000, //417
        39'b000010000001001000010010110000000000000, //418
        39'b000000110000000000000010110000000000000, //419
        39'b000001110000000000000011011000110100100, //420
        39'b001000000010011001101010110000000000000, //421
        39'b101000000001001001101010010000000000000, //422
        39'b000010000000001000100010110000000000000, //423
        39'b000000110000000000000010110000000000000, //424
        39'b000001110000000000000011011000110101001, //425
        39'b001000000010011001101010110000000000000, //426
        39'b101000000001001001101010010000000000000, //427
        39'b000010000000001000010010110000000000000, //428
        39'b000000110000000000000010110000000000000, //429
        39'b000001110000000000000011011000110101110, //430
        39'b001000000010011001101010110000000000000, //431
        39'b101000000001001001101010010000000000000, //432
        39'b001000000001001000100010110000000000000, //433
        39'b000010000000001010000010110000000000000, //434
        39'b000000110000000000000010110000000000000, //435
        39'b000001110000000000000011011000110110100, //436
        39'b001000000010011001101010110000000000000, //437
        39'b101000000001001001101010010000000000000, //438
        39'b001000000001001000010010110000000000000, //439
        39'b000010000000001010000010110000000000000, //440
        39'b000000110000000000000010110000000000000, //441
        39'b000001110000000000000011011000110111010, //442
        39'b001000000010011001101010110000000000000, //443
        39'b101000000001001001101010010000000000000, //444
        39'b001000000000001000100010110000000000000, //445
        39'b000010000000001010000010110000000000000, //446
        39'b000000110000000000000010110000000000000, //447
        39'b000001110000000000000011011000111000000, //448
        39'b001000000010011001101010110000000000000, //449
        39'b101000000001001001101010010000000000000, //450
        39'b001000000000001000010010110000000000000, //451
        39'b000010000000001010000010110000000000000, //452
        39'b000000110000000000000010110000000000000, //453
        39'b000001110000000000000011011000111000110, //454
        39'b001000000010011001101010110000000000000, //455
        39'b101000000001001001101010010000000000000, //456
        39'b000010000000001010000010110000000000000, //457
        39'b001000000001001000100010110000000000000, //458
        39'b000000110000000000000010110000000000000, //459
        39'b000001110000000000000011011000111001100, //460
        39'b001000000010011001101010110000000000000, //461
        39'b101000000001001001101010010000000000000, //462
        39'b000010000000001010000010110000000000000, //463
        39'b001000000001001000010010110000000000000, //464
        39'b000000110000000000000010110000000000000, //465
        39'b000001110000000000000011011000111010010, //466
        39'b001000000010011001101010110000000000000, //467
        39'b101000000001001001101010010000000000000, //468
        39'b000010000000001010000010110000000000000, //469
        39'b001000000000001000100010110000000000000, //470
        39'b000000110000000000000010110000000000000, //471
        39'b000001110000000000000011011000111011000, //472
        39'b001000000010011001101010110000000000000, //473
        39'b101000000001001001101010010000000000000, //474
        39'b000010000000001010000010110000000000000, //475
        39'b001000000000001000010010110000000000000, //476
        39'b000000110000000000000010110000000000000, //477
        39'b000001110000000000000011011000111011110, //478
        39'b001000000010011001101010110000000000000, //479
        39'b101000000001001001101010010000000000000, //480
        39'b011000000001011001101000010000000000000, //481
        39'b001000000001011001101000010000000000000, //482
        39'b011000000001011001111000010000000000000, //483
        39'b001000000001011001111000010000000000000, //484
        39'b010000000001001000010000010000000000000, //485
        39'b010000000001001000100000010000000000000, //486
        39'b010000000001001000000000010000000000000, //487
        39'b010000000001001000001000010000000000000, //488
        39'b001000000001011000010000010000000000000, //489
        39'b011000000001011000010000010000000000000, //490
        39'b011000000001011000011000010000000000000, //491
        39'b001000000001011000011000010000000000000, //492
        39'b011000000001011000101000010000000000000, //493
        39'b001000000001011000101000010000000000000, //494
        39'b011000000001011000110000010000000000000, //495
        39'b001000000001011000110000010000000000000, //496
        39'b011000000001011000111000010000000000000, //497
        39'b001000000001011000111000010000000000000, //498
        39'b011000000001011000000000010000000000000, //499
        39'b001000000001011000000000010000000000000, //500
        39'b011000000001011001110000010000000000000, //501
        39'b001000000001011001110000010000000000000, //502
        39'b011000000001011000001000010000000000000, //503
        39'b001000000001011000001000010000000000000, //504
        39'b011000000001011001100000010000000000000, //505
        39'b001000000001011001100000010000000000000, //506
        39'b000010000001001000100110110000000000000, //507
        39'b000001000100001110000110110000000000000, //508
        39'b000000010000000000000110110000000000000, //509
        39'b000000010000000000000111011000111111110, //510
        39'b000010000001001010010110110000000000000, //511
        39'b000001001100001110000110110000000000000, //512
        39'b000000010000000000000110110000000000000, //513
        39'b000000010000000000000111110001000000010, //514
        39'b000010000001001000010110110000000000000, //515
        39'b000001000100001110000110110000000000000, //516
        39'b000000010000000000000110110000000000000, //517
        39'b000000010000000000000111011001000000110, //518
        39'b000010000001001010101110110000000000000, //519
        39'b000001001100001110000110110000000000000, //520
        39'b000000010000000000000110110000000000000, //521
        39'b000000010000000000000111110001000001010, //522
        39'b000010000000001000100110110000000000000, //523
        39'b000001000100001110000110110000000000000, //524
        39'b000000010000000000000110110000000000000, //525
        39'b000000010000000000000111011001000001110, //526
        39'b000010000000001010010110110000000000000, //527
        39'b000001001100001110000110110000000000000, //528
        39'b000000010000000000000110110000000000000, //529
        39'b000000010000000000000111110001000010010, //530
        39'b000010000000001000010110110000000000000, //531
        39'b000001000100001110000110110000000000000, //532
        39'b000000010000000000000110110000000000000, //533
        39'b000000010000000000000111011001000010110, //534
        39'b000010000000001010101110110000000000000, //535
        39'b000001001100001110000110110000000000000, //536
        39'b000000010000000000000110110000000000000, //537
        39'b000000010000000000000111110001000011010, //538
        39'b001000000001001000100110110000000000000, //539
        39'b000010000000001010000110110000000000000, //540
        39'b000001000100001110000110110000000000000, //541
        39'b000000010000000000000110110000000000000, //542
        39'b000000010000000000000111011001000011111, //543
        39'b000010000000001010001110110000000000000, //544
        39'b000001001100001110000110110000000000000, //545
        39'b000000010000000000000110110000000000000, //546
        39'b000000010000000000000111110001000100011, //547
        39'b001000000001001000010110110000000000000, //548
        39'b000010000000001010000110110000000000000, //549
        39'b000001000100001110000110110000000000000, //550
        39'b000000010000000000000110110000000000000, //551
        39'b000000010000000000000111011001000101000, //552
        39'b000010000000001010001110110000000000000, //553
        39'b000001001100001110000110110000000000000, //554
        39'b000000010000000000000110110000000000000, //555
        39'b000000010000000000000111110001000101011, //556
        39'b001000000000001000100110110000000000000, //557
        39'b000010000000001010000110110000000000000, //558
        39'b000001000100001110000110110000000000000, //559
        39'b000000010000000000000110110000000000000, //560
        39'b000000010000000000000111011001000110001, //561
        39'b000010000000001010001110110000000000000, //562
        39'b000001001100001110000110110000000000000, //563
        39'b000000010000000000000110110000000000000, //564
        39'b000000010000000000000111110001000110101, //565
        39'b001000000000001000010110110000000000000, //566
        39'b000010000000001010000110110000000000000, //567
        39'b000001000100001110000110110000000000000, //568
        39'b000000010000000000000110110000000000000, //569
        39'b000000010000000000000111011001000111010, //570
        39'b000010000000001010001110110000000000000, //571
        39'b000001001100001110000110110000000000000, //572
        39'b000000010000000000000110110000000000000, //573
        39'b000000010000000000000111110001000111110, //574
        39'b000010000000001010000110110000000000000, //575
        39'b000001000100001110000110110000000000000, //576
        39'b000000010000000000000110110000000000000, //577
        39'b000000010000000000000111011001001000010, //578
        39'b000010000000001010001110110000000000000, //579
        39'b000001001100001110000110110000000000000, //580
        39'b000000010000000000000110110000000000000, //581
        39'b000000010000000000000111011001001000110, //582
        39'b001000000101011000100110010000000000000, //583
        39'b000010000000001010000110110000000000000, //584
        39'b000001000100001110000110110000000000000, //585
        39'b000000010000000000000110110000000000000, //586
        39'b000000010000000000000111011001001001011, //587
        39'b000010000000001010001110110000000000000, //588
        39'b000001001100001110000110110000000000000, //589
        39'b000000010000000000000110110000000000000, //590
        39'b000000010000000000000111011001001001111, //591
        39'b001000000101011000010110010000000000000, //592
        39'b000010000000001010000110110000000000000, //593
        39'b000001000100001110000110110000000000000, //594
        39'b000000010000000000000110110000000000000, //595
        39'b000000010000000000000111011001001010100, //596
        39'b000010000000001010001110110000000000000, //597
        39'b000001001100001110000110110000000000000, //598
        39'b000000010000000000000110110000000000000, //599
        39'b000000010000000000000111011001001011000, //600
        39'b001000000100011000100110010000000000000, //601
        39'b000010000000001010000110110000000000000, //602
        39'b000001000100001110000110110000000000000, //603
        39'b000000010000000000000110110000000000000, //604
        39'b000000010000000000000111011001001011101, //605
        39'b000010000000001010001110110000000000000, //606
        39'b000001001100001110000110110000000000000, //607
        39'b000000010000000000000110110000000000000, //608
        39'b000000010000000000000111011001001100001, //609
        39'b001000000100011000010110010000000000000, //610
        39'b000010000001001000100110110000000000000, //611
        39'b000000110000000000000110110000000000000, //612
        39'b000001110000000000000111011001001100101, //613
        39'b001000000010011001101110110000000000000, //614
        39'b000010000001001010010110110000000000000, //615
        39'b000000110000000000000110110000000000000, //616
        39'b000001110000000000000111011001001101001, //617
        39'b001000000010111001101110010000000000000, //618
        39'b000010000001001000010110110000000000000, //619
        39'b000000110000000000000110110000000000000, //620
        39'b000001110000000000000111011001001101101, //621
        39'b001000000010011001101110110000000000000, //622
        39'b000010000001001010101110110000000000000, //623
        39'b000000110000000000000110110000000000000, //624
        39'b000001110000000000000111011001001110001, //625
        39'b001000000010111001101110010000000000000, //626
        39'b000010000000001000100110110000000000000, //627
        39'b000000110000000000000110110000000000000, //628
        39'b000001110000000000000111011001001110101, //629
        39'b001000000010011001101110110000000000000, //630
        39'b000010000000001010010110110000000000000, //631
        39'b000000110000000000000110110000000000000, //632
        39'b000001110000000000000111011001001111001, //633
        39'b001000000010111001101110010000000000000, //634
        39'b000010000000001000010110110000000000000, //635
        39'b000000110000000000000110110000000000000, //636
        39'b000001110000000000000111011001001111101, //637
        39'b001000000010011001101110110000000000000, //638
        39'b000010000000001010101110110000000000000, //639
        39'b000000110000000000000110110000000000000, //640
        39'b000001110000000000000111011001010000001, //641
        39'b001000000010111001101110010000000000000, //642
        39'b001000000001001000100110110000000000000, //643
        39'b000010000000001010000110110000000000000, //644
        39'b000000110000000000000110110000000000000, //645
        39'b000001110000000000000111011001010000110, //646
        39'b001000000010011001101110110000000000000, //647
        39'b000010000000001010001110110000000000000, //648
        39'b000000110000000000000110110000000000000, //649
        39'b000001110000000000000111011001010001010, //650
        39'b001000000010111001101110010000000000000, //651
        39'b001000000001001000010110110000000000000, //652
        39'b000010000000001010000110110000000000000, //653
        39'b000000110000000000000110110000000000000, //654
        39'b000001110000000000000111011001010001111, //655
        39'b001000000010011001101110110000000000000, //656
        39'b000010000000001010001110110000000000000, //657
        39'b000000110000000000000110110000000000000, //658
        39'b000001110000000000000111011001010010011, //659
        39'b001000000010111001101110010000000000000, //660
        39'b001000000000001000100110110000000000000, //661
        39'b000010000000001010000110110000000000000, //662
        39'b000000110000000000000110110000000000000, //663
        39'b000001110000000000000111011001010011000, //664
        39'b001000000010011001101110110000000000000, //665
        39'b000010000000001010001110110000000000000, //666
        39'b000000110000000000000110110000000000000, //667
        39'b000001110000000000000111011001010011100, //668
        39'b001000000010111001101110010000000000000, //669
        39'b001000000000001000010110110000000000000, //670
        39'b000010000000001010000110110000000000000, //671
        39'b000000110000000000000110110000000000000, //672
        39'b000001110000000000000111011001010100001, //673
        39'b001000000010011001101110110000000000000, //674
        39'b000010000000001010001110110000000000000, //675
        39'b000000110000000000000110110000000000000, //676
        39'b000001110000000000000111011001010100101, //677
        39'b001000000010111001101110010000000000000, //678
        39'b000010000000001010000110110000000000000, //679
        39'b000000110000000000000110110000000000000, //680
        39'b000001110000000000000111011001010101001, //681
        39'b001000000010011001101110110000000000000, //682
        39'b000010000000001010001110110000000000000, //683
        39'b000000110000000000000110110000000000000, //684
        39'b000001110000000000000111011001010101101, //685
        39'b001000000010111001101110110000000000000, //686
        39'b001000000001001000100110010000000000000, //687
        39'b000010000000001010000110110000000000000, //688
        39'b000000110000000000000110110000000000000, //689
        39'b000001110000000000000111011001010110010, //690
        39'b001000000010011001101110110000000000000, //691
        39'b000010000000001010001110110000000000000, //692
        39'b000000110000000000000110110000000000000, //693
        39'b000001110000000000000111011001010110110, //694
        39'b001000000010111001101110110000000000000, //695
        39'b001000000001001000010110010000000000000, //696
        39'b000010000000001010000110110000000000000, //697
        39'b000000110000000000000110110000000000000, //698
        39'b000001110000000000000111011001010111011, //699
        39'b001000000010011001101110110000000000000, //700
        39'b000010000000001010001110110000000000000, //701
        39'b000000110000000000000110110000000000000, //702
        39'b000001110000000000000111011001010111111, //703
        39'b001000000010111001101110110000000000000, //704
        39'b001000000000001000100110010000000000000, //705
        39'b000010000000001010000110110000000000000, //706
        39'b000000110000000000000110110000000000000, //707
        39'b000001110000000000000111011001011000100, //708
        39'b001000000010011001101110110000000000000, //709
        39'b000010000000001010001110110000000000000, //710
        39'b000000110000000000000110110000000000000, //711
        39'b000001110000000000000111011001011001000, //712
        39'b001000000010111001101110110000000000000, //713
        39'b001000000000001000010110010000000000000 //714

        
        

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
    5'b10010:   Out = A + B + 4;
    5'b10011:  Out = A >>1; // right shift
    5'b10100:  Out = A <<1; // left shift
    5'b10101:  Out = A - B + 4;
    5'b10110:   Out = B <<1;
    endcase
    
     Zero = ~(|Out ); //bitwise or
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
    $display("__R15: %b, Clock:%b, t:%0d\n__R1: %b, Clock:%b, t:%0d\n__R2: %b, Clock:%b, t:%0d\n__R3: %b, __R0: %b, __R1: %b, Clock:%b, t:%0d", I15, clk, $time, I1, clk, $time, I2, clk, $time, I3, I0, I1, clk, $time);
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

