module main;
//main is used strictly for testing purposes
    
parameter sim_time = 1600;


wire [32:0] CRout;
wire [6:0] curr_state;
wire [31:0] IR;
reg [31:0] IRin;
reg Cond, MOC, clk, LE, reset;

InstructionRegister instruc_reg(IR, IRin, LE, clk);

ControlUnit cu(CRout, curr_state, IR, Cond, MOC, reset, clk);

//simulation time
initial #sim_time $finish;

initial begin
    MOC = 1'b1;
    // MOC = 1'b0;
    // Cond = 1'b0;
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
  repeat(1000) #5 clk = ~clk;
end

initial begin
  reset = 1'b1;
//   #15 reset = ~reset;
#5 reset = ~reset;
end

initial begin
  LE <= 1'b1;
end

initial begin
  
//   IRin <= 32'b1110_000_0100_0_0110_0100_00001110_0100; //estado 5 - ADD R4,R6,R4,ROR #? --> funciona mas o menos chilling
//   IRin <= 32'b1110_010_11100_0110_0100_000011100100;// estado 8 STRB R4,[R6,#+?] 
//   IRin <= 32'b1110_011_11100_0110_0100_000000000100;// estado 16 STRB register offset ADD
//    IRin <= 32'b1110_101_01100_0110_0100_000011100100; // estado 64 Branch instruction
end


initial begin
  $display("\nControl Unit Test - Jorge Vega | Sebastian Merced | Wiliel Florenciani \n");
end

initial #1 begin
  $display("Signals to be tested\n");
  $monitor("CRout:%b, IR:%b, State:%0d, Cond:%b, MOC:%b, reset:%b, clk:%b, time:%0d",CRout[32:0],IR,curr_state,Cond,MOC,reset,clk,$time);
// $monitor("CRout:%b, IR:%b, State:%0d, Cond:%b, MOC:%b, reset:%b, clk:%b, time:%0d",CRout,IR,CRout[39:33],Cond,MOC,reset,clk,$time);
end
endmodule



module ControlUnit(output [32:0] CRout, output [6:0] curr_state, input [31:0] IR, input Cond, MOC, reset, clk);

wire [6:0] mux7Out;
wire mux1Out;
wire [6:0] AdderOut;
wire [6:0] EncoderOut;
wire [32:0] CRin;
wire invOut;
wire [6:0] incrementedState;
wire [1:0] M;
wire noValue = 0;
wire [6:0] val1 = 1;

// always@(*) begin
//     RFld <= CR[x]
//     IRld <= CR[x]
//     MARld <= CR[x]
//     MDRld <= CR[x]
//     R_W <= CR[x]
//     MOV <= CR[x]
//     MOC <= CR[x]
//     FRld <= CR[x]
//     Cin <= CR[x]
//     Cond <= CR[x]
//     MA <= CR[:]
//     MB <= CR[:]
//     MC <= CR[:]
//     MD <= CR[:]
//     ME <= CR[:]
// end


Multiplexer7_4x2 mux7_4x2 (mux7Out, EncoderOut, val1, CRout[6:0], incrementedState, M, reset);
Adder adder (AdderOut, mux7Out);
Microstore microstore (CRin, curr_state, reset, mux7Out);
ControlRegister control_register (CRout, clk, CRin);
NextStateAddressSelector nsas (M, invOut, CRout[32:30]);
Inverter inv (invOut, mux1Out, CRout[29]);
IncrementerRegister incr_reg (incrementedState, AdderOut, clk);
Multiplexer1_4x2 mux1_4x2 (mux1Out, MOC, Cond, noValue, noValue, CRout[28:27]); //aqui MOC tiene que ir en 0 y Cond en 1
Encoder encoder (EncoderOut, IR);

// initial begin
// $monitor("CU_monitor: MuxOut = %d", mux7Out); ---
// end

endmodule

module Inverter(output reg out, input in, inv);
    
    always @(in, inv)
        out = inv ? !in: in;
        
endmodule

// multiplexer4x2
module Multiplexer7_4x2(output reg [6:0] out, input [6:0] I0, I1, I2, I3, input [1:0] S, input reset);
    always @ (S,I0,I1,I2,I3) begin
    // $display("MUX7 - changes ---- out %b,  I0 %b, I1 %b, I2 %b, I3 %b, S %b   time %0d", out, I0, I1, I2, I3, S, $time); ---
    end

    always @ (*) begin
    // always @ (S, reset) begin
        if(reset) 
        begin
            // out = 0; //maybe make this <=?
            out <= 0;
            // $display("~~~ mux is outputting 0 because of reset ~~~");
        end
        else begin
        // $display("Mux7 - before changes ---- out %b,  I0 %b, I1 %b, I2 %b, I3 %b, S %b", out,I0,I1,I2,I3,S);
        case(S)
            2'h0: out <= I0;
            2'h1: out <= I1;
            2'h2: out <= I2;
            2'h3: out <= I3;
        endcase
        $display("__Mux7 - out:%d,  enc:%d, same:%d, cr:%d, inc:%d, S:%b,       t:%0d", out,I0,I1,I2,I3,S,$time); 
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

    // always @ (*) begin
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

        $display("__NSAS - out:%b, Ns:%b, Sts:%b                                     t:%0d", M, N, Sts, $time); 
        end

    initial begin
    $monitor("__NSAS - out:%b, Ns:%b, Sts:%b                                     t:%0d", M, N, Sts, $time); 
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
// $display("__Adder input: %d, Adder output: %d", in, out); ---

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
                    5'b10100:  Out = 7'b0010100; 
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

module ControlRegister(output reg [32:0] Qs, input Clk, input [32:0] Ds); //32b bus, 20 moore lines, 6 CR and 6 NSAS
  always @ (posedge Clk) begin
   Qs <= Ds;
end
endmodule