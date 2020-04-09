module main;
//main is used strictly for testing purposes
    
parameter sim_time = 600;

wire [31:0] PA, PB; //output of reads of register file

reg [3:0] BD; //inputs to binary decoder 

reg rfLd; // register file load

reg clk; //clock

reg [31:0] PC; //32 bits Port C

reg [3:0] SA; //Select for MUX A
reg [3:0] SB; //Select for MUX B

registerfile regfile (PA, PB, PC, clk , rfLd, BD, SA, SB);

//simulation time
initial #sim_time $finish;

initial begin
  PC <= 0;
  repeat(15) #20 PC <= PC + 4;
end

//manejar clock
initial begin
  clk <= 1'b1;
  repeat(100) #10 clk = ~clk;
end

initial begin
    BD <= 1'b0;
  repeat(16) #20 BD <= BD + 4'b0001;
    BD <= 1'b0;
end



//prevents loading 
initial begin
    rfLd <= 1'b1;
    #320 rfLd <= 1'b0;
end


initial #330 begin

    SA <= 4'b0000;
    repeat(7) #10 SA <= SA + 4'b0001;
end

initial #330 begin
    SB <= 4'b1000;
    repeat(7) #10 SB <= SB + 4'b0001;
end


//Load to register 10
initial begin
    #440 begin
    BD <= 10;
    rfLd <= 1'b1;
    end
end

initial begin
    #430 PC <= 99;
end

initial begin
    #430 SA <= 10;
end

initial begin
  $display("Register File Test - By Wiliel Florenciani\n");
end

initial #0 begin
  $display("Load contents onto register - stage Register File\n");

  $display("         PA          PB         PC    Clock  Load  bdin   MuxA    MuxB   Time");
  $display("--------------------------------------------------------------------------------");
  $monitor("%d, %d, %d,     %b,     %b,   %d,   R%d,     R%d,  %0d",PA,PB,PC,clk,rfLd,BD,SA,SB,$time);
end

initial begin
    #330 begin
    $display("\n\nShow register contents - stage Register File\n");
    $display("         PA          PB         PC    Clock  Load  bdin   MuxA    MuxB   Time");
    $display("--------------------------------------------------------------------------------");
    end
end

initial begin
    #410 begin
    $display("\n\nRegister 10 unique number - stage Register File\n");
    $display("         PA          PB         PC    Clock  Load  bdin   MuxA    MuxB   Time");
    $display("--------------------------------------------------------------------------------");
    end
end

    
endmodule



module registerfile(output wire [31:0] PA, PB, input [31:0] PC, input clk, rfLd, input [3:0] BD, SA, SB);

wire [15:0] BDselect;

wire [31:0] I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15;



binaryDecoder16bit decoder (BDselect, BD, rfLd);

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
register32bit r15 (I15, PC, clk, BDselect[15]);


Multiplexer16x4 muxA (PA, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, SA);

Multiplexer16x4 muxB (PB, I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, SB);

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












