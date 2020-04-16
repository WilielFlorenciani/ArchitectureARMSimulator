module main;
//main is used strictly for testing purposes
    
parameter sim_time = 600;
reg clk;

//simulation time
initial #sim_time $finish;

//manejar clock
initial begin
  clk <= 1'b1;
  repeat(100) #10 clk = ~clk;
end

initial begin
  $display("Control Unit Test - Jorge Vega | Sebastian Merced | Wiliel Florenciani \n");
end

initial #1 begin
  $display("Signals to be tested\n");

  $display("Clock  Load   Mux1    Mux7   Time");
  $display("--------------------------------------------------------------------------------");
  $monitor("%b, %0d",clk,$time);
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

// module register32bit(output reg [31:0] Q, input [31:0] D, input clk, ld);
// initial Q <= 32'd0;
//   always @ (posedge clk)
//   if(ld) Q <= D;
// endmodule