module RAM_Access;

integer fi, fo, code, i; reg [7:0] data; //variables to handle file info
reg Enable, ReadWrite; reg [31:0] DataIn; //variables to handle ram interfacing (enable, r/w signals; address reg, data in reg and data out bus)
reg [6:0] Address; wire [31:0] DataOut; wire MOC; reg [1:0] OpCode;

ram512x8 ramobj (DataOut, MOC, Enable, ReadWrite, Address, DataIn, OpCode); //declaring a ram instance

initial begin //initial to precharge ramobj's memory with the file
    fi = $fopen("PF1_Vega_Rodriguez_Jorge_ramdata.txt","r");
    /* en el documento de input se encuentra la siguiente instruccion de ARM cuatro veces:
    LDMFD R3!, {R1, R7-R10, R12}
    11101000101100110001011110000010 -> su equivalente en binario
    E8 B3 17 82 -> su equivalente hexadecimal
    */
    Address = 7'b0000000;
    OpCode = 2'b00;
    while (!$feof(fi)) begin
        code = $fscanf(fi, "%x", data);
        ramobj.Mem[Address] = data;
        Address = Address + 1;
    end
    $fclose(fi);
end

initial begin //initial to read content of memory after precharging
    $display("----- Memory contents after precharging -----"); 
    Enable = 1'b0; 
    ReadWrite = 1'b1;                        
    Address = #1 7'b0000000;
    OpCode = 2'b00;
    repeat (16) begin
        Enable = 1'b1;
        #1;
        $display("data in address %d = %x    -    Enable: %b ReadWrite: %b  time: %d", Address, DataOut, Enable, ReadWrite, $time);
        Enable = 1'b0;
        #1;
        Address = Address + 1;
        #1;
    end                                                             
end 

initial begin //initial to write a byte into address 0
    #50;
    $display("----- Data at address 0 before writing a byte: -----");
    $display("Data in address 0: %h    - Enable: %b ReadWrite: %b time: ", ramobj.Mem[0], Enable, ReadWrite, $time);

    Enable = 1'b0;
    ReadWrite = 1'b0; //write mode 
    Address = 7'b0000000;
    DataIn = 8'hFF;
    OpCode = 2'b00; // 00 is opcode for writing bytes
    Enable = 1'b1; //activate RAM
    #1;
    $display("----- Data at address 0 after writing a byte: -----");
    $display("Data in address 0: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[0], Enable, ReadWrite, $time);
    #1;
    Enable = 1'b0;
end

initial begin //initial to write a halfword into address 2
    #53;
    $display("----- Data at address 2 and 3 before writing a halfword into address 2: -----");
    $display("Data in address 2: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[2], Enable, ReadWrite, $time);
    $display("Data in address 3: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[3], Enable, ReadWrite, $time);

    ReadWrite = 1'b0; //write mode 
    Address = 7'b0000010;
    DataIn = 16'hDDDD;
    OpCode = 2'b01; // 01 is opcode for writing halfwords
    Enable = 1'b1; //activate RAM
    #1;
    $display("----- Data at address 2 and 3 after writing a halfword: -----");
    $display("Data in address 2: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[2], Enable, ReadWrite, $time);
    $display("Data in address 3: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[3], Enable, ReadWrite, $time);
    $display("DataIn bus content: %h - Enable: %b ReadWrite: %b time: ", DataIn, Enable, ReadWrite, $time);
    #1;
    Enable = 1'b0;
end

initial begin //initial to write a halfword into address 4
    #56;
    $display("----- Data at address 4 and 5 before writing a halfword into address 4: -----");
    $display("Data in address 4: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[4], Enable, ReadWrite, $time);
    $display("Data in address 5: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[5], Enable, ReadWrite, $time);

    ReadWrite = 1'b0; //write mode 
    Address = 7'b0000100;
    DataIn = 16'hEFFE;
    OpCode = 2'b01; // 01 is opcode for writing halfwords
    Enable = 1'b1; //activate RAM
    #1;
    $display("----- Data at address 4 and 5 after writing a halfword: -----");
    $display("Data in address 4: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[4], Enable, ReadWrite, $time);
    $display("Data in address 5: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[5], Enable, ReadWrite, $time);
    $display("DataIn bus content: %h - Enable: %b ReadWrite: %b time: ", DataIn, Enable, ReadWrite, $time);
    #1;
    Enable = 1'b0;
end

initial begin //initial to write a word into address 8
    #59;
    $display("----- Data at address 8, 9, 10, 11 before writing a word into address 8: -----");
    $display("Data in address 8: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[8], Enable, ReadWrite, $time);
    $display("Data in address 9: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[9], Enable, ReadWrite, $time);
    $display("Data in address 10: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[10], Enable, ReadWrite, $time);
    $display("Data in address 11: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[11], Enable, ReadWrite, $time);

    ReadWrite = 1'b0; //write mode 
    Address = 7'b0001000;
    DataIn = 32'hABCDABCD;
    OpCode = 2'b10; // 10 is opcode for writing words
    Enable = 1'b1; //activate RAM
    #1;
    $display("----- Data at address 8, 9, 10, 11 after writing a word into address 8: -----");
    $display("Data in address 8: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[8], Enable, ReadWrite, $time);
    $display("Data in address 9: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[9], Enable, ReadWrite, $time);
    $display("Data in address 10: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[10], Enable, ReadWrite, $time);
    $display("Data in address 11: %h - Enable: %b ReadWrite: %b time: ", ramobj.Mem[11], Enable, ReadWrite, $time);
    $display("DataIn bus content: %h - Enable: %b ReadWrite: %b time: ", DataIn, Enable, ReadWrite, $time);
    #1;
    Enable = 1'b0;
end

initial begin //initial to read a byte from address 0
    #62;
    $display("----- Reading a byte from address 0 -----"); 
    ReadWrite = 1'b1; //read mode                         
    Address = #1 7'b0000000;
    OpCode = 2'b00;
    Enable = 1'b1;
    #1;
    $display("DataOut bus content in hexadecimal: %x    -    Enable: %b ReadWrite: %b  time: %d", DataOut, Enable, ReadWrite, $time);
    Enable = 1'b0;
    #1;                                                          
end 

initial begin //initial to read a halfword from address 2
    #66;
    $display("----- Reading a halfword from address 2 -----"); 
    ReadWrite = 1'b1; //read mode                         
    Address = #1 7'b0000010;
    OpCode = 2'b01;
    Enable = 1'b1;
    #1;
    $display("DataOut bus content in hexadecimal: %x    -    Enable: %b ReadWrite: %b  time: %d", DataOut, Enable, ReadWrite, $time);
    Enable = 1'b0;
    #1;                                                          
end 

initial begin //initial to read a halfword from address 4
    #70;
    $display("----- Reading a halfword from address 4 -----"); 
    ReadWrite = 1'b1; //read mode                         
    Address = #1 7'b0000100;
    OpCode = 2'b01;
    Enable = 1'b1;
    #1;
    $display("DataOut bus content in hexadecimal: %x    -    Enable: %b ReadWrite: %b  time: %d", DataOut, Enable, ReadWrite, $time);
    Enable = 1'b0;
    #1;                                                          
end 

initial begin //initial to read a word from address 8
    #74;
    $display("----- Reading a word from address 8 -----"); 
    ReadWrite = 1'b1; //read mode                         
    Address = #1 7'b0001000;
    OpCode = 2'b10;
    Enable = 1'b1;
    #1;
    $display("DataOut bus content in hexadecimal: %x    -    Enable: %b ReadWrite: %b  time: %d", DataOut, Enable, ReadWrite, $time);
    Enable = 1'b0;
    #1;                                                          
end

initial begin //initial to read a word from address 0
    #78;
    $display("----- Reading a word from address 0 -----"); 
    ReadWrite = 1'b1; //read mode                         
    Address = #1 7'b0000000;
    OpCode = 2'b10;
    Enable = 1'b1;
    #1;
    $display("DataOut bus content in hexadecimal: %x    -    Enable: %b ReadWrite: %b  time: %d", DataOut, Enable, ReadWrite, $time);
    Enable = 1'b0;
    #1;                                                          
end

initial begin //initial to read a word from address 4
    #82;
    $display("----- Reading a word from address 4 -----"); 
    ReadWrite = 1'b1; //read mode                         
    Address = #1 7'b0000100;
    OpCode = 2'b10;
    Enable = 1'b1;
    #1;
    $display("DataOut bus content in hexadecimal: %x    -    Enable: %b ReadWrite: %b  time: %d", DataOut, Enable, ReadWrite, $time);
    Enable = 1'b0;
    #1;                                                          
end

endmodule

module ram512x8(output reg [31:0] DataOut, output reg MOC, input Enable, input ReadWrite, input [6:0] Address, input [31:0] DataIn, input [1:0] OpCode);

  reg [7:0] Mem[0:511]; //512 localizaciones de 8 bits
  always @ (Enable, ReadWrite) begin
    MOC <= 0; 
    if (Enable) begin
        case (OpCode) 
            2'b00: begin //opcode for byte operations 
                if(ReadWrite) begin //read
                    DataOut[7:0] = Mem[Address];
                    DataOut[31:8] = 24'h000000;
                    MOC <= 1; 
                end else begin  //write
                    Mem[Address] <= DataIn[7:0];
                    MOC <= 1;
                end
            end
            2'b01: begin //opcode for halfword operations
                if(ReadWrite) begin //read
                    DataOut[31:16] <= Mem[Address];
                    DataOut[15:8] <= Mem[Address + 1];
                    DataOut[7:0] <= 16'h0000; 
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
                    #2
                    DataOut[31:24] <= Mem[Address+4];
                    DataOut[23:16] <= Mem[Address+5];
                    DataOut[15:8] <= Mem[Address+6];
                    DataOut[7:0] <= Mem[Address+7];
                    MOC <= 1;
                end
                else begin //write 
                    Mem[Address] <= DataIn[31:24];
                    Mem[Address+1] <= DataIn[23:16];
                    Mem[Address+2] <= DataIn[15:8];
                    Mem[Address+3] <= DataIn[7:0];
                    #2
                    Mem[Address+4] <= DataIn[31:24];
                    Mem[Address+5] <= DataIn[23:16];
                    Mem[Address+6] <= DataIn[15:8];
                    Mem[Address+7] <= DataIn[7:0];
                    MOC <= 1;
                end
            end
        endcase 
    end
end
endmodule




