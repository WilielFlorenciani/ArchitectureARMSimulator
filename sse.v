module testing;

reg [31:0] instr;
wire [9:0] out;
Encoder enc(out, instr);

initial begin
$monitor("Enc out: %0d, Enc in: %b", out, instr);
end

initial begin 
$display("Testing Encoder");
instr = 32'b0;
#1
instr = 32'b11100010000000010000000000000000;
#1
instr = 32'b11100011100000000001000000101000;
#1
instr = 32'b11100111110100010010000000000000;
#1
instr = 32'b11100101110100010011000000000010;
#1
instr = 32'b11100000100000000101000000000000;
#1
instr = 32'b11100000100000100101000000000101;
#1
instr = 32'b11100010010100110011000000000001;
#1
instr = 32'b00011010111111111111111111111101;
#1
instr = 32'b11100101110000010101000000000011;
#1
instr = 32'b11101010000000000000000000000001;
#1
instr = 32'b00001011000001010000011100000100;
#1
instr = 32'b11101010111111111111111111111111;
end

endmodule

/////////// encoder testing /////////////////

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
                                    Out = 10'b0; //estado de update Rn //734
                                else
                                    Out = 10'b0; //estado de no update Rn
                        //Decrement after
                        2'b00:   if(Instruction[21]==1'b1)
                                    Out = 10'b1011110010; //estado de permitir update Rn
                                    // Out = 10'b1011110011; //estado de permitir update Rn //755
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

    default: begin 
        Out = 10'b0000000001;
    end
endcase
end
endmodule



// module shift_sign_extender(output reg [31:0] extender_out, output reg carry, input [31:0] instruction, B, input Cin);

// reg [7:0] temp;

// always @(*) 
// begin
//     //immediate shifter operand
//     case (instruction[27:25])
//     3'b001: 
//         begin
//             if(instruction[11:8]==4'b0000)
//                 begin
//                 extender_out = instruction[7:0];
//                 carry = Cin;
//                 end
//             else
//                 begin
//                     temp = instruction[7:0];
//                     extender_out = {temp,temp} >>(2*{instruction[11:8]});
//                     carry = extender_out[31];
//                 end
//         end 
//     //shift by immediate shifter opperand
//     3'b000: 
//     begin
//         if(instruction[4]==0)
//         begin
//             if(instruction[6:5]==2'b00)
//             begin
//                 if(instruction[11:7] == 5'b00000)
//                 begin
//                     extender_out = B;
//                     carry = Cin;
//                 end
//                 else
//                     begin
//                         extender_out = B << instruction[11:7];
//                         carry = B[32-instruction[11:7]];
//                     end
//             end 

//             else if(instruction[6:5]==2'b01)
//                 begin
//                     if(instruction[11:7]==5'b00000)
//                         begin
//                             extender_out = 32'b0;
//                             carry = B[31];
//                         end
//                     else
//                         begin
//                             extender_out = B >> instruction[11:7];
//                             carry = B[instruction[11:7]-1];
//                         end
//                 end
//             else if(instruction[6:5]==2'b10)
//                 begin
//                     if(instruction[11:7]==5'b00000) begin
//                         if(B[31]==0)
//                             begin
//                                 extender_out = 32'b0;
//                                 carry = B[31];
//                             end
//                         else
//                             begin
//                                 extender_out = 32'hFFFFFFFF;
//                                 carry = B[31];
//                             end
//                     end
//                     else        
//                         begin
//                         extender_out = $signed(B) >>> instruction[11:7];
//                         carry = B[instruction[11:7]-1];
//                         end
//                 end
//             else if(instruction[6:5]== 2'b11)
//                     begin
//                          extender_out =  B >> instruction[11:7];
//                          carry = B[instruction[11:7]-1];
//                     end
//         end
//         else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b10 && instruction[24]== 1'b1)       
//                 extender_out = (instruction[11:8]<<4) | instruction[3:0]; 

//         else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b00 && instruction[24]== 1'b1)
//                 extender_out = B;

//         else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b11 && instruction[24]== 1'b1)
//                extender_out =  (instruction[11:8]<<4) | instruction[3:0];

//         else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b01 && instruction[24]== 1'b1)
//                 extender_out = B;

//         else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b10 && instruction[24]== 1'b0)   
//                 extender_out = (instruction[11:8]<<4) | instruction[3:0];

//         else if(instruction[4]==1'b1 && instruction[7]==1'b1 && instruction[22:21]==2'b00 && instruction[24]== 1'b0)
//                 extender_out = B;
//     end
//     3'b101:  begin
//         {extender_out} = {{6{instruction[23]}},instruction[23:0]} <<2;
//         end
//     //addressing mode 2
//     //immediate offset/pre/post
//     3'b010: begin
//             if((instruction[24]==1'b1 && instruction[21]==1'b0) || (instruction[24]==1'b1 && instruction[21]==1'b1) || (instruction[24]==1'b0 && instruction[21]==1'b0))
//                 extender_out = instruction[11:0];
//             end
//     //register offset/pre/post
//     3'b011: begin
//                 if(instruction[11:4]==8'b00000000)
//                     begin
//                        if((instruction[24]==1'b1 && instruction[21]==1'b0) || (instruction[24]==1'b1 && instruction[21]==1'b1) || (instruction[24]==1'b0 && instruction[21]==1'b0))
//                             extender_out = B; 
//                     end
//             end
//     endcase
        

// end           
// endmodule


// module test;

// reg [31:0] inst;
// reg cin;
// wire [31:0] out;
// wire carry;
// reg [31:0] RM;

// initial begin
//    cin = 1'b0; 
//    RM = 32'b00000000000000000000000000000011;
// end

// initial begin
//  inst = 32'b00000000000001111111000000000000;
// //10 inst = 32'b0000000101010000000000111001111;
// end

// shift_sign_extender sse(out,carry,inst,RM, cin);

// initial begin
// $display("Testing");
// $monitor("out:%b    inst: %b   carry:%b", out, inst, carry);
// end

// endmodule
