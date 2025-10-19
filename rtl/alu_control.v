`default_nettype none

/*
 * ALU Control Unit for RISC-V
 *
 * This module translates the 2-bit ALU_OP (from Main Control) and the
 * instruction's funct3/funct7 bits into a 4-bit control signal
 * that directly tells the ALU which operation to perform.
 */
module alu_control (
    // Input from Main Control Unit
    input wire [2:0] i_alu_op,
    
    // Inputs from the instruction word
    input wire [2:0] i_funct3,
    input wire [6:0] i_funct7,
    
    // Output to the ALU's operation select port
    output wire [3:0] o_alu_control
);
    // ALU Operation Codes
    localparam OP_R_TYPE = 3'b000;
    localparam OP_I_TYPE = 3'b001;
    localparam OP_LOAD   = 3'b010;
    localparam OP_STORE  = 3'b011;
    localparam OP_BRANCH = 3'b100;
    localparam OP_LOAD_UPPER_IMM = 3'b101;
    localparam OP_ADD_UPPER_IMM  = 3'b110;
    localparam OP_JUMP  = 3'b111;

    assign o_alu_control =  (i_alu_op == OP_R_TYPE) ? (
                            ({i_funct7[5], i_funct3} == 4'b0000) ? 4'b0000 : // ADD
                            ({i_funct7[5], i_funct3} == 4'b1000) ? 4'b0001 : // SUB
                            ({i_funct7[5], i_funct3} == 4'b0001) ? 4'b0010 : // SLL
                            ({i_funct7[5], i_funct3} == 4'b0010) ? 4'b0011 : // SLT
                            ({i_funct7[5], i_funct3} == 4'b0011) ? 4'b0100 : // SLTU
                            ({i_funct7[5], i_funct3} == 4'b0100) ? 4'b0101 : // XOR
                            ({i_funct7[5], i_funct3} == 4'b0101) ? 4'b0110 : // SRL
                            ({i_funct7[5], i_funct3} == 4'b1101) ? 4'b0111 : // SRA
                            ({i_funct7[5], i_funct3} == 4'b0110) ? 4'b1000 : // OR
                            ({i_funct7[5], i_funct3} == 4'b0111) ? 4'b1001 : // AND
                            4'b0000 // Default (ADD)
                            ) :

                            (i_alu_op == OP_I_TYPE) ? (
                            (i_funct3 == 3'b000) ? 4'b0000 : // ADDI
                            (i_funct3 == 3'b010) ? 4'b0011 : // SLTI
                            (i_funct3 == 3'b011) ? 4'b0100 : // SLTIU
                            (i_funct3 == 3'b100) ? 4'b0101 : // XORI
                            (i_funct3 == 3'b110) ? 4'b1000 : // ORI
                            (i_funct3 == 3'b111) ? 4'b1001 : // ANDI
                            (i_funct3 == 3'b001) ? 4'b0010 : // SLLI
                            (i_funct3 == 3'b101 && i_funct7[5] == 1'b0) ? 4'b0110 : // SRLI
                            (i_funct3 == 3'b101 && i_funct7[5] == 1'b1) ? 4'b0111 : // SRAI
                            4'b0000 // Default (ADDI)
                            ) :

                            (i_alu_op == OP_LOAD) ? 4'b0000  : // ADD for address calc

                            (i_alu_op == OP_STORE) ? 4'b0000 : // ADD for address calc

                            (i_alu_op == OP_LOAD_UPPER_IMM) ? 4'b1010 : // LUI

                            (i_alu_op == OP_ADD_UPPER_IMM) ? 4'b0000 : // AUIPC (ADD)

                            (i_alu_op == OP_JUMP) ? 4'b0000 : // ADD for JAL/JALR address

                            (i_alu_op == OP_BRANCH) ? (
                                (i_funct3 == 3'b000) ? 4'b0001 : // BEQ (SUB)
                                (i_funct3 == 3'b001) ? 4'b0001 : // BNE (SUB)
                                (i_funct3 == 3'b100) ? 4'b0011 : // BLT (SLT)
                                (i_funct3 == 3'b101) ? 4'b0011 : // BGE (SLT)
                                (i_funct3 == 3'b110) ? 4'b0100 : // BLTU (SLTU)
                                (i_funct3 == 3'b111) ? 4'b0100 : // BGEU (SLTU)
                                4'b0000 // Default
                            ) :
                            4'b0000; // Global default


endmodule