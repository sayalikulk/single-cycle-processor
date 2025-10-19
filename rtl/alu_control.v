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
    output reg [3:0] o_alu_control
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

    // Combinational logic to determine ALU control signal
    always @(*) begin
        case (i_alu_op)
            OP_R_TYPE: begin
                case ({i_funct7[5], i_funct3})
                    4'b0000: o_alu_control = 4'b0000; // ADD
                    4'b1000: o_alu_control = 4'b0001; // SUB
                    4'b0001: o_alu_control = 4'b0010; // SLL
                    4'b0010: o_alu_control = 4'b0011; // SLT
                    4'b0011: o_alu_control = 4'b0100; // SLTU
                    4'b0100: o_alu_control = 4'b0101; // XOR
                    4'b0101: o_alu_control = 4'b0110; // SRL
                    4'b1101: o_alu_control = 4'b0111; // SRA
                    4'b0110: o_alu_control = 4'b1000; // OR
                    4'b0111: o_alu_control = 4'b1001; // AND
                    default: o_alu_control = 4'b0000; // Default to ADD
                endcase
            end
            OP_I_TYPE: begin
                case (i_funct3)
                    3'b000: o_alu_control = 4'b0000; // ADDI
                    3'b010: o_alu_control = 4'b0011; // SLTI
                    3'b011: o_alu_control = 4'b0100; // SLTIU
                    3'b100: o_alu_control = 4'b0101; // XORI
                    3'b110: o_alu_control = 4'b1000; // ORI
                    3'b111: o_alu_control = 4'b1001; // ANDI
                    3'b001: o_alu_control = 4'b0010; // SLLI
                    3'b101: begin
                        if (i_funct7[5] == 1'b0)
                            o_alu_control = 4'b0110; // SRLI
                        else
                            o_alu_control = 4'b0111; // SRAI
                    end
                    default: o_alu_control = 4'b0000; // Default to ADDI
                endcase
            end
            OP_LOAD: o_alu_control = 4'b0000; // ADD for address calculation
            OP_STORE: o_alu_control = 4'b0000; // ADD for address calculation
            OP_LOAD_UPPER_IMM: o_alu_control = 4'b1010; // LUI
            OP_ADD_UPPER_IMM: o_alu_control = 4'b0000; // AUIPC (ADD)
            OP_JUMP: o_alu_control = 4'b0000; // ADD for JAL/JALR address calculation
            OP_BRANCH: begin
                case (i_funct3)
                    3'b000: o_alu_control = 4'b0001; // SUB for BEQ
                    3'b001: o_alu_control = 4'b0001; // SUB for BNE
                    3'b100: o_alu_control = 4'b0011; // SLT for BLT
                    3'b101: o_alu_control = 4'b0011; // SLT for BGE
                    3'b110: o_alu_control = 4'b0100; // SLTU for BLTU
                    3'b111: o_alu_control = 4'b0100; // SLTU for BGEU
                    default: o_alu_control = 4'b0000; // Default to ADD
                endcase
            end
            default: o_alu_control = 4'b0000; // Default to ADD
        endcase
    end


endmodule