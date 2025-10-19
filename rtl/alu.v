module ALU (
    input wire [3:0] i_opsel,    // 4-bit ALU operation select
    input wire [31:0] i_op1,     // First ALU operand
    input wire [31:0] i_op2,     // Second ALU operand
    output wire [31:0] o_result,  // ALU result
    output wire alu_zero,         // ALU zero flag
    input wire [31:0] branch_target
);

    // Combinational logic for ALU operations
    assign o_result =   (i_opsel == 4'b0000) ? (i_op1 + i_op2) :                // ADD
                        (i_opsel == 4'b0001) ? (i_op1 - i_op2) :                // SUB
                        (i_opsel == 4'b0010) ? (i_op1 << i_op2[4:0]) :          // SLL
                        (i_opsel == 4'b0011) ? (($signed(i_op1) < $signed(i_op2)) ? 32'b1 : 32'b0) : // SLT
                        (i_opsel == 4'b0100) ? ((i_op1 < i_op2) ? 32'b1 : 32'b0) :                  // SLTU
                        (i_opsel == 4'b0101) ? (i_op1 ^ i_op2) :                // XOR
                        (i_opsel == 4'b0110) ? (i_op1 >> i_op2[4:0]) :          // SRL
                        (i_opsel == 4'b0111) ? ($signed(i_op1) >>> i_op2[4:0]) :// SRA
                        (i_opsel == 4'b1000) ? (i_op1 | i_op2) :                // OR
                        (i_opsel == 4'b1001) ? (i_op1 & i_op2) :                // AND
                        (i_opsel == 4'b1010) ? (i_op2) :                        // LUI (Pass B)
                        32'b0;                                                  // Default

    // Zero flag logic
    assign alu_zero = (o_result == 32'b0) ? 1'b1 : 1'b0;
    

endmodule
