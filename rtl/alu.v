module ALU (
    input wire [3:0] i_opsel,    // 4-bit ALU operation select
    input wire [31:0] i_op1,     // First ALU operand
    input wire [31:0] i_op2,     // Second ALU operand
    output reg [31:0] o_result,  // ALU result
    output wire alu_zero         // ALU zero flag
);

    // Combinational logic for ALU operations
    always @(*) begin
        case (i_opsel)
            4'b0000: o_result = i_op1 + i_op2;               // ADD
            4'b0001: o_result = i_op1 - i_op2;               // SUB
            4'b0010: o_result = i_op1 << i_op2[4:0];        // SLL
            4'b0011: o_result = ($signed(i_op1) < $signed(i_op2)) ? 32'b1 : 32'b0; // SLT
            4'b0100: o_result = (i_op1 < i_op2) ? 32'b1 : 32'b0;               // SLTU
            4'b0101: o_result = i_op1 ^ i_op2;               // XOR
            4'b0110: o_result = i_op1 >> i_op2[4:0];        // SRL
            4'b0111: o_result = $signed(i_op1) >>> i_op2[4:0]; // SRA
            4'b1000: o_result = i_op1 | i_op2;               // OR
            4'b1001: o_result = i_op1 & i_op2;               // AND
            4'b1010: o_result = i_op2;                        // LUI (Pass B)
            default: o_result = 32'b0;                        // Default case
        endcase
    end

    // Zero flag logic
    assign alu_zero = (o_result == 32'b0) ? 1'b1 : 1'b0;
endmodule
