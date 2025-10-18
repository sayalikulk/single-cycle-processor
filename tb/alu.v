`default_nettype none

// The arithmetic logic unit (ALU) is responsible for performing the core
// calculations of the processor. It takes two 32-bit operands and outputs
// a 32 bit result based on the selection operation - addition, comparison,
// shift, or logical operation. This ALU is a purely combinational block, so
// you should not attempt to add any registers or pipeline it in phase 3.
module alu (
    // Major operation selection.
    // NOTE: In order to simplify instruction decoding in phase 4, both 3'b010
    // and 3'b011 are used for set less than (they are equivalent).
    // Unsigned comparison is controlled through the `i_unsigned` signal.
    //
    // 3'b000: addition/subtraction if `i_sub` asserted
    // 3'b001: shift left logical
    // 3'b010,
    // 3'b011: set less than/unsigned if `i_unsigned` asserted
    // 3'b100: exclusive or
    // 3'b101: shift right logical/arithmetic if `i_arith` asserted
    // 3'b110: or
    // 3'b111: and
    input  wire [3:0] i_opsel,
    // When asserted, addition operations should subtract instead.
    // This is only used for `i_opsel == 3'b0000` (addition/subtraction).
    // When asserted, comparison operations should be treated as unsigned.
    // This is only used for branch comparisons and set less than.
    // For branch operations, the ALU result is not used, only the comparison
    // results.
    // When asserted, right shifts should be treated as arithmetic instead of
    // logical. This is only used for `i_opsel == 3'b011` (shift right).
    // First 32-bit input operand.
    input  wire [31:0] i_op1,
    // Second 32-bit input operand.
    input  wire [31:0] i_op2,
    // 32-bit output result. Any carry out (from addition) should be ignored.
    output wire [31:0] o_result,
    // Equality result. This is used downstream to determine if a
    // branch should be taken.
    // Set less than result. This is used downstream to determine if a
    // branch should be taken.
    output wire   alu_zero // branch
);
    wire [31:0] add = (i_op1 + i_op2); //Addition or subtraction based on i_sub signal
    wire [31:0] sub = (i_op1 - i_op2);
    wire [4:0] sh_amt = i_op2[4:0]; //Shift amount is determined by the lower 5 bits of i_op2
    wire [31:0] sll = i_op1 << sh_amt; //Shift left logical
    wire [31:0] srl = i_op1 >> sh_amt; //Shift right logical
    wire [31:0] sra = $signed(i_op1) >>> sh_amt; //Shift right arithmetic
    wire [31:0] slt_signed = ($signed(i_op1) < $signed(i_op2)) ? 32'b1 : 32'b0; //Set less than for signed comparison
    wire [31:0] slt_unsigned = (i_op1 < i_op2) ? 32'b1 : 32'b0; //Set less than for unsigned comparison
    wire [31:0] xor_res = i_op1 ^ i_op2; //Bitwise XOR
    wire [31:0] or_res = i_op1 | i_op2; //Bitwise OR
    wire [31:0] and_res = i_op1 & i_op2; //Bitwise AND

    //Equality check
    assign o_eq = (i_op1 == i_op2);
    //Set less than result based on signed or unsigned comparison
    assign o_slt = i_unsigned ? (i_op1 < i_op2) : ($signed(i_op1) < $signed(i_op2));

    //Selecting the appropriate operation result based on i_opsel
    assign o_result = (i_opsel == 4'b0000) ? add :
                      (i_opsel == 4'b0001) ? sub :
                      (i_opsel == 4'b0010) ? sll :
                      (i_opsel == 4'b0011 || i_opsel == 4'b0100) ? (i_unsigned ? slt_unsigned : slt_signed) :
                      (i_opsel == 4'b0101) ? xor_res :
                      (i_opsel == 4'b0110) ? (i_arith ? sra  : srl) :
                      (i_opsel == 4'b0111) ? or_res :
                      (i_opsel == 4'b10000) ? and_res :
                      32'b0; //Default case (should not occur)                      
endmodule

`default_nettype wire
