`default_nettype none

// The immediate generator is responsible for decoding the 32-bit
// sign-extended immediate from the incoming instruction word. It is a purely
// combinational block that is expected to be embedded in the instruction
// decoder.
module imm (
    // Input instruction word. This is used to extract the relevant immediate
    // bits and assemble them into the final immediate.
    input  wire [31:0] i_inst,

    // Output 32-bit immediate, sign-extended from the immediate bitstring.
    // Because the R-type format does not have an immediate, the output
    // immediate can be treated as a don't-care under this case. It is
    // included for completeness.
    output wire [31:0] o_immediate,
    output wire [31:0] branch_target
);
    // Fill in your implementation here.

    wire [31:0] imm_i; //I-type immediate
    wire [31:0] imm_s; //S-type immediate
    wire [31:0] imm_u; //U-type immediate
    wire [31:0] imm_j; //J-type immediate

    assign imm_i = {{21{i_inst[31]}}, i_inst[30:20]}; //Sign-extend and assign bits [31:20]
    assign imm_s = {{21{i_inst[31]}}, i_inst[30:25], i_inst[11:7]}; //Sign-extend and assign bits [31:25] and [11:7]
    assign branch_target = {{20{i_inst[31]}}, i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0}; //Sign-extend and assign bits [31], [7], [30:25], [11:8] and append 0 at LSB
    assign imm_u = {i_inst[31:12], 12'b0}; //Assign bits [31:12] and append 12 zeros at LSB
    assign imm_j = {{12{i_inst[31]}}, i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0}; //Sign-extend and assign bits [31], [19:12], [20], [30:21] and append 0 at LSB

    //Mux to select the appropriate immediate based on instruction format
    assign o_immediate = (i_inst[6:0] == 7'b0010011 || //I-type instructions
                          i_inst[6:0] == 7'b0000011 || //Load instructions
                          i_inst[6:0] == 7'b1100111)   //JALR instruction
                         ? imm_i :
                         (i_inst[6:0] == 7'b0100011)   //S-type instructions
                         ? imm_s :
                         (i_inst[6:0] == 7'b0110111 || //LUI instruction
                          i_inst[6:0] == 7'b0010111)   //AUIPC instruction
                         ? imm_u :
                         (i_inst[6:0] == 7'b1100111)   //JALR instruction
                         ? imm_j :
                         32'b0; //Default case (R-type instructions)

endmodule

`default_nettype wire
