`default_nettype none

// The immediate generator is responsible for decoding the 32-bit
// sign-extended immediate from the incoming instruction word. It is a purely
// combinational block that is expected to be embedded in the instruction
// decoder.
module imm (
    // Input instruction word. This is used to extract the relevant immediate
    // bits and assemble them into the final immediate.
    input  wire [31:0] i_inst,

    input wire [31:0] i_op1,

    // Output 32-bit immediate, sign-extended from the immediate bitstring.
    // Because the R-type format does not have an immediate, the output
    // immediate can be treated as a don't-care under this case. It is
    // included for completeness.
    output wire [31:0] o_immediate
);
    // Fill in your implementation here.

    wire [31:0] imm_i; //I-type immediate
    wire [31:0] imm_s; //S-type immediate
    wire [31:0] imm_u; //U-type immediate
    wire [31:0] imm_j; //J-type immediate
    wire [31:0] imm_b; //B-type immediate
    wire [31:0] imm_jr; //JALR

    assign imm_i = {{21{i_inst[31]}}, i_inst[30:20]}; //Sign-extend and assign bits [31:20]
    assign imm_s = {{21{i_inst[31]}}, i_inst[30:25], i_inst[11:7]}; //Sign-extend and assign bits [31:25] and [11:7]
    assign imm_b = {{20{i_inst[31]}}, i_inst[7], i_inst[30:25], i_inst[11:8], 1'b0}; //Sign-extend and assign bits [31], [7], [30:25], [11:8] and append 0 at LSB
    assign imm_u = {i_inst[31:12], 12'h000}; //Assign bits [31:12] and append 12 zeros at LSB
    assign imm_j = {{12{i_inst[31]}}, i_inst[19:12], i_inst[20], i_inst[30:21], 1'b0}; //Sign-extend and assign bits [31], [19:12], [20], [30:21] and append 0 at LSB
    assign imm_jr = i_op1 + imm_i;

       // Opcodes
    localparam OPC_I_ARITH = 7'b0010011;
    localparam OPC_LOAD     = 7'b0000011;
    localparam OPC_JALR     = 7'b1100111;
    localparam OPC_STORE    = 7'b0100011;
    localparam OPC_LUI      = 7'b0110111;
    localparam OPC_AUIPC    = 7'b0010111;
    localparam OPC_JAL      = 7'b1101111;
    localparam OPC_BRANCH   = 7'b1100011; 

    assign o_immediate = (i_inst[6:0] == OPC_I_ARITH  ||        // I-type arithmetic
                          i_inst[6:0]  == OPC_LOAD)   ? imm_i : // loads
                         (i_inst[6:0]  == OPC_JALR)   ? {imm_jr[31:1],1'b0} : // JALR (I-type)
                         (i_inst[6:0] == OPC_STORE)   ? imm_s :
                         (i_inst[6:0] == OPC_LUI      || 
                          i_inst[6:0] == OPC_AUIPC)   ? imm_u :
                         (i_inst[6:0] == OPC_JAL)     ? imm_j :
                         (i_inst[6:0] == OPC_BRANCH)  ? imm_b :
                         32'b0;
endmodule

`default_nettype wire
