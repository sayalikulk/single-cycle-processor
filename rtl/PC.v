//Program Counter with Continuous Adding and Branch Support
module PC(
    input wire i_clk, // Clock Input from hart
    input wire i_rst, // Reset Input from hart
    // output reg [31:0] current_pc, // Current PC value from hart
    input wire en_branch, //Branch signal from control unit (ALU_zero && branch)
    input wire [31:0] branch_target, // Target address for branch
    input wire i_ebreak, // Ebreak signal from control unit
    output reg [31:0] current_pc // Next PC value to be sent to hart
);

    always @(posedge i_clk) begin
        if (i_rst)       // Synchronous Reset
            current_pc <= 32'h00000000;
        else if (i_ebreak) // If ebreak is encountered, halt the CPU
            current_pc <= current_pc; // Hold PC value (could also implement halt logic)
        else if (en_branch) // If branch is taken, update PC to branch target
            current_pc <= current_pc + branch_target;
        else             // Otherwise, increment PC by 4(memory is byte-addressable)
            current_pc <= current_pc + 4; 
    end

endmodule