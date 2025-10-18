//Program Counter with Continuous Adding and Branch Support
module PC(
    input wire i_clk, // Clock Input from hart
    input wire i_rst, // Reset Input from hart
    input wire [31:0] current_pc, // Current PC value from hart
    input wire en_branch, //Branch signal from control unit (ALU_zero && branch)
    input wire [31:0] branch_target, // Target address for branch
    output reg [31:0] next_pc // Next PC value to be sent to hart
);

    always @(posedge i_clk) begin
        if (i_rst)       // Synchronous Reset
            next_pc <= 32'b0;
        else if (branch) // If branch is taken, update PC to branch target
            next_pc <= branch_target;
        else             // Otherwise, increment PC by 4(memory is byte-addressable)
            next_pc <= current_pc + 4; 
    end

endmodule