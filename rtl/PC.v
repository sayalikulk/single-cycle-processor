//Program Counter with Continuous Adding and Branch Support
module PC(
    input wire i_clk, // Clock Input from hart
    input wire i_rst, // Reset Input from hart
    // output reg [31:0] current_pc, // Current PC value from hart
    input wire en_branch, //Branch signal from control unit (ALU_zero && branch)
    input wire [31:0] branch_target, // Target address for branch
    output reg [31:0] current_pc // Next PC value to be sent to hart
);
    wire branch_PC;
    assign branch_PC = current_pc + branch_target;

    always @(posedge i_clk) begin
        if (i_rst)       // Synchronous Reset
            current_pc <= 32'h00000000;
        else if (en_branch) // If branch is taken, update PC to branch target
            current_pc <= branch_PC;
        else             // Otherwise, increment PC by 4(memory is byte-addressable)
            current_pc <= current_pc + 4; 
    end

endmodule