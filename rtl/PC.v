//Program Counter with Continuous Adding and Branch Support
module PC(
    input wire i_clk, // Clock Input from hart
    input wire i_rst, // Reset Input from hart
    input wire [31:0] i_next_pc, // Next PC value from hart
    output reg [31:0] current_pc // Next PC value to be sent to hart
    
);

    always @(posedge i_clk) begin
        if (i_rst)       // Synchronous Reset
            current_pc <= 32'b0;
        else             // Otherwise, increment PC by 4(memory is byte-addressable)
            current_pc <= i_next_pc; 
    end

endmodule