module hart #(
    // After reset, the program counter (PC) should be initialized to this
    // address and start executing instructions from there.
    parameter RESET_ADDR = 32'h00000000
) (
    // Global clock.
    input  wire        i_clk,
    // Synchronous active-high reset.
    input  wire        i_rst,
    // Instruction fetch goes through a read only instruction memory (imem)
    // port. The port accepts a 32-bit address (e.g. from the program counter)
    // per cycle and combinationally returns a 32-bit instruction word. This
    // is not representative of a realistic memory interface; it has been
    // modeled as more similar to a DFF or SRAM to simplify phase 3. In
    // later phases, you will replace this with a more realistic memory.
    //
    // 32-bit read address for the instruction memory. This is expected to be
    // 4 byte aligned - that is, the two LSBs should be zero.
    output wire [31:0] o_imem_raddr,
    // Instruction word fetched from memory, available on the same cycle.
    input  wire [31:0] i_imem_rdata,
    // Data memory accesses go through a separate read/write data memory (dmem)
    // that is shared between read (load) and write (stored). The port accepts
    // a 32-bit address, read or write enable, and mask (explained below) each
    // cycle. Reads are combinational - values are available immediately after
    // updating the address and asserting read enable. Writes occur on (and
    // are visible at) the next clock edge.
    //
    // Read/write address for the data memory. This should be 32-bit aligned
    // (i.e. the two LSB should be zero). See `o_dmem_mask` for how to perform
    // half-word and byte accesses at unaligned addresses.
    output wire [31:0] o_dmem_addr,
    // When asserted, the memory will perform a read at the aligned address
    // specified by `i_addr` and return the 32-bit word at that address
    // immediately (i.e. combinationally). It is illegal to assert this and
    // `o_dmem_wen` on the same cycle.
    output wire        o_dmem_ren,
    // When asserted, the memory will perform a write to the aligned address
    // `o_dmem_addr`. When asserted, the memory will write the bytes in
    // `o_dmem_wdata` (specified by the mask) to memory at the specified
    // address on the next rising clock edge. It is illegal to assert this and
    // `o_dmem_ren` on the same cycle.
    output wire        o_dmem_wen,
    // The 32-bit word to write to memory when `o_dmem_wen` is asserted. When
    // write enable is asserted, the byte lanes specified by the mask will be
    // written to the memory word at the aligned address at the next rising
    // clock edge. The other byte lanes of the word will be unaffected.
    output wire [31:0] o_dmem_wdata,
    // The dmem interface expects word (32 bit) aligned addresses. However,
    // WISC-25 supports byte and half-word loads and stores at unaligned and
    // 16-bit aligned addresses, respectively. To support this, the access
    // mask specifies which bytes within the 32-bit word are actually read
    // from or written to memory.
    //
    // To perform a half-word read at address 0x00001002, align `o_dmem_addr`
    // to 0x00001000, assert `o_dmem_ren`, and set the mask to 0b1100 to
    // indicate that only the upper two bytes should be read. Only the upper
    // two bytes of `i_dmem_rdata` can be assumed to have valid data; to
    // calculate the final value of the `lh[u]` instruction, shift the rdata
    // word right by 16 bits and sign/zero extend as appropriate.
    //
    // To perform a byte write at address 0x00002003, align `o_dmem_addr` to
    // `0x00002003`, assert `o_dmem_wen`, and set the mask to 0b1000 to
    // indicate that only the upper byte should be written. On the next clock
    // cycle, the upper byte of `o_dmem_wdata` will be written to memory, with
    // the other three bytes of the aligned word unaffected. Remember to shift
    // the value of the `sb` instruction left by 24 bits to place it in the
    // appropriate byte lane.
    output wire [ 3:0] o_dmem_mask,
    // The 32-bit word read from data memory. When `o_dmem_ren` is asserted,
    // this will immediately reflect the contents of memory at the specified
    // address, for the bytes enabled by the mask. When read enable is not
    // asserted, or for bytes not set in the mask, the value is undefined.
    input  wire [31:0] i_dmem_rdata,
	// The output `retire` interface is used to signal to the testbench that
    // the CPU has completed and retired an instruction. A single cycle
    // implementation will assert this every cycle; however, a pipelined
    // implementation that needs to stall (due to internal hazards or waiting
    // on memory accesses) will not assert the signal on cycles where the
    // instruction in the writeback stage is not retiring.
    //
    // Asserted when an instruction is being retired this cycle. If this is
    // not asserted, the other retire signals are ignored and may be left invalid.
    output wire        o_retire_valid,
    // The 32 bit instruction word of the instrution being retired. This
    // should be the unmodified instruction word fetched from instruction
    // memory.
    output wire [31:0] o_retire_inst,
    // Asserted if the instruction produced a trap, due to an illegal
    // instruction, unaligned data memory access, or unaligned instruction
    // address on a taken branch or jump.
    output wire        o_retire_trap,
    // Asserted if the instruction is an `ebreak` instruction used to halt the
    // processor. This is used for debugging and testing purposes to end
    // a program.
    output wire        o_retire_halt,
    // The first register address read by the instruction being retired. If
    // the instruction does not read from a register (like `lui`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs1_raddr,
    // The second register address read by the instruction being retired. If
    // the instruction does not read from a second register (like `addi`), this
    // should be 5'd0.
    output wire [ 4:0] o_retire_rs2_raddr,
    // The first source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs1 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs1_rdata,
    // The second source register data read from the register file (in the
    // decode stage) for the instruction being retired. If rs2 is 5'd0, this
    // should also be 32'd0.
    output wire [31:0] o_retire_rs2_rdata,
    // The destination register address written by the instruction being
    // retired. If the instruction does not write to a register (like `sw`),
    // this should be 5'd0.
    output wire [ 4:0] o_retire_rd_waddr,
    // The destination register data written to the register file in the
    // writeback stage by this instruction. If rd is 5'd0, this field is
    // ignored and can be treated as a don't care.
    output wire [31:0] o_retire_rd_wdata,
    // The current program counter of the instruction being retired - i.e.
    // the instruction memory address that the instruction was fetched from.
    output wire [31:0] o_retire_pc,
    // the next program counter after the instruction is retired. For most
    // instructions, this is `o_retire_pc + 4`, but must be the branch or jump
    // target for *taken* branches and jumps.
    output wire [31:0] o_retire_next_pc

`ifdef RISCV_FORMAL
    ,`RVFI_OUTPUTS,
`endif
);
    // --- Internal signals ---
    wire [31:0] o_imm, o_rs2, i_op1, i_op2, o_result;
    wire [3:0]  i_opsel;
    wire alu_zero, ALU_src, mem_to_reg, reg_write;
    wire [2:0] ALU_OP;
    wire Branch, Jump, Jalr, load_upper_imm, upper_imm;
    wire en_branch;
    wire [31:0] branch_target;
    wire [31:0] writeback_data;

    // --- Program Counter ---
    PC pc_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .current_pc(o_imem_raddr),
        .en_branch(en_branch),
        .branch_target(branch_target)
    );

    
    // --- Control Unit ---
    Control_unit control_unit_inst (
        .opcode(i_imem_rdata[6:0]),
        .ALU_zero(ALU_zero),
        .alu_src(ALU_src),
        .mem_to_reg(mem_to_reg),
        .reg_write(reg_write),
        .mem_read(o_dmem_ren),
        .mem_write(o_dmem_wen),
        .Branch(Branch),
        .Jump(Jump),
        .Jalr(Jalr),
        .load_upper_imm(load_upper_imm),
        .ALU_OP(ALU_OP),
        .upper_imm(upper_imm),
        .en_branch(en_branch)
    );

    // --- Immediate Generator ---
    imm imm_gen_inst (
        .i_inst(i_imem_rdata),
        .branch_target(branch_target),
        .o_immediate(o_imm)
    );

    // --- ALU Control ---
    alu_control alu_control_inst (
        .i_alu_op(ALU_OP),
        .i_funct3(i_imem_rdata[14:12]),
        .i_funct7(i_imem_rdata[31:25]),
        .o_alu_control(i_opsel)
    );

    // --- Register File ---
    reg_file rf (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_rs1_raddr(i_imem_rdata[19:15]),
        .o_rs1_rdata(i_op1),
        .i_rs2_raddr(i_imem_rdata[24:20]),
        .o_rs2_rdata(o_rs2),
        .i_rd_wen(reg_write),
        .i_rd_waddr(i_imem_rdata[11:7]),
        .i_rd_wdata(writeback_data)
    );

    // --- ALU ---
    assign i_op2 = (ALU_src) ? o_imm : o_rs2;
    ALU alu_inst (
        .i_opsel(i_opsel),
        .i_op1(i_op1),
        .i_op2(i_op2),
        .o_result(o_result),
        .alu_zero(alu_zero)
    );

    // --- Branch logic ---
    //assign en_branch    = Branch & alu_zero;
    //assign branch_target = o_imm; // simple branch offset

    // --- Writeback ---
    assign writeback_data = (mem_to_reg) ? i_dmem_rdata : o_result;
    wire ebreak = (i_imem_rdata == 32'h00100073); // ebreak instruction
    assign o_retire_halt      = ebreak;
    wire rf_wen; // Disable register file write on ebreak
    assign rf_wen = reg_write & ~ebreak;
    assign o_dmem_wen = o_dmem_wen & ~ebreak;
    assign o_dmem_ren = o_dmem_ren & ~ebreak;
    
    // --- Data Memory interface ---
    assign o_dmem_addr  = o_result;
    assign o_dmem_wdata = o_rs2;
    assign o_dmem_mask  = 4'b1111; // simple word access for now
    

    /* --- Retire interface ---
    assign o_retire_valid     = 1'b1;
    assign o_retire_inst      = i_imem_rdata;
    assign o_retire_trap      = 1'b0;
    assign o_retire_halt      = 1'b0;
    assign o_retire_rs1_raddr = i_imem_rdata[19:15];
    assign o_retire_rs2_raddr = i_imem_rdata[24:20];
    assign o_retire_rs1_rdata = i_op1;
    assign o_retire_rs2_rdata = o_rs2;
    assign o_retire_rd_waddr  = i_imem_rdata[11:7];
    assign o_retire_rd_wdata  = writeback_data;
    assign o_retire_pc        = o_imem_raddr;
    assign o_retire_next_pc   = o_imem_raddr + 4;
    */
endmodule


`default_nettype wire
