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

    //Wires and reg declaration
    wire [31:0] o_imm; //Immediate value from immediate generation module
    wire [31:0] o_rs2; //Second output from register file
    wire [31:0] i_op1; //First input to ALU from register file
    wire [31:0] i_op2; //Second input to ALU from mux between register file and immediate value
    wire [3:0] i_opsel; //ALU operation selection from ALU control module
    wire alu_zero; //ALU zero flag output
    wire ALU_src; //ALU source selection signal from control unit
    wire mem_to_reg;  //Memory to register selection signal from control unit
    wire reg_write; //Register write enable signal from control unit
    wire [2:0] ALU_OP; //ALU operation selection bits from control  
    wire Branch;  //Branch will coming from control unit, anded with ALU_zero to generate en_branch
    wire Jump;
    wire Jalr;
    wire load_upper_imm;
    wire upper_imm;
    
    //Control unit module instantiation
    // Fill in your implementation here.
    PC pc_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .current_pc(o_imem_raddr),
        .en_branch(en_branch),
        .branch_target(branch_target)
    );

    Control_unit control_unit_inst (
        .opcode(i_imem_rdata[6:0]),
        .alu_src(ALU_src), //ALU source selection signal from control unit and will be connected to ALU module
        .mem_to_reg(mem_to_reg),  //Memory to register selection signal from control unit and will be connected to multiplexer before writeback stage
        .reg_write(reg_write), //Register write enable signal from control unit and will be connected to register file
        .mem_read(/* Connect to appropriate wire */), //Memory interface is already provided in testbench
        .mem_write(/* Connect to appropriate wire */), //Memory interface is already provided in testbench ?? what to connect here?? 
        .branch(Branch),  //Branch will coming from control unit, anded with ALU_zero to generate en_branch
        .Jump(Jump),
        .Jalr(Jalr),
        .load_upper_imm(load_upper_imm),
        .ALU_OP(ALU_OP),
        .upper_imm(upper_imm)
    );

    // Program counter module instantiation
    program_counter pc_inst (
        .clk(i_clk),
        .rst_n(i_rst),
        .branch_target(/* Connect to branch target address wire */),
        .branch_taken(Branch & alu_zero), // Branch taken signal generated by ANDing Branch
        .pc_out(o_imem_raddr)
    );

        //Immediate generation module instantiation
    imm_gen imm_gen_inst (
        .i_inst(i_imem_rdata),
        .o_imm(o_imm)
    );

    // Assign Second Input of ALU based on ALU_src signal from control unit
    assign i_op2 = (ALU_src) ? o_imm : o_rs2;

    alu alu_inst (
        .i_opsel(i_opsel),
        .i_op1(i_op1),
        .i_op2(i_op2),
        .o_result(o_result),
        .alu_zero(alu_zero)
    );

    // Register file module instantiation
    rf #(
        .BYPASS_EN(1) // Enable bypassing for this implementation
    ) rf_inst (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_rs1_raddr(i_imem_rdata[19:15]), // rs1 address from instruction
        .o_rs1_rdata(i_op1), // Connect to ALU first operand
        .i_rs2_raddr(i_imem_rdata[24:20]), // rs2 address from instruction
        .o_rs2_rdata(o_rs2), // Connect to ALU second operand or store data
        .i_rd_wen(reg_write), // Register write enable from control unit
        .i_rd_waddr(i_imem_rdata[11:7]), // rd address from instruction
        .i_rd_wdata(/* Connect to data to be written back to register file */)
    );

    //ALU control module instantiation
    alu_control alu_control_inst (
        .i_alu_op(ALU_OP),
        .i_funct3(i_imem_rdata[14:12]),
        .i_funct7(i_imem_rdata[31:25]),
        .o_alu_control(i_opsel)
    );

    // Additional modules and logic to complete the hart implementation
    // such as register file, data memory interface, writeback logic, etc.

    // Ensure to connect all outputs to the corresponding retire signals
    
    assign o_retire_valid = /* Connect to valid signal */;
    assign o_retire_inst = i_imem_rdata;
    assign o_retire_trap = /* Connect to trap signal */;
    assign o_retire_halt = /* Connect to halt signal */;
    assign o_retire_rs1_raddr = /* Connect to rs1 address from instruction */;
    assign o_retire_rs2_raddr = /* Connect to rs2 address from instruction */;
    assign o_retire_rs1_rdata = /* Connect to rs1 data from register file` */;
    assign o_retire_rs2_rdata = /* Connect to rs2 data from register file */;
    assign o_retire_rd_waddr = /* Connect to rd address from instruction */;    
    assign o_retire_rd_wdata = /* Connect to data being written back to register file */;
    assign o_retire_pc = /* Connect to current PC value */;
    assign o_retire_next_pc = /* Connect to next PC value */;
    assign o_dmem_addr = /* Connect to data memory address */;
    assign o_dmem_ren = /* Connect to data memory read enable */;
    assign o_dmem_wen = /* Connect to data memory write enable */;
    assign o_dmem_wdata = /* Connect to data memory write data */;
    assign o_dmem_mask = /* Connect to data memory mask */;
    

    // end of hart module

endmodule

`default_nettype wire
