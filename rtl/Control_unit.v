/*
 * Control Unit for a RISC-V CPU
 *
 * Decodes the 7-bit instruction opcode to generate control signals
 * for the rest of the datapath.
 */
module Control_unit (
    input wire [6:0] opcode,        // 7-bit opcode from instruction[6:0]
    input wire ALU_zero,
    
    // Control Signals
    output wire mem_to_reg,   // Selects (memory data) or (ALU result) to write to register
    output wire reg_write,    // Enable writing to the register file
    output wire mem_read,     // Enable reading from data memory
    output wire mem_write,    // Enable writing to data memory
    output wire alu_src,      // Selects (register 2) or (immediate) as 2nd ALU input
    output wire [2:0] ALU_OP, // Control bits for the ALU Control unit
    
    // Instruction Type Signals (can be used for PC logic)
    output wire Branch,       // B-type (e.g., BEQ)
    output wire Jump,         // J-type (JAL)
    output wire Jalr,         // I-type (JALR)
    
    // U-Type Instruction Signals
    output wire load_upper_imm, // U-type (LUI)
    output wire upper_imm,      // U-type (AUIPC) - Renamed from original
    output wire en_branch
);

    // --- Opcode Definitions ---
    // Using localparam makes the code much more readable
    localparam OP_R_TYPE  = 7'b0110011; // R-type
    localparam OP_I_TYPE  = 7'b0010011; // I-type (Arithmetic)
    localparam OP_LOAD    = 7'b0000011; // I-type (Load)
    localparam OP_STORE   = 7'b0100011; // S-type (Store)
    localparam OP_BRANCH  = 7'b1100011; // B-type (Branch)
    localparam OP_JALR    = 7'b1100111; // I-type (JALR)
    localparam OP_JUMP     = 7'b1101111; // J-type (JAL)
    localparam OP_LOAD_UPPER_IMM     = 7'b0110111; // U-type (LUI)
    localparam OP_ADD_UPPER_IMM   = 7'b0010111; // U-type (AUIPC)

    // --- 1. Instruction Type Decoding ---
    // Internal wires for primary instruction types
    wire r_type;
    wire load;
    wire i_type_arith;
    wire store;

    // Decode opcodes into 1-bit signals
    // Some are internal, others are outputs
    assign r_type         = (opcode == R_TYPE_OP);
    assign load           = (opcode == LOAD_OP);
    assign i_type_arith   = (opcode == I_TYPE_OP);
    assign store          = (opcode == STORE_OP);
    
    assign Branch         = (opcode == BRANCH_OP);  // B-type
    assign Jalr           = (opcode == JALR_OP);    // I-type
    assign Jump           = (opcode == JAL_OP);     // J-type
    assign load_upper_imm = (opcode == LUI_OP);     // U-type
    assign upper_imm      = (opcode == AUIPC_OP);   // U-type

    // --- 2. Composite Signal Generation ---
    // Group instruction types for easier control signal logic
    
    // I-type includes Loads, I-type Arithmetic, and JALR
    wire i_type;
    assign i_type = (load | i_type_arith | Jalr);
    
    // U-type includes LUI and AUIPC
    wire u_type;
    assign u_type = (load_upper_imm | upper_imm);


    // --- 3. Control Signal Generation ---
    
    // alu_src: 1 if 2nd ALU operand is immediate (I, S, U types)
    //          0 if 2nd ALU operand is register (R, B types)
    assign alu_src = (i_type | store | u_type);

    // mem_to_reg: 1 if writing data from memory to register (Loads)
    assign mem_to_reg = load;

    // reg_write: 1 for instructions that write to the register file (R, I, U, J)
    assign reg_write = (r_type | i_type | u_type | Jump);

    // mem_read: 1 for instructions that read from data memory (Loads)
    assign mem_read = load;

    // mem_write: 1 for instructions that write to data memory (Stores)
    assign mem_write = store;

    // ALU_OP: 3 bit control signal to ALU Control unit
    // 000: OP_R_TYPE (R-type)
    // 001: OP_I_TYPE (I-type)
    // 010: OP_LOAD
    // 011: OP_STORE
    // 100: OP_BRANCH
    // 101: OP_LOAD_UPPER_IMM (U-type)
    // 110: OP_ADD_UPPER_IMM (U-type)
    // 111: OP_JUMP (J-type)
    assign ALU_OP = (OP_R_TYPE) ? 3'b000 :
                    (OP_I_TYPE) ? 3'b001 :
                    (OP_LOAD)   ? 3'b010 :
                    (OP_STORE)  ? 3'b011 :
                    (OP_BRANCH) ? 3'b100 :
                    (OP_LOAD_UPPER_IMM) ? 3'b101 :
                    (OP_ADD_UPPER_IMM)  ? 3'b110 :
                    (OP_JUMP)   ? 3'b111 : // Anyway Jump won't be used in ALU control so not distincting between JAL and JALR
                    3'b000; // Default to R-type

    assign en_branch = ALU_zero && Branch;
    
endmodule