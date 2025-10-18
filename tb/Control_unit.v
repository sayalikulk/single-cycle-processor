module Control_unit(
    input wire [6:0] opcode,  //7-bit opcode will come from instruction[6:0]
        
          
    output wire mem_to_reg,
    output wire reg_write,
    output wire mem_read,
    output wire mem_write,
    output wire Branch,
    output wire Jump,
    output wire Jalr, //Jump and Link
    output wire load_upper_imm,
    output wire upper_imm,
    output wire alu_src,
    output wire [1:0]ALU_OP
    // output wire Add_upper_imm,
    
);
    wire Add_upper_imm;
    wire R_type;
    wire I_type;  
    wire Imm_Load;
    wire Arith_I_type;
    wire Store;

    /*Intermediate signals for instruction types*/
    assign R_type = (opcode == 7'b0110011) ? 1'b1 : 1'b0; //R-type instruction signal
    assign I_type = (Imm_load | Arith_I_type | Jalr); // I-type has three different instructions
    assign Imm_Load = (opcode == 7'b0000011) ? 1'b1 : 1'b0; //I-type instruction signal
    assign Arith_I_type = (opcode == 7'b0010011) ? 1'b1 : 1'b0; //Load instruction signal
    assign Store = (opcode == 7'b0100011) ? 1'b1 : 1'b0; //Store instruction signal
    assign Branch = (opcode == 7'b1100011) ? 1'b1 : 1'b0; //Branch instruction signal
    //assign upper_imm = (opcode == 7'b0010111) ? 1'b1 : 1'b0; //AUIPC instruction signal is high or it will take immediate value only 

    /*Control signal assignments based on instruction types*/
    assign Jal = (opcode == 7'b1101111) ? 1'b1 : 1'b0; //Jump signal for JAL instruction
    assign upper_imm = Add_upper_imm | load_upper_imm; //To assert U type instruction signal
    assign load_upper_imm = (opcode == 7'b0110111) ? 1'b1 : 1'b0; //LUI instruction signal 
    assign Add_upper_imm = (opcode == 7'b0010111) ? 1'b1 : 1'b0; //AUIPC instruction signal
    assign Jalr = (opcode == 7'b1100111) ? 1'b1 : 1'b0; //JALR instruction signal
    assign alu_src = (I_type || load || store || Jalr || load_upper_imm || Add_upper_imm) ? 1'b1 : 1'b0; //ALU source selection
    assign mem_to_reg = (load) ? 1'b1 : 1'b0; //Memory to register selection
    assign reg_write = (R_type || I_type || load || load_upper_imm || Add_upper_imm || Jump) ? 1'b1 : 1'b0; //Register write enable
    assign mem_read = (load) ? 1'b1 : 1'b0; //Memory read enable
    assign mem_write = (store) ? 1'b1 : 1'b0; //Memory write enable
    assign ALU_OP = (R_type) ? 2'b00:
                    (I_type) ? 2'b01:
                    (Store)  ? 2'b10:
                    (Branch) ? 2'b11;
endmodule