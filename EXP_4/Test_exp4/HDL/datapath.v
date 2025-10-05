module datapath #(
    parameter WIDTH = 32
)(
    input  wire            clk,
    input  wire            reset,
    input  wire            PCSrcW,
    input  wire            BranchTakenE,
    input  wire            StallF,
    input  wire            StallD,
    input  wire            FlushD,
    input  wire  [1:0]     RegSrcD,
    input  wire            WA3_mux,
    input  wire            WD3_mux,
    input  wire  [1:0]     ImmSrcD,
    input  wire            RegWriteW,
    input  wire            FlushE,
    input  wire  [1:0]     ForwardAE,
    input  wire  [1:0]     ForwardBE,
    input  wire            AluSrcE,
    input  wire  [3:0]     AluControlE,
    input  wire  [1:0]     sh_control,
    input  wire  [4:0]     shamt,
    input  wire            MemWriteM,
    input  wire            MemtoRegW,
    input  wire fwdA_BL_M_mux,
    input  wire fwdB_BL_M_mux,
    
   
    output wire [3:0]       ALUFlags,
    output wire [3:0]       RDA1,
    output wire [3:0]       RDA2,


    // --- Debug ports ---------------------------------
    input  wire   [3:0]    debug_input,    // which reg to peek
    output wire  [WIDTH-1:0] debug_output,  // that register's value

    // --- Status outputs ------------------------------
    output wire  [WIDTH-1:0] PC_out,        // current PC
    output wire  [WIDTH-1:0] IR_out,        // current IF
    output wire  [3:0] WA3E_s,
    output wire  [3:0] WA3M_s,
    output wire  [3:0] WA3W_s

);
assign WA3E_s = ID_EX_Rd ;
assign WA3M_s = WA3M;
assign WA3W_s = WA3W ;

    // -------------------------------------------------
    // 1) FETCH STAGE
    // -------------------------------------------------
    wire [WIDTH-1:0] PC_reg, PC_plus4, PC_in;
    wire [WIDTH-1:0] IF_ID_IR, inst_word;
    wire [WIDTH-1:0] PC_inter_word;
    wire [WIDTH-1:0] ResultW;
    wire [WIDTH-1:0] ALUResultE;

    // PC + 4
    Adder #(.WIDTH(WIDTH)) PC_PLUS4 (
        .DATA_A(PC_reg),
        .DATA_B(32'd4),
        .OUT   (PC_plus4)
    );
	
	

    // select PC source 1
    Mux_2to1 #(.WIDTH(WIDTH)) MUX_PCSrc1 (
        .select       (PCSrcW),
        .input_0      (PC_plus4),
        .input_1      (ResultW),
        .output_value(PC_inter_word)
    );
    
    // select PC source 2
    Mux_2to1 #(.WIDTH(WIDTH)) MUX_PCSrc2 (
        .select       (BranchTakenE),
        .input_0      (PC_inter_word),
        .input_1      (ALUResultE),
        .output_value(PC_in)
    );

    // PC register with stall    
    Register_rsten #(.WIDTH(WIDTH)) PC_Reg (
       .clk   (clk),
       .reset (reset),     // global clear
       .we    (~StallF),   // stall when 1→0
       .DATA  (PC_in),
       .OUT   (PC_reg)
    );
    
    assign PC_out = PC_reg;

    // fetch instruction
    Instruction_memory #(.BYTE_SIZE(4), .ADDR_WIDTH(WIDTH)) IM (
        .ADDR (PC_reg),
        .RD   (inst_word)
    );
	
	wire [WIDTH-1:0] d_lr;
	
	Adder #(.WIDTH(WIDTH)) PC_MINUS (
        .DATA_A(PC_reg),
        .DATA_B(32'hFFFFFFF0),
        .OUT   (d_lr)
    );
	
	wire [WIDTH-1:0] bx_2;
	
	Adder #(.WIDTH(WIDTH)) PC_MINUS_2 (
        .DATA_A(PC_reg),
        .DATA_B(32'hFFFFFFF4),
        .OUT   (bx_2)
    );



    // IF/ID.IR register with stall
    Register_rsten #(.WIDTH(WIDTH)) IFID_IR (
        .clk   (clk),
        .reset (reset | FlushD),   // global reset OR flush
        .we    (~StallD),          // write-enable = not stalled
        .DATA  (inst_word),        // incoming instruction bits
        .OUT   (IF_ID_IR)          // pipeline-registered IR
    );

    assign IR_out = IF_ID_IR;
    assign RDA1 = RA1;
    assign RDA2 = RA2;
    
    

    // -------------------------------------------------
    // 2) DECODE STAGE
    // -------------------------------------------------
    
    wire [3:0] RA1, RA2;
    
    // if RegSrcD[0]=0 → IR[19:16], if =1 → R15 (4'd15)
    Mux_2to1 #(.WIDTH(4)) MUX_RA1 (
     .select       (RegSrcD[0]),
     .input_0      (IF_ID_IR[19:16]),
     .input_1      (4'd15),
     .output_value(RA1)
    );

    // if RegSrcD[1]=0 → IR[3:0],  if =1 → IR[15:12]
    Mux_2to1 #(.WIDTH(4)) MUX_RA2 (
     .select       (RegSrcD[1]),
     .input_0      (IF_ID_IR[3:0]),
     .input_1      (IF_ID_IR[15:12]),
     .output_value(RA2)
    );
    
    // --- WA3 select (Rd vs R14) via 2×1 mux ---
    wire [3:0] RF_WA;
    wire [3:0] WA3W;
 
    Mux_2to1 #(.WIDTH(4)) MUX_WA3 (
     .select       (WA3_mux),
     .input_0      (WA3W),  // Rd field
     .input_1      (4'd14),            // R14 for link
     .output_value(RF_WA)
    );

    // --- WD3 select (normal WB vs link PC+4) via 2×1 mux ---
    wire [WIDTH-1:0] RF_WD;
    Mux_2to1 #(.WIDTH(WIDTH)) MUX_WD3 (
     .select       (WD3_mux),
     .input_0      (ResultW),    
     .input_1      (d_lr),// link value
     .output_value(RF_WD)
    );

    // register file
    wire [WIDTH-1:0] RF_RD1, RF_RD2;
    Register_file #(.WIDTH(WIDTH)) RF (
        .clk                (clk),
        .write_enable       (RegWriteW),
        .reset              (reset),
        .Source_select_0    (RA1),
        .Source_select_1    (RA2),
        .Destination_select (RF_WA),
        .DATA               (RF_WD),
        .Reg_15             (PC_plus4),
        .out_0              (RF_RD1),
        .out_1              (RF_RD2),
        .Debug_Source_select(debug_input),
        .Debug_out          (debug_output)
    );

    // immediate extender
    wire [WIDTH-1:0] Imm_ext;
    Extender EXT (
        .DATA          (IF_ID_IR[23:0]),
        .select        (ImmSrcD),
        .Extended_data (Imm_ext)
    );

    // ID/EX pipeline outputs
    wire [WIDTH-1:0] ID_EX_A;
    wire [WIDTH-1:0] ID_EX_B;
    wire [WIDTH-1:0] ID_EX_Imm;
    wire  [3:0]      ID_EX_Rd;
	wire      combin_res = reset | FlushE;

    // ID/EX pipeline registers with synchronous clear on FlushE or global reset
    Register_reset #(.WIDTH(WIDTH)) IDEX_A (
       .clk   (clk),
       .reset (combin_res),  // clear on global reset or pipeline flush
       .DATA  (RF_RD1),
       .OUT   (ID_EX_A)
    );

    Register_reset #(.WIDTH(WIDTH)) IDEX_B (
      .clk   (clk),
      .reset (combin_res),
      .DATA  (RF_RD2),
      .OUT   (ID_EX_B)
    );

    Register_reset #(.WIDTH(WIDTH)) IDEX_Imm (
     .clk   (clk),
     .reset (combin_res),
     .DATA  (Imm_ext),
     .OUT   (ID_EX_Imm)
    );

    Register_reset #(.WIDTH(4)) IDEX_Rd (
     .clk   (clk),
     .reset (combin_res),
     .DATA  (IF_ID_IR[15:12]),
     .OUT   (ID_EX_Rd)
    );


    // -------------------------------------------------
    // 3) EXECUTE STAGE (4×1 forwarding muxes)
    // -------------------------------------------------
    
    Mux_2to1 #(.WIDTH(WIDTH)) MUX_forwardA (
        .select       (fwdA_BL_M_mux),
        .input_0      (d_lr),
        .input_1      (bx_2),
        .output_value(FwdA_3)
    );
    
    Mux_2to1 #(.WIDTH(WIDTH)) MUX_forwardB (
        .select       (fwdB_BL_M_mux),
        .input_0      (d_lr),
        .input_1      (bx_2),
        .output_value(FwdB_3)
    );
    
    
    
    wire [WIDTH-1:0] ALUOutM;

    // Prepare the four possible sources for operand A:
    wire [WIDTH-1:0] FwdA_0 = ID_EX_A;      
    wire [WIDTH-1:0] FwdA_1 = ResultW;  
    wire [WIDTH-1:0] FwdA_2 = ALUOutM;        
    wire [WIDTH-1:0] FwdA_3 ;         

    // 4-to-1 MUX for ALU's A input
    wire [WIDTH-1:0] EXA;
    Mux_4to1 #(.WIDTH(WIDTH)) MUX_ForwardAE (
     .select       (ForwardAE),
     .input_0      (FwdA_0),
     .input_1      (FwdA_1),
     .input_2      (FwdA_2),
     .input_3      (FwdA_3),
     .output_value(EXA)
    );

    // Prepare the four possible sources for operand B (pre-shift):
    wire [WIDTH-1:0] FwdB_0 = ID_EX_B;
    wire [WIDTH-1:0] FwdB_1 = ResultW;
    wire [WIDTH-1:0] FwdB_2 = ALUOutM;
    wire [WIDTH-1:0] FwdB_3;

    // 4-to-1 MUX for B into the shifter
    wire [WIDTH-1:0] EXB0;
    Mux_4to1 #(.WIDTH(WIDTH)) MUX_ForwardBE (
    .select       (ForwardBE),
    .input_0      (FwdB_0),
    .input_1      (FwdB_1),
    .input_2      (FwdB_2),
    .input_3      (FwdB_3),
    .output_value(EXB0)
    );
    
    Mux_2to1 #(.WIDTH(WIDTH)) MUX_ALUSrc (
     .select       (AluSrcE),
     .input_0      (EXB0),  
     .input_1      (ID_EX_Imm),      
     .output_value(SH_in)
    );

    wire [WIDTH-1:0] SH_out;
    wire [WIDTH-1:0] SH_in;
    shifter #(.WIDTH(WIDTH)) SHIFT_UNIT (
    .control (sh_control),
    .shamt   (shamt),
    .DATA    (SH_in),
    .OUT     (SH_out)
    );

    ALU #(.WIDTH(WIDTH)) ALU_UNIT (
    .control (AluControlE),
    .CI      (1'b0),
    .DATA_A  (EXA),
    .DATA_B  (SH_out),
    .OUT     (ALUResultE),
    .CO      (ALUFlags[1]),
    .OVF     (ALUFlags[0]),
    .N       (ALUFlags[3]),
    .Z       (ALUFlags[2])
    );

    // -------------------------------------------------
    // EX/MEM PIPELINE REGISTERS (clear on reset)
    // -------------------------------------------------
    // ALU result:
    Register_reset #(.WIDTH(WIDTH)) EXMEM_ALUOut (
     .clk   (clk),
     .reset (reset),
     .DATA  (ALUResultE),
     .OUT   (ALUOutM)
    );
    wire [WIDTH-1:0] WD;
    wire [3:0] WA3M;
    // B operand (for store data):
    Register_reset #(.WIDTH(WIDTH)) EXMEM_B (
     .clk   (clk),
     .reset (reset),
     .DATA  (EXB0),
     .OUT   (WD)
    );

    // Destination register number:
    Register_reset #(.WIDTH(4)) EXMEM_Rd (
     .clk   (clk),
     .reset (reset),
     .DATA  (ID_EX_Rd),
     .OUT   (WA3M)
    );

    // -------------------------------------------------
    // 4) MEMORY STAGE
    // -------------------------------------------------
    wire [WIDTH-1:0] MEM_RD;
    Memory #(.BYTE_SIZE(4), .ADDR_WIDTH(WIDTH)) DM (
        .clk  (clk),
        .WE   (MemWriteM),
        .ADDR (ALUOutM),
        .WD   (WD),
        .RD   (MEM_RD)
    );

    // EX/MEM → MEM/WB registers
    
    wire [WIDTH-1:0] ALUOutW;
    wire [WIDTH-1:0] ReadDataW;
    
    Register_simple #(.WIDTH(WIDTH)) MEMWB_ALUOut (
        .clk   (clk),
        .DATA  (ALUOutM),
        .OUT   (ALUOutW)
    );
    Register_simple #(.WIDTH(WIDTH)) MEMWB_MDR (
        .clk   (clk),
        .DATA  (MEM_RD),
        .OUT   (ReadDataW)
    );
    Register_simple #(.WIDTH(4)) MEMWB_Rd (
        .clk   (clk),
        .DATA  (WA3M),
        .OUT   (WA3W)
    );

    // -------------------------------------------------
    // 5) WRITEBACK STAGE
    // -------------------------------------------------
    // select ALUOut vs MDR
    Mux_2to1 #(.WIDTH(WIDTH)) MUX_MemToReg (
        .select        (MemtoRegW),
        .input_0       (ALUOutW),
        .input_1       (ReadDataW),
        .output_value (ResultW)
    );


endmodule