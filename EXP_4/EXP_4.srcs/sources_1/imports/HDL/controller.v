module controller (
    input  wire        clk,
    input  wire        reset,
    input wire FlushE,

    // From IF/ID
    input  wire [31:0] IR_D,
	input  wire  FlushD,

    // From ALU in EX/MEM 
    input  wire [3:0]  FlagsE,    

    // Control outputs to datapath
    output wire        PCSrcW,        
    output wire        BranchTakenE,  

    output wire [1:0]  RegSrcD,
    output wire        WA3_mux,
    output wire        WD3_mux,
    output wire [1:0]  ImmSrcD,
    output wire        RegWriteW,
    output wire        RegWriteMs,
    output wire        MemtoRegEs,

    output wire        AluSrcE,     
    output wire [3:0]  ALUControlE,
    output wire [1:0]  sh_controlE,
    output wire [4:0]  shamtE,

    output wire        MemWriteM,
    output wire        MemtoRegW,
    output wire        PCSrcD_s,
    output wire        PCSrcE_s,
    output wire        PCSrcM_s,
    output wire        mux_M,
    output wire        mux_W,
	output wire        isBL_
     
);
	assign isBL_ =isBL;
    assign        mux_M = WA3_M ;
    assign        mux_W = WA3_W ;
    assign        PCSrcD_s = PCSrcD ;
    assign        PCSrcE_s = PCSrcE ;
    assign        PCSrcM_s = PCSrcM_r ;
    assign        RegWriteMs = RegWriteM_r ;
    assign        MemtoRegEs = MemtoRegE;

    //========================================================================
    // 1) DECODE-STAGE 
    //========================================================================
    // Slice fields from IR_D
    wire [3:0]  condD   = IR_D[31:28];
    wire [1:0]  opD     = IR_D[27:26];
    wire        iD      = IR_D[25];
    wire [3:0]  opcodeD = IR_D[24:21];
    wire [3:0]  RdD     = IR_D[15:12];
    

    wire isBX = ((IR_D & 32'h0FFFFFF0) == 32'h012FFF10);
	wire isBL = (opD == 2'b10) && (IR_D[24] == 1'b1);
	
    // decode outputs (D-stage)
    reg  [1:0] RegSrcD_r;
    reg        WA3_D_r, WD3_D_r, RegWriteD, MemWriteD, MemtoRegD;
    reg        AluSrcD;
    reg  [1:0] ImmSrcD_r;
    reg  [1:0] sh_controlD;
    reg  [4:0] shamtD;
    reg  [3:0] ALUControlD;
    reg        BranchD;
    reg        PCSrcD;
   

    always @(*) begin
        // defaults
        RegSrcD_r    = 2'b00;
        WA3_D_r      = 1'b0;
        WD3_D_r      = 1'b0;
        RegWriteD    = 1'b0;
        MemWriteD    = 1'b0;
        MemtoRegD    = 1'b0;
        AluSrcD      = 1'b0;
        ImmSrcD_r    = 2'b00;
        sh_controlD  = 2'b00;
        shamtD       = 5'b0;
        ALUControlD  = 4'b0000;
        BranchD      = 1'b0;
        PCSrcD       = 1'b0;

        case (opD)
            // -----------------------------
            // Data-Processing (ADD, SUB, AND, ORR, CMP, MOV,BX)
            // -----------------------------
            2'b00: begin
                 if (isBX) begin
                 // -------------------------------
                 // BX Rm  →  PC ← Rm (no reg write)
                 // -------------------------------
                 RegWriteD   = 1'b0 ;
                 MemWriteD   = 1'b0;
                 MemtoRegD   = 1'b0;
                 BranchD     = 1'b1;       // will cause PC update in EX stage

                 RegSrcD_r   = 2'b00;      // select Rm as the data source
                 WA3_D_r     = 1'b0;       // no destination write
                 WD3_D_r     = 1'b0;

                 AluSrcD     = 1'b0;       // ALU A = Rn (don't care)
           
                 ImmSrcD_r   = 2'b00;
                 sh_controlD = 2'b00;      // no shift
                 shamtD      = 5'b0;
                 PCSrcD      = 1'b0;

                 ALUControlD = 4'b1101;    // MOV operation: OUT = B → gives Rm straight through
                end
                else begin
                 // 2) Any DP writing to R15 → PC ← DP result
                 if (RdD == 4'hF) begin
                   RegWriteD   = 1'b0;    
                   WA3_D_r     = 1'b0;   
                   WD3_D_r     = 1'b0;  
                   BranchD     = 1'b0;
				   PCSrcD       = 1'b1;
				   
                 end
                 // 3) All other DP instructions
                 else begin
                   RegWriteD   = (opcodeD != 4'b1010) & (~isBX) & (~flushd_2); // CMP doesn't write
                   WA3_D_r     = 1'b0;    // destination = Rd
                   WD3_D_r     = 1'b0;
                   BranchD     = 1'b0;
				   PCSrcD       = 1'b0;
                 end
            
                 // common DP fields:
                 
                 MemWriteD   = 1'b0;
                 MemtoRegD   = 1'b0;
                 RegSrcD_r   = 2'b00;      // Rn & Rm
                 AluSrcD     = iD;      
                 ImmSrcD_r   = 2'b00;   
                 sh_controlD = iD 
                         ? 2'b11                       // I=1 → rotate-imm (RR)
                         : IR_D[6:5];                  // I=0 → register-shift type

                 shamtD      = iD 
                         ? {IR_D[11:8], 1'b0}          // I=1 → rotate amount = imm8.rot <<1
                         : IR_D[11:7];                 // I=0 → register shift amount

                 ALUControlD = (opcodeD == 4'b1010) 
                         ? 4'b0010                     // CMP → subtraction
                         : opcodeD;                    // else direct DP opcode
                     end
                 end

            // -----------------------------
            // Load/Store
            // -----------------------------
            2'b01: begin
				// common LDR/STR fields
				BranchD     = 1'b0;  
				AluSrcD     = 1'b1;       // use Rn
				ImmSrcD_r   = 2'b01;    // 12‑bit immediate
				sh_controlD = 2'b00;
				shamtD      = 5'b0;
			
				if (IR_D[20] == 1'b0) begin
					// STR Rd,[Rn,#imm]
					RegWriteD = 1'b0;
					MemWriteD = 1'b1;
					MemtoRegD = 1'b0;
					WA3_D_r   = 1'b0;
					RegSrcD_r   = 2'b10;
					ALUControlD = 4'b0100; 
					WD3_D_r   = 1'b0;
					PCSrcD       = 1'b0;
				end else begin
					// LDR Rd,[Rn,#imm]
					MemWriteD = 1'b0;
					MemtoRegD = 1'b1;
					ALUControlD = 4'b0100; 
					RegSrcD_r   = 2'b00;
					WA3_D_r = 1'b0;   
					WD3_D_r = 1'b0;   
			
					if (RdD == 4'hF) begin
						// LDR PC, [...]  → PC ← memory data
						RegWriteD = 1'b0;
						PCSrcD       = 1'b1;
					end else begin
						// normal register load
						RegWriteD = 1'b1;
						PCSrcD       = 1'b0;
					end
				end
			end

            // -----------------------------
            // Branch / Branch-with-Link
            // -----------------------------
            2'b10: begin
                RegWriteD = IR_D[24]; 
                MemWriteD = 1'b0;
                MemtoRegD = 1'b0;
                BranchD   = 1'b1;
                RegSrcD_r = 2'b01;              // R15 if needed
                WA3_D_r   = IR_D[24];           // link? R14
                WD3_D_r   = IR_D[24];           // PC+4
                AluSrcD   = 1'b1;
                ImmSrcD_r = 2'b10;              // 24-bit
                sh_controlD=2'b00;
                shamtD     =5'b0;
                ALUControlD=4'b0100;            // ADD PC+imm
                PCSrcD       = 1'b0;
            end
        

            default: ;
        endcase
    end
    
    wire isCMP_D = (opD == 2'b00) && (opcodeD == 4'b1010);
    
    wire [3:0] CPSR;
    Register_rsten #(.WIDTH(4)) CPSR_reg (
        .clk   (clk),
        .reset (reset),
        .we    (isCMP_E),   // only write when this was a CMP in EX
        .DATA  (FlagsE),    // ALU's new NZCV
        .OUT   (CPSR)
     );
    
	wire [3:0] condE;
	
	Register_reset #(.WIDTH(4)) Cond_E (
         .clk   (clk),
         .reset (reset_combined),
         .DATA  (condD),
         .OUT   (condE)
        ); 
		
    // simple condition-check (AL, EQ, NE only)
    wire cond_pass = (condE == 4'hE)   ? 1'b1       :  // AL
                 (condE == 4'h0)   ? CPSR[2]    :  // EQ
                 (condE == 4'h1)   ? ~CPSR[2]   :  // NE
                                     1'b0;      // other conds unused
									 
                                     
    //========================================================================
    // 2) PIPELINE REGISTERS FOR CONTROL SIGNALS
    //    ID→EX, EX→MEM, MEM→WB via modules
    //========================================================================
    

    // EX stage
    wire        RegWriteE, MemWriteE, MemtoRegE, WA3_E, WD3_E, BranchE, PCSrcE;
    wire        AluSrcE_ID ;
    wire [1:0]  sh_controlE_ID;
    wire [3:0]  ALUControlE_ID;
    wire [4:0]  shamtE_ID;
    wire isCMP_E; 
    wire reset_combined = reset | FlushE ;
	wire flushd_2;



    Register_reset #(.WIDTH(1)) CMP_D2E (
         .clk   (clk),
         .reset (reset_combined),
         .DATA  (isCMP_D),
         .OUT   (isCMP_E)
        ); 
	
	
	Register_reset #(.WIDTH(1)) bize_allahtan_baska_kimsenin_gucu_yetmez  (clk, reset, FlushD, flushd_2);
	
    Register_reset #(.WIDTH(1)) REG_REGW_E  (clk, reset_combined, RegWriteD,  RegWriteE);
    Register_reset #(.WIDTH(1)) REG_MEMW_E  (clk, reset_combined, MemWriteD,  MemWriteE);
    Register_reset #(.WIDTH(1)) REG_MEMto_E (clk, reset_combined, MemtoRegD,  MemtoRegE);
    Register_reset #(.WIDTH(1)) REG_WA3_E   (clk, reset_combined, WA3_D_r,      WA3_E);
    Register_reset #(.WIDTH(1)) REG_WD3_E   (clk, reset_combined, WD3_D_r,      WD3_E);
    Register_reset #(.WIDTH(1)) REG_ASRC_E  (clk, reset_combined, AluSrcD,    AluSrcE_ID);
    Register_reset #(.WIDTH(2)) REG_SHC_E   (clk, reset_combined, sh_controlD,sh_controlE_ID);
    Register_reset #(.WIDTH(5)) REG_SHAMT_E (clk, reset_combined, shamtD,     shamtE_ID);
    Register_reset #(.WIDTH(4)) REG_ALUCTL_E(clk, reset_combined, ALUControlD, ALUControlE_ID);
    Register_reset #(.WIDTH(1)) REG_PCSrc_E(clk, reset_combined, PCSrcD, PCSrcE);
	Register_reset #(.WIDTH(1)) REG_BR_E    (clk, reset_combined, BranchD,    BranchE);
    
    wire and_1 = PCSrcE & cond_pass;
    wire and_2 = BranchE & cond_pass;
    wire and_3 = RegWriteE & cond_pass;
    wire and_4 = MemWriteE & cond_pass;

    // MEM stage
    wire RegWriteM_r, MemWriteM_r, MemtoRegM_r,PCSrcM_r , WA3_M , WD3_M;
    Register_reset #(.WIDTH(1)) REG_REGW_M(clk, reset, and_3,  RegWriteM_r);
    Register_reset #(.WIDTH(1)) REG_MEMW_M(clk, reset, and_4,  MemWriteM_r);
    Register_reset #(.WIDTH(1)) REG_MEMto_M(clk, reset, MemtoRegE,  MemtoRegM_r);
    Register_reset #(.WIDTH(1)) REG_PCSrc_M(clk, reset, and_1,  PCSrcM_r);
    Register_reset #(.WIDTH(1)) REG_WA3_M   (clk, reset, WA3_E,      WA3_M);
    Register_reset #(.WIDTH(1)) REG_WD3_M   (clk, reset, WD3_E,      WD3_M);

    // WB stage
    wire RegWriteW_r, MemtoRegW_r,PCSrcW_r ,WA3_W , WD3_W;
    Register_reset #(.WIDTH(1)) REG_REGW_W(clk, reset, RegWriteM_r, RegWriteW_r);
    Register_reset #(.WIDTH(1)) REG_MEMto_W(clk, reset, MemtoRegM_r, MemtoRegW_r);
    Register_reset #(.WIDTH(1)) REG_PCSrc_W(clk, reset, PCSrcM_r,  PCSrcW_r);
    Register_reset #(.WIDTH(1)) REG_WA3_W   (clk, reset, WA3_M,      WA3_W);
    Register_reset #(.WIDTH(1)) REG_WD3_W   (clk, reset, WD3_M,      WD3_W);

    //========================================================================
    // 3) FINAL OUTPUTS
    //========================================================================

    assign RegSrcD   = RegSrcD_r;
    assign WA3_mux   = WA3_W;
    assign WD3_mux   = WD3_W;
    assign ImmSrcD   = ImmSrcD_r;

    // Execute-stage outputs
    assign AluSrcE       = AluSrcE_ID;
    assign ALUControlE   = ALUControlE_ID;
    assign sh_controlE   = sh_controlE_ID;
    assign shamtE        = shamtE_ID;
    assign BranchTakenE  = and_2;

    // Memory-stage output
    assign MemWriteM     = MemWriteM_r  ;
    
    // Writeback-stage outputs
    assign RegWriteW     = RegWriteW_r ;
    assign MemtoRegW     = MemtoRegW_r ;


    assign PCSrcW        = PCSrcW_r ;

endmodule