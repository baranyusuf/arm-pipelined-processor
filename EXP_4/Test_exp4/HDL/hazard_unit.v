module hazard_unit (
    // Decode-stage register numbers
    input  wire [3:0] RA1D, RA2D,
    // Execute-stage destination and control
    input  wire [3:0] RA1E, RA2E,
    input  wire [3:0] WA3E,
    input  wire       MemtoRegE,
	input  wire       isBL,
    // Memory-stage destination and control
    input  wire [3:0] WA3M,
    input  wire       RegWriteM,
    input  wire       RegWriteW,
    input  wire       w_mux_M,
    input  wire       w_mux_W,
    // Writeback-stage destination and control
    input  wire [3:0] WA3W,
    // PC write-pending signals from D, E, M
    input  wire       PCSrcD,
    input  wire       PCSrcE,
    input  wire       PCSrcM,
    // Writeback to PC at WB
    input  wire       PCSrcW,
    // Branch-taken from EX
    input  wire       BranchTakenE,

    // Outputs
    output wire       StallF,
    output wire       StallD,
    output wire       FlushD,
    output wire       FlushE,
    output wire [1:0] ForwardAE,
    output wire [1:0] ForwardBE,
    output wire fwdA_BL_M_mux,
    output wire fwdB_BL_M_mux
);

    //─────────────────────────────────────────────────────────────────────────
    // 1) Load-use stall (one-cycle bubble)
    //    Match_12D_E = (RA1D==WA3E) or (RA2D==WA3E)
    //    LDRstall    = Match_12D_E · MemtoRegE
    //─────────────────────────────────────────────────────────────────────────
    wire match12 = (RA1D == WA3E) | (RA2D == WA3E);
    wire LDRstall = match12 & MemtoRegE;
		
	wire BL1,BL2,BL3;	
	Register_reset #(.WIDTH(1)) BL1_R (clk, reset_combined, isBL,  BL1);
	Register_reset #(.WIDTH(1)) BL2_R  (clk, reset, BL1,  BL2);	
	Register_reset #(.WIDTH(1)) BL3_R  (clk, reset, BL2,  BL3);
    //─────────────────────────────────────────────────────────────────────────
    // 2) Control-hazard stall: any pending PC-write in D/E/M
    //    PCWrPendingF = PCSrcD + PCSrcE + PCSrcM
    //─────────────────────────────────────────────────────────────────────────
    wire PCWrPendingF = PCSrcD | PCSrcE | PCSrcM;

    //─────────────────────────────────────────────────────────────────────────
    // 3) Stall Fetch & Decode, Flush D/E
    //    StallF = (LDRstall + PCWrPendingF) not asserted during PCSrcW
    //    StallD = LDRstall
    //    FlushD = PCWrPendingF + PCSrcW + BranchTakenE
    //    FlushE = LDRstall + BranchTakenE
    //─────────────────────────────────────────────────────────────────────────
    assign StallF  = (LDRstall | PCWrPendingF );
    assign StallD  = LDRstall;
    assign FlushD  = PCWrPendingF | PCSrcW | BranchTakenE;
    assign FlushE  = LDRstall | BranchTakenE;

    //─────────────────────────────────────────────────────────────────────────
    // 4) Forwarding (4→1 selects) for EX-stage operands
    //    Match_1E_M = (RA1E==WA3M)
    //    Match_1E_W = (RA1E==WA3W)
    //    if match&M then 10
    //    else if match&W then 01
    //    else 00
    //
    //    same for RA2E→ForwardBE
    //─────────────────────────────────────────────────────────────────────────
    wire fwdA_M    = (RA1E == WA3M) & RegWriteM & (~BL3) & (~BL2) ;
    wire fwdA_W    =  (RA1E == WA3W) & RegWriteW & (~BL3) & (~BL2);
    wire fwdA_BL_M   = (w_mux_M == 1'b1) & RegWriteM & (RA1E == 4'd14);  
    wire fwdA_BL_W   = (w_mux_W == 1'b1) & RegWriteW & (RA1E == 4'd14); 
    wire fwdA_BL =  fwdA_BL_M | fwdA_BL_W;

    assign ForwardAE = 
        fwdA_M  ? 2'b10 :   // from ALUOutM
        fwdA_W  ? 2'b01 :   // from ResultW
        fwdA_BL ? 2'b11 :   // from PCplus4E
                  2'b00 ;   // from RF

    // same for the B side:
    wire fwdB_M    = (RA2E == WA3M) & RegWriteM & (~BL3) & (~BL2) ;
    wire fwdB_W    = (RA2E == WA3W) & RegWriteW & (~BL3) & (~BL2);
    wire fwdB_BL_M   = (w_mux_M == 1'b1) & RegWriteM & (RA2E == 4'd14);  
    wire fwdB_BL_W   = (w_mux_W == 1'b1) & RegWriteW & (RA2E == 4'd14); 
    wire fwdB_BL =  fwdB_BL_M | fwdB_BL_W;

    assign ForwardBE =
       fwdB_M  ? 2'b10 :
       fwdB_W  ? 2'b01 :
       fwdB_BL ? 2'b11 :
                 2'b00 ;
                 
     assign fwdA_BL_M_mux =  fwdA_BL_M;
     assign fwdB_BL_M_mux =  fwdB_BL_M;

endmodule
