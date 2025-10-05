module top (
  input  wire        clk,
  input  wire        reset,
  input  wire [3:0]  debug_input,
  output wire [31:0] debug_output,
  output wire [31:0] PC_out
);

  // ──────────────────────────────────────────────────────────────────────────
  // Wires for controller ↔ datapath
  // ──────────────────────────────────────────────────────────────────────────
  wire        PCSrcW, BranchTakenE;
  wire [1:0]  RegSrcD;
  wire        WA3_mux, WD3_mux;
  wire [1:0]  ImmSrcD;
  wire        AluSrcE;
  wire [3:0]  ALUControlE;
  wire [1:0]  sh_controlE;
  wire [4:0]  shamtE;
  wire        MemWriteM, MemtoRegW;
  wire [31:0] IR;
  wire [3:0] Flag;

  // ──────────────────────────────────────────────────────────────────────────
  // Wires for hazard_unit ↔ datapath
  // ──────────────────────────────────────────────────────────────────────────
  // decode-stage reg numbers
  wire [3:0] RA1D, RA2D;
  // EX-stage, pipelined
  wire [3:0] RA1E, RA2E, WA3E;
  wire        MemtoRegE;
  // MEM-stage, pipelined
  wire [3:0] WA3M;
  wire        RegWriteM;
  // WB-stage, pipelined
  wire [3:0] WA3W;
  wire        RegWriteW;
  // PC-writes at each stage
  wire        PCSrcD, PCSrcE, PCSrcM ;
  wire w_mux_M;
  wire       w_mux_W;

  // ──────────────────────────────────────────────────────────────────────────
  // Hazard outputs
  // ──────────────────────────────────────────────────────────────────────────
  wire        StallF, StallD, FlushD, FlushE;
  wire [1:0]  ForwardAE, ForwardBE;
  // extra mux selects for BL→PC+4 forwarding
  wire        fwdA_BL_M_mux, fwdB_BL_M_mux;
  wire isBL;
  // ──────────────────────────────────────────────────────────────────────────
  // 1) Instantiate your controller
  // ──────────────────────────────────────────────────────────────────────────
  controller ctrl (
	.isBL_(isBL),
	.FlushD(FlushD),
    .clk          (clk),
    .reset        (reset),
    .FlushE       (FlushE),
    .RegWriteMs(RegWriteM),
    .mux_M(w_mux_M),
    .mux_W(w_mux_W),
    .MemtoRegEs(MemtoRegE),

    .IR_D         (IR),
    .FlagsE       (Flag),

    .PCSrcW       (PCSrcW),
    .PCSrcD_s       (PCSrcD),
    .PCSrcE_s       (PCSrcE),
    .PCSrcM_s       (PCSrcM),
    .BranchTakenE (BranchTakenE),

    .RegSrcD      (RegSrcD),
    .WA3_mux      (WA3_mux),
    .WD3_mux      (WD3_mux),
    .ImmSrcD      (ImmSrcD),
    .RegWriteW    (RegWriteW),

    .AluSrcE      (AluSrcE),
    .ALUControlE  (ALUControlE),
    .sh_controlE  (sh_controlE),
    .shamtE       (shamtE),

    .MemWriteM    (MemWriteM),
    .MemtoRegW    (MemtoRegW)
  );

  // ──────────────────────────────────────────────────────────────────────────
  // 2) Instantiate your hazard unit
  // ──────────────────────────────────────────────────────────────────────────
  hazard_unit haz (
	.isBL(isBL),
    .RA1D            (RA1D),        .RA2D       (RA2D),
    .RA1E            (RA1E),        .RA2E       (RA2E),
    .WA3E            (WA3E),        .MemtoRegE  (MemtoRegE),
    .WA3M            (WA3M),        .RegWriteM  (RegWriteM),
    .WA3W            (WA3W),        .RegWriteW  (RegWriteW),
    .PCSrcD          (PCSrcD),
    .PCSrcE          (PCSrcE),
    .PCSrcM          (PCSrcM),
    .PCSrcW          (PCSrcW),
    .BranchTakenE    (BranchTakenE),

    .StallF          (StallF),
    .StallD          (StallD),
    .FlushD          (FlushD),
    .FlushE          (FlushE),
    .ForwardAE       (ForwardAE),
    .ForwardBE       (ForwardBE),
    .fwdA_BL_M_mux   (fwdA_BL_M_mux),
    .fwdB_BL_M_mux   (fwdB_BL_M_mux),
    .w_mux_M(w_mux_M), 
    .w_mux_W(w_mux_W)
  );

  // ──────────────────────────────────────────────────────────────────────────
  // 3) Instantiate your datapath
  // ──────────────────────────────────────────────────────────────────────────
  datapath dp (
    .clk             (clk),
    .reset           (reset),
    .RDA1(RA1D),
    .RDA2(RA2D),
    .WA3E_s(WA3E),
    .WA3M_s(WA3M),
    .WA3W_s(WA3W),
    

    .PCSrcW          (PCSrcW),
    .BranchTakenE    (BranchTakenE),

    .StallF          (StallF),
    .StallD          (StallD),
    .FlushD          (FlushD),

    // pass RegSrcD, WA3_mux, WD3_mux, ImmSrcD from ctrl
    .RegSrcD         (RegSrcD),
    .WA3_mux         (WA3_mux),
    .WD3_mux         (WD3_mux),
    .ImmSrcD         (ImmSrcD),

    .RegWriteW       (RegWriteW),
    .FlushE          (FlushE),

    .ForwardAE       (ForwardAE),
    .ForwardBE       (ForwardBE),
    .fwdA_BL_M_mux   (fwdA_BL_M_mux),
    .fwdB_BL_M_mux   (fwdB_BL_M_mux),

    .AluSrcE         (AluSrcE),
    .AluControlE     (ALUControlE),
    .sh_control      (sh_controlE),
    .shamt           (shamtE),

    .MemWriteM       (MemWriteM),
    .MemtoRegW       (MemtoRegW),

    .debug_input     (debug_input),
    .debug_output    (debug_output),

    .PC_out          (PC_out),
    .IR_out          (IR),
    .ALUFlags        (Flag)
  );

  wire combined_res = reset & FlushE;
  Register_reset #(.WIDTH(4)) RDE1(.clk(clk), .reset(combined_res), .DATA(RA1D), .OUT(RA1E));
  Register_reset #(.WIDTH(4)) RDE2(.clk(clk), .reset(combined_res), .DATA(RA2D), .OUT(RA2E));

endmodule
