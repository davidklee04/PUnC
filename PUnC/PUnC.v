//==============================================================================
// Module for PUnC LC3 Processor
//==============================================================================

module PUnC(
	// External Inputs
	input  wire        clk,            // Clock
	input  wire        rst,            // Reset

	// Debug Signals
	input  wire [15:0] mem_debug_addr,
	input  wire [2:0]  rf_debug_addr,
	output wire [15:0] mem_debug_data,
	output wire [15:0] rf_debug_data,
	output wire [15:0] pc_debug_data
);

	//----------------------------------------------------------------------
	// Interconnect Wires
	//----------------------------------------------------------------------
    
    wire  [1:0] MUX_input;
    wire  [1:0] PCMUX; //
    wire  pcld;
    wire  pcclr;
    wire  [1:0] MUX;
    wire  MUX_w_addr;
    wire  mem_en;  //w_en memory
    wire  irld;
    wire  [2:0] reg_w_addr;
    wire  reg_w_en;
    wire  [2:0] regIn1;
    wire  [2:0] regIn2;
    wire  cond_chk;
    wire  reg_chk1;
    wire  reg_chk2;
    wire  [3:0] ALU_Cur;
    wire  cond_en;
    wire  [15:0] ir;
    wire  JMP_In;
    wire  N;
    wire  Z;
    wire  P;
    wire  signed [15:0] regIn3;
    
//----------------------------------------------------------------------
// Control Module
//----------------------------------------------------------------------
PUnCControl ctrl(
    .clk             (clk),
    .rst             (rst),
    .ir              (ir),
    .MUX_input       (MUX_input),
    .PCMUX           (PCMUX),
    .pcld            (pcld),
    .pcclr           (pcclr),
    .MUX             (MUX),
    .MUX_w_addr      (MUX_w_addr),
    .mem_en          (mem_en),
    .irld            (irld),
    .reg_w_addr      (reg_w_addr),
    .reg_w_en        (reg_w_en),
    .regIn1          (regIn1),
    .regIn2          (regIn2),
    .regIn3          (regIn3),
    .cond_chk        (cond_chk),
    .reg_chk1        (reg_chk1),
    .reg_chk2        (reg_chk2),
    .ALU_Cur         (ALU_Cur),
    .cond_en         (cond_en),
    .JMP_In          (JMP_In),
    .N               (N),
    .Z               (Z),
    .P               (P)
);

//----------------------------------------------------------------------
// Datapath Module
//----------------------------------------------------------------------
PUnCDatapath dpath(
    .clk             (clk),
    .rst             (rst),

    .mem_debug_addr  (mem_debug_addr),
    .rf_debug_addr   (rf_debug_addr),
    .mem_debug_data  (mem_debug_data),
    .rf_debug_data   (rf_debug_data),
    .pc_debug_data   (pc_debug_data),

    .MUX_input       (MUX_input),
    .PCMUX           (PCMUX),
    .pcld            (pcld),
    .pcclr           (pcclr),
    .MUX             (MUX),
    .MUX_w_addr      (MUX_w_addr),
    .mem_en          (mem_en),
    .irld            (irld),
    .ir              (ir),
    .reg_w_addr      (reg_w_addr),
    .reg_w_en        (reg_w_en),
    .regIn1          (regIn1),
    .regIn2          (regIn2),
    .regIn3          (regIn3),
    .cond_chk        (cond_chk),
    .reg_chk1        (reg_chk1),
    .reg_chk2        (reg_chk2),
    .ALU_Cur         (ALU_Cur),
    .cond_en         (cond_en),
    .JMP_In          (JMP_In),
    .N               (N),
    .Z               (Z),
    .P               (P)
	);

endmodule
