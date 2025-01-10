//==============================================================================
// Datapath for PUnC LC3 Processor
//==============================================================================

`include "Defines.v"

module PUnCDatapath(
	// External Inputs
	input  wire        clk,            // Clock
	input  wire        rst,            // Reset

	// DEBUG Signals
	input  wire [15:0] mem_debug_addr,
	input  wire [2:0]  rf_debug_addr,
	output wire [15:0] mem_debug_data,
	output wire [15:0] rf_debug_data,
	output wire [15:0] pc_debug_data,

	// Add more ports here
	
	input wire  [1:0] MUX_input,
	input wire  [1:0] PCMUX,
	input wire  pcld,
	input wire  pcclr,
	input wire  [1:0] MUX,
	input wire  MUX_w_addr,
    input wire  mem_en,  //w_en memory
    input wire  irld,
    input wire  [2:0] reg_w_addr,
    input wire  reg_w_en,
    input wire  [2:0] regIn1,
    input wire  [2:0] regIn2,
    input wire  signed [15:0] regIn3,
    input wire  cond_chk,
    input wire  reg_chk1,
    input wire  reg_chk2,
    input wire  [3:0] ALU_Cur,
    input wire  cond_en,
    output reg  JMP_In,
    output reg  N,
    output reg  Z,
    output reg  P,
    output reg  [15:0] ir
);

	// Local Registers
	reg  [15:0] pc = 0;
	reg  signed [15:0] ALU_Out;

	// Declare other local wires and registers here
	reg  [15:0] PCoffsetSum;
	reg  [15:0] Mem_store;
	
	//INPUT and OUTPUT to/from PC
	//pcld
	//pcclr
    wire pcup = 4'b0001;	
	//Input and Output to/from IR
	//mem_out (input)
	//IRld (input) What should it be when it's not in fetch?
	//ir (output)

    //Memory Input & Output 
    // 1. PC_DATA (r_addr_0)
    // 2. mem_debug_addr
    // 3. RegOut1 (w_data)
    // 4. mem_debug_data
    reg  [15:0] mem_in;
    wire signed [15:0] mem_out; //r_data_0
    reg  [15:0] mem_w_addr; //w_addr memory
    
    
    //MUX Loading to Register
    // 1. mem_out (1)
    // 2. PC_DATA (2)
    reg  signed [15:0] Load_Reg;
    
    //Register
    // wire RegIn1;
    // wire RegIn2;
    //rf_debug_addr
    // wire Reg_w_addr;
    // wire Reg_w_en;
    //Load_Reg (w_data)
    //rst
    wire signed [15:0] RegOut1;
    wire signed [15:0] RegOut2;
    //rf_debug_data 
    reg signed [15:0] A;
    reg signed [15:0] B;
    
    //ALU 
    //RegOut1 (A)
    //RegOut2 (B)
    
    //Condition Register MUX
    //Cond_chk
    //Load_Reg (0)
    //ALU_Out (1)
    reg signed [15:0] cond_out;
	
	
	
	// Assign PC debug net
	assign pc_debug_data = pc;


	//----------------------------------------------------------------------
	// Memory Module
	//----------------------------------------------------------------------

	// 1024-entry 16-bit memory (connect other ports)
	Memory mem(
		.clk      (clk),
		.rst      (rst),
		.r_addr_0 (mem_in),
		.r_addr_1 (mem_debug_addr),
		.w_addr   (mem_w_addr),
		.w_data   (B),
		.w_en     (mem_en),
		.r_data_0 (mem_out),
		.r_data_1 (mem_debug_data)
	);

	//----------------------------------------------------------------------
	// Register File Module
	//----------------------------------------------------------------------

	// 8-entry 16-bit register file (connect other ports)
	RegisterFile rfile(
		.clk      (clk),
		.rst      (rst),
		.r_addr_0 (regIn1),
		.r_addr_1 (regIn2),
		.r_addr_2 (rf_debug_addr),
		.w_addr   (reg_w_addr),
		.w_data   (Load_Reg),
		.w_en     (reg_w_en),
		.r_data_0 (RegOut1),
		.r_data_1 (RegOut2),
		.r_data_2 (rf_debug_data)
	);
	
	always @(posedge clk) begin
        // Store level
        if (rst) begin
            N <= 0;
            P <= 0;
            Z <= 0;
            pc <= 0;
            // Also Reset everything in Memory (To do)
        end
        
        //PC MUX Setting
        if (PCMUX == 2'b00) begin
            pc <= PCoffsetSum;
        end
        if (PCMUX == 2'b01) begin
            pc <= pc + pcup;
        end
        
        if (MUX_input == 2'b00) begin //Fetch
           mem_in <= pc;
        end
        else if (MUX_input == 2'b10) begin
           mem_in <= PCoffsetSum; 
        end
        else if (MUX_input == 2'b11) begin
           mem_in <= Mem_store;
        end

        
        if (irld == 1) begin
           ir <= mem_out;
        end
        
        PCoffsetSum <= ALU_Out;
        Mem_store <= mem_out;
        

         
         if (pcclr == 1) begin
            pc <= 16'd0;
         end
    end

	//----------------------------------------------------------------------
	// Add all other datapath logic here
	//----------------------------------------------------------------------
    always @( * ) begin
    
        
         
        // Output Logic Here
        if (reg_chk1 == 0) begin
            A = pc;
        end
        else begin
            A = RegOut1;
        end
        if (reg_chk2 == 0) begin
            B = regIn3;
        end
        else begin
            B = RegOut2;
        end
        
        
        //ALU_CUR setting
        if (ALU_Cur == 4'b0001) begin
            ALU_Out = A + B;
        end
        else if (ALU_Cur == 4'b0010) begin
            ALU_Out = A & B;
        end
        else if (ALU_Cur == 4'b0100) begin
            ALU_Out = A;
        end
        else if (ALU_Cur == 4'b1000) begin
            ALU_Out = ~(A);
        end
        
        //Condition Code Setting
        if (cond_chk == 1) begin
            cond_out = ALU_Out;
        end
        else begin
            cond_out = Load_Reg;
        end
        
        //Register Input MUX Setting
        if (MUX == 2'b00) begin
            Load_Reg = ALU_Out;
        end
        if (MUX == 2'b01) begin
            Load_Reg = mem_out;
        end
        if (MUX == 2'b10) begin
            Load_Reg = pc;
        end
        
        //R_Addr_0 Memory MUX Setting
        if (MUX_input == 2'b00) begin
            mem_in = pc;
        end
        if (MUX_input == 2'b10) begin
            mem_in = PCoffsetSum;
        end
        if (MUX_input == 2'b11) begin
            mem_in = Mem_store;
        end
        

        
        //mem_w_addr MUX Setting
        if (MUX_w_addr == 0) begin
            mem_w_addr = PCoffsetSum;
        end
        else begin 
            mem_w_addr = Mem_store;
        end
        
        // Change according to 2 bit complement
        if (cond_en == 1) begin
            if (cond_out == 16'd0) begin
                 N = 0;
                 Z = 1;
                 P = 0;
            end
            else if (cond_out[15] == 1) begin
                 N = 1;
                 Z = 0;
                 P = 0;
            end 
            else if (cond_out[15] == 0) begin
                 N = 0;
                 Z = 0;
                 P = 1;
            end
         end
        
        
    end
endmodule
