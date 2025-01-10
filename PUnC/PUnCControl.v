//==============================================================================
// Control Unit for PUnC LC3 Processor
//==============================================================================

`include "Defines.v"

module PUnCControl(
	// External Inputs
	input  wire        clk,            // Clock
	input  wire        rst,            // Reset
	
	
	// Memory Controls 
	output reg  [1:0]  MUX_input,
	//....
	output reg  mem_en,
	output reg  [1:0] PCMUX,
	
	//MUX Control
	output reg  [1:0] MUX,
	output reg MUX_w_addr,

	// Register File Controls
	//....
	output reg [2:0] reg_w_addr,
	output reg reg_w_en,
	output reg [2:0] regIn1,
	output reg [2:0] regIn2,
	output reg signed [15:0] regIn3,
	
	//Register MUX Control
	output reg reg_chk1,
	output reg reg_chk2,

	// Instruction Register Controls
	//...
	input wire [15:0] ir,
	output reg irld,

	// Program Counter Controls
	//...
	output reg pcclr,
	output reg pcld,
	
	//ALU Choice
	output reg [3:0] ALU_Cur,


	// Add more ports here
	
	//Condition Control
	output reg cond_chk,
	output reg cond_en,
	
	//Jump Input
	input wire JMP_In,
	
	//Current N,Z,P State
	input wire N,
	input wire Z,
	input wire P

);

	// FSM States
	// Add your FSM State values as localparams here
	localparam STATE_INIT      = 5'b11111;
	localparam STATE_FETCH     = 5'b00000;
	localparam STATE_DECODE     = 5'b00001;
	localparam STATE_EXECUTE_ADD     = 5'b00010;
	localparam STATE_EXECUTE_AND     = 5'b00011;
    localparam STATE_EXECUTE_BR      = 5'b00100;
    localparam STATE_EXECUTE_BR2     = 5'b00101;
    localparam STATE_EXECUTE_JMP     = 5'b00110;
    localparam STATE_EXECUTE_JMP2    = 5'b00111;
    localparam STATE_EXECUTE_JSR_R   = 5'b01000;
    localparam STATE_EXECUTE_JSR_R2  = 5'b01001;
    localparam STATE_EXECUTE_LD      = 5'b01010;
    localparam STATE_EXECUTE_LD2     = 5'b01011;
    localparam STATE_EXECUTE_LDI     = 5'b01100;
    localparam STATE_EXECUTE_LDI2    = 5'b01101;
    localparam STATE_EXECUTE_LDI3    = 5'b01110;
    localparam STATE_EXECUTE_LDR     = 5'b01111;
    localparam STATE_EXECUTE_LDR2    = 5'b10000;
    localparam STATE_EXECUTE_LEA     = 5'b10001;
    localparam STATE_EXECUTE_NOT     = 5'b10010;
    localparam STATE_EXECUTE_RET     = 5'b10011;
    localparam STATE_EXECUTE_ST      = 5'b10100;
    localparam STATE_EXECUTE_ST2     = 5'b10101;
    localparam STATE_EXECUTE_STI     = 5'b10110;
    localparam STATE_EXECUTE_STI2    = 5'b10111;
    localparam STATE_EXECUTE_STI3    = 5'b11000;
    localparam STATE_EXECUTE_STR     = 5'b11001;
    localparam STATE_EXECUTE_STR2    = 5'b11010;
    localparam STATE_EXECUTE_HALT    = 5'b11011;
	
	// State, Next State
	reg [4:0] state, next_state;
	reg [2:0] DR;
	reg [2:0] SR, SR1, BaseR;
	reg [2:0] SR2;
	reg signed [15:0] imm5;
	reg signed [15:0] offset6;
	reg signed [15:0] PCoffset9;
	reg signed [15:0] PCoffset11;
	reg n;
	reg z;
	reg p;
	reg [2:0] counter;
	
	

	// Output Combinational Logic
	always @( * ) begin
		// Set default values for outputs here (prevents implicit latching)

		// Add your output logic here
		 
          irld          = 1'd0;
          pcld          = 1'd0;
          pcclr         = 1'd0;
          ALU_Cur          = 4'd0;
          	//Condition Control
	      cond_chk     = 1'd0;
	      cond_en     = 1'd0;
	
      
		case (state)
		    STATE_INIT: begin
		      pcclr = 1'd1;
		    end
			STATE_FETCH: begin
		      MUX_input = 2'b00;
		      irld = 1;
		      PCMUX = 2'b10;
		      regIn1 = 0;
		      regIn2 = 0;
		      reg_w_en = 0;
		      mem_en = 0;
			end
			STATE_DECODE: begin
			    MUX_input = 2'b01;
			    PCMUX = 2'b01;
			    irld = 0;
			    DR  = ir[11:9];
                SR = ir[11:9];
                SR1 = ir[8:6];
                BaseR = ir[8:6];
                SR2 = ir[2:0];
                imm5  = {{11{ir[4]}}, ir[4:0]};
                offset6 = {{10{ir[5]}}, ir[5:0]};
                PCoffset9 = {{7{ir[8]}}, ir[8:0]};
                PCoffset11 = {{5{ir[10]}}, ir[10:0]};
                n = ir[11];
                z = ir[10];
                p = ir[9];
			end
			STATE_EXECUTE_ADD: begin
            // Add logic for STATE_EXECUTE_AND
                if (ir[5] == 0) begin
                  reg_w_addr = DR;
                  regIn1 = SR1;
                  regIn2 = SR2;
                  PCMUX = 2;
                  reg_w_en = 1;
                  reg_chk1 = 1;
                  reg_chk2 = 1;
                  MUX = 2'b00;
                  ALU_Cur = 4'b0001;
                  cond_chk = 1;
                  cond_en = 1;
                  irld = 0;
                end
                else begin
                  reg_w_addr = DR;
                  regIn1 = SR1;
                  regIn3 = imm5;
                  reg_w_en = 1;
                  reg_chk1 = 1;
                  reg_chk2 = 0;
                  MUX = 2'b00;
                  PCMUX = 2;
                  ALU_Cur = 4'b0001;
                  cond_chk = 1;
                  cond_en = 1;
                  irld = 0;
                end
            end
            STATE_EXECUTE_AND: begin
                if (ir[5] == 0) begin
                  reg_w_addr = DR;
                  regIn1 = SR1;
                  regIn2 = SR2;
                  reg_w_en = 1;
                  reg_chk1 = 1;
                  reg_chk2 = 1;
                  MUX = 2'b00;
                  PCMUX = 2;
                  ALU_Cur = 4'b0010;
                  cond_chk = 1;
                  cond_en = 1;
                  irld = 0;
                end
                else begin
                  reg_w_addr = DR;
                  regIn1 = SR1;
                  regIn3 = imm5;
                  reg_w_en = 1;
                  reg_chk1 = 1;
                  PCMUX = 2;
                  reg_chk2 = 0;
                  MUX = 2'b00;
                  ALU_Cur = 4'b0010;
                  cond_chk = 1;
                  cond_en = 1;
                  irld = 0;
                end
                // Add logic for STATE_EXECUTE_BR
            end
            STATE_EXECUTE_BR: begin
 
                  if ((n == 1 && N == 1) || (z == 1 && Z == 1) || (p == 1 && P == 1)) begin 
                      reg_chk1 = 0;
                      reg_chk2 = 0;
                      regIn3 = PCoffset9;
                      ALU_Cur = 4'b0001;
                      irld = 0;
                      cond_en = 0;
                      PCMUX = 2;
                // Add logic for STATE_EXECUTE_BR
                  end
                  else begin
                      PCMUX = 2;
                      cond_en = 0;
                  end
          
            end
            
            STATE_EXECUTE_BR2: begin
                 if ((n == 1 && N == 1) || (z == 1 && Z == 1) || (p == 1 && P == 1)) begin 
                      PCMUX = 0;
                // Add logic for STATE_EXECUTE_BR
                  end
                  else begin
                      PCMUX = 2;
                  end
                // Add logic for STATE_EXECUTE_BR2
            end
            
            STATE_EXECUTE_JMP: begin
                  regIn1 = BaseR;
                  reg_chk1 = 1;
                  ALU_Cur = 4'b0100;
                  cond_en = 0;
                  PCMUX = 0;
                  irld = 0;
                // Add logic for STATE_EXECUTE_JMP
            end
            
            STATE_EXECUTE_JMP2: begin
                   
                // Add logic for STATE_EXECUTE_JMP2
            end
            
            STATE_EXECUTE_JSR_R: begin
                reg_w_en = 1;
                reg_w_addr = 3'b111;
                MUX = 2;
                if (ir[11] == 0) begin
                  regIn1 = BaseR;
                  reg_chk1= 1;
                  ALU_Cur = 4'b0100;
                  cond_en = 0;
                  PCMUX = 2;
                  irld = 0;
               
                end
                else begin
                  reg_chk1 = 0;
                  PCMUX = 2;
                  regIn3 = PCoffset11;
		          reg_chk2 = 0;
                  ALU_Cur = 4'b0001;
                  cond_en = 0;
                  irld = 0;
                end
                // Add logic for STATE_EXECUTE_JSR_R
            end
            
            STATE_EXECUTE_JSR_R2: begin
                PCMUX = 0;
                // Add logic for STATE_EXECUTE_JSR_R2
            end
            
            STATE_EXECUTE_LD: begin
                  reg_w_en = 1;
                  reg_w_addr = DR;
                  reg_chk1 = 0;
                  regIn3 = PCoffset9;
                  reg_chk2 = 0;
                  ALU_Cur = 4'b0001;
                  cond_en = 1;
                  PCMUX = 2;
                  cond_chk = 0;
                  irld = 0;
                // Add logic for STATE_EXECUTE_LD
            end
            
            STATE_EXECUTE_LD2: begin
                  MUX_input = 2;
                  MUX = 1;  
                // Add logic for STATE_EXECUTE_LD2
            end
            
            STATE_EXECUTE_LDI: begin
                regIn1 = SR1;
                reg_w_addr = DR;
                reg_w_en = 1;
                reg_chk1 = 0;
                reg_chk2 = 0;
                ALU_Cur = 4'b0001;
                regIn3 = PCoffset9;
                cond_chk = 0;
                PCMUX = 2;
                cond_en = 1;
                irld = 0;
                
                // Add logic for STATE_EXECUTE_LDI
            end
            
            STATE_EXECUTE_LDI2: begin
                MUX_input = 2;
                cond_chk = 0;
                cond_en = 1;
                // Add logic for STATE_EXECUTE_LDI2
            end
            
            STATE_EXECUTE_LDI3: begin
                MUX_input = 3;
                MUX = 1;
                cond_chk = 0;
                cond_en = 1;
                // Add logic for STATE_EXECUTE_LDI3
            end
            
            STATE_EXECUTE_LDR: begin
                reg_w_en = 1;
                  regIn1 = BaseR;
                  reg_w_addr = DR;
                  reg_chk1 = 1;
                  reg_chk2 = 0;
                  regIn3 = offset6;
                  ALU_Cur = 4'b0001;
                  MUX_input = 2;
                  MUX = 1;
                  cond_en = 1;
                  PCMUX = 2;
                  cond_chk = 0;
                  irld = 0;
                // Add logic for STATE_EXECUTE_LDR
            end
            
            STATE_EXECUTE_LDR2: begin
                // Add logic for STATE_EXECUTE_LDR2
            end
            
            STATE_EXECUTE_LEA: begin
                 reg_w_en = 1;
                  reg_w_addr = DR;
                  reg_chk1 = 0;
                  reg_chk2 = 0;
                  PCMUX = 2;
                  ALU_Cur = 4'b0001;
                  regIn3 = PCoffset9;
                  MUX = 0;
                  cond_en = 1;
                  cond_chk = 1;
                  irld = 0;
                // Add logic for STATE_EXECUTE_LEA
            end
            
            STATE_EXECUTE_NOT: begin
                reg_w_en = 1;
                  reg_w_addr = DR;
                  regIn1 = SR1;
                  reg_chk1 = 1;
                  ALU_Cur = 4'b1000;
                  PCMUX = 2;
                  MUX = 0;
                  cond_en = 1;
                  cond_chk = 1;
                  irld = 0;
                // Add logic for STATE_EXECUTE_NOT
            end
            
            STATE_EXECUTE_RET: begin
                // Add logic for STATE_EXECUTE_RET
                  regIn1 = 3'b111;
                  reg_chk1 = 1;
                  ALU_Cur = 4'b0100;
                  PCMUX = 0;
                  MUX = 0;  
                  cond_en = 0;
                  irld = 0;
            end
            
            STATE_EXECUTE_ST: begin
                // Add logic for STATE_EXECUTE_ST
                reg_chk1 = 0;
                reg_chk2 = 0;
                MUX_w_addr = 0;
                ALU_Cur = 4'b0001;
                PCMUX = 2;
                cond_en = 0;
                reg_w_en = 0;
                irld = 0;
                regIn3 = PCoffset9;
                
            end
            
            STATE_EXECUTE_ST2: begin
                // Add logic for STATE_EXECUTE_ST2
                reg_chk2 = 1;
                MUX = 1;
                mem_en = 1;
                regIn2 = SR;
                MUX_input = 2;
            end
            
            STATE_EXECUTE_STI: begin
                // Add logic for STATE_EXECUTE_STI
                reg_chk1 = 0;
                reg_chk2 = 0;
                MUX_w_addr = 1;
                ALU_Cur = 4'b0001;
                cond_en = 0;
                PCMUX = 2;
                irld = 0;
                regIn3 = PCoffset9;
            end
            
            STATE_EXECUTE_STI2: begin
                // Add logic for STATE_EXECUTE_STI2
                MUX_input = 2'b10;
            end
            
            STATE_EXECUTE_STI3: begin
                // Add logic for STATE_EXECUTE_STI3
                MUX_input = 2'b10;
                regIn2 = SR;
                reg_chk2 = 1;
                PCMUX = 2;
                mem_en = 1;
            end
            
            STATE_EXECUTE_STR: begin
                regIn1 = BaseR;
                reg_chk1 = 1;
                reg_chk2 = 0;
                MUX_w_addr = 0;
                ALU_Cur = 4'b0001;
                PCMUX = 2;
                cond_en = 0;
                irld = 0;
                regIn3 = offset6;
                // Add logic for STATE_EXECUTE_STR
            end
            
            STATE_EXECUTE_STR2: begin
                reg_chk2 = 1;
                MUX = 1; 
                mem_en = 1;
                regIn2 = SR;
                // Add logic for STATE_EXECUTE_STR2
            end
            
            STATE_EXECUTE_HALT: begin
                PCMUX = 2'b10;
            end
		endcase
	end

	// Next State Combinational Logic
	always @( * ) begin
		// Set default value for next state here
		next_state = state;

		// Add your next-state logic here
		case (state)
		    STATE_INIT: begin
		          next_state = STATE_FETCH;
		    end
			STATE_FETCH: begin
		       if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		       else begin 
		          next_state = STATE_DECODE;
		       end
			end
			STATE_DECODE: begin
			   if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		       else begin 
		       if (ir[15:12] == 4'b0001) begin
		            ALU_Cur = 4'b0001;
                    next_state = STATE_EXECUTE_ADD;
                end
                else if (ir[15:12] == 4'b0101) begin
                    next_state = STATE_EXECUTE_AND;
                end
                else if (ir[15:12] == 4'b0000) begin
                    next_state = STATE_EXECUTE_BR;
                end
                else if (ir[15:12] == 4'b1100) begin
                    next_state = STATE_EXECUTE_JMP;
                end 
                else if (ir[15:12] == 4'b0100) begin
                    next_state = STATE_EXECUTE_JSR_R;
                end
                else if (ir[15:12] == 4'b0010) begin
                    next_state = STATE_EXECUTE_LD;
                end
                else if (ir[15:12] == 4'b1010) begin
                    next_state = STATE_EXECUTE_LDI;
                end
                else if (ir[15:12] == 4'b0110) begin
                    next_state = STATE_EXECUTE_LDR;
                end
                else if (ir[15:12] == 4'b1110) begin
                    next_state = STATE_EXECUTE_LEA;
                end
                else if (ir[15:12] == 4'b1001) begin
                    next_state = STATE_EXECUTE_NOT;
                end
                else if (ir[15:12] == 4'b1100) begin
                    next_state = STATE_EXECUTE_RET;
                end
                else if (ir[15:12] == 4'b0011) begin
                    next_state = STATE_EXECUTE_ST;
                end
                else if (ir[15:12] == 4'b1011) begin
                    next_state = STATE_EXECUTE_STI;
                end
                else if (ir[15:12] == 4'b0111) begin
                    next_state = STATE_EXECUTE_STR;
                end
                else if (ir[15:12] == 4'b1111) begin
                    next_state = STATE_EXECUTE_HALT;
                end     
		       end
			end
			STATE_EXECUTE_JSR_R: begin
			   if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_JSR_R2;	
            end
            STATE_EXECUTE_BR: begin
                if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_BR2;	
            end
            STATE_EXECUTE_JMP: begin
                if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_JMP2;	
            end
            STATE_EXECUTE_LD: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_LD2;	
            end
            STATE_EXECUTE_LDI: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_LDI2;	
            end
            STATE_EXECUTE_LDI2: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_LDI3;	
            end
            STATE_EXECUTE_LDR: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_LDR2;	
            end
            STATE_EXECUTE_ST: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_ST2;	
            end
            STATE_EXECUTE_STI: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_STI2;	
            end
            STATE_EXECUTE_STI2: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_STI3;	
            end
            STATE_EXECUTE_STR: begin
               if (rst == 1) begin
		          next_state = STATE_INIT;
		       end
		          next_state = STATE_EXECUTE_STR2;	
            end
            STATE_EXECUTE_ADD: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_AND: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_BR2: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_JMP2: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_JSR_R2: begin
                next_state = STATE_FETCH;
            end
            STATE_EXECUTE_LD2: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_LDI3: begin
                next_state = STATE_FETCH;
            end 
            
            STATE_EXECUTE_LDR2: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_LEA: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_NOT: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_RET: begin
                next_state = STATE_FETCH;
            end
            STATE_EXECUTE_ST2: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_STI3: begin
                next_state = STATE_FETCH;
            end
            
            STATE_EXECUTE_STR2: begin
                next_state = STATE_FETCH;
            end

		endcase
	end

	// State Update Sequential Logic
	always @(posedge clk) begin
		if (rst) begin
			state <= STATE_INIT;
		end
		else begin
			state <= next_state;
		end
	end

endmodule
