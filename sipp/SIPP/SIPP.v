//==============================================================================
// Module for Six-Instruction Programmable Processor
//==============================================================================

`include "SIPPDatapath.v"
`include "SIPPControl.v"

module SIPP(
	// External Inputs
	input  wire        clk,            // Clock
	input  wire        rst,            // Reset
	input  wire [15:0] instruction     // Instruction
);

	//----------------------------------------------------------------------
	// Interconnect Wires
	//----------------------------------------------------------------------

	// Memory Controls
	wire        d_rd;
	wire        d_wr;

	// Register File Controls
	wire        rf_w_wr;
	wire        rf_p_rd;
	wire        rf_q_rd;
	wire 		rf_p_addr_sel;
	wire [1:0]  rf_w_data_sel;


	// Instruction Register Controls
	wire        ir_ld;

	// Program Counter Controls
	wire        pc_ld;
	wire		pc_clr;
	wire		pc_inc;

	// ALU Controls
	wire [1:0]  alu_s;

	// Datapath Outputs
	wire [15:0] ir;
	wire 		rf_p_zero;

	

	//----------------------------------------------------------------------
	// Control Module
	//----------------------------------------------------------------------
	SIPPControl ctrl(
		.clk             (clk),
		.rst             (rst),

		.ir              (ir),
		.rf_p_zero		 (rf_p_zero),

		.d_rd			 (d_rd),			 
		.d_wr			 (d_wr),

		.rf_w_wr		 (rf_w_wr),		
		.rf_p_rd		 (rf_p_rd),		
		.rf_q_rd		 (rf_q_rd),		
		.rf_p_addr_sel	 (rf_p_addr_sel),		
		.rf_w_data_sel	 (rf_w_data_sel),	 		

		.ir_ld           (ir_ld),

		.pc_ld			 (pc_ld), 						
		.pc_clr			 (pc_clr), 	 		
		.pc_inc			 (pc_inc), 			

		.alu_s           (alu_s)
	);

	//----------------------------------------------------------------------
	// Datapath Module
	//----------------------------------------------------------------------
	SIPPDatapath dpath(
		.clk             (clk),
		.rst             (rst),
		.instruction 	 (instruction),

		.d_rd			 (d_rd),			 
		.d_wr			 (d_wr),

		.rf_w_wr		 (rf_w_wr),		
		.rf_p_rd		 (rf_p_rd),		
		.rf_q_rd		 (rf_q_rd),		
		.rf_p_addr_sel	 (rf_p_addr_sel),	 		
		.rf_w_data_sel	 (rf_w_data_sel),	 		

		.ir_ld           (ir_ld),

		.pc_ld			 (pc_ld), 						
		.pc_clr			 (pc_clr), 	 		
		.pc_inc			 (pc_inc), 			

		.alu_s           (alu_s),

		.ir              (ir),
		.rf_p_zero		 (rf_p_zero)
	);

endmodule
