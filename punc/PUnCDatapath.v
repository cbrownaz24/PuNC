//==============================================================================
// Datapath for PUnC LC3 Processor
//==============================================================================

`include "Memory.v"
`include "RegisterFile.v"
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

	// Inputs from controller
	input wire [15:0] d_w_addr,
	input wire [15:0] d_w_data,
	input wire d_w_en,
	input wire [15:0] d_r_addr,
	input wire d_rst,

	input wire [2:0] rf_w_addr,
	input wire rf_w_en,
	input wire [2:0] rf_r_addr_0,
	input wire [2:0] rf_r_addr_1,
	input wire [15:0] rf_rst,

	input wire n_value,
	input wire z_value,
	input wire p_value,
	input wire npz_clr,
	input wire npz_ld,

	input wire alu_s0,
	input wire alu_s1,

	input wire pc_ld,
	input wire pc_clr,

	input wire ret,
	input wire [1:0] en_1,
	input wire [1:0] en_2,
	input wire en_3,
	
	// Outputs to controller
	output reg npz,
	output wire [15:0] instruction
);

	// Local Registers
	reg  [15:0] pc;
	reg  [15:0] ir;
	reg p;
	reg z;
	reg n;

	// Declare other local wires and registers here
	wire [15:0] d_r_data;
	reg [15:0] rf_w_data;
	wire [15:0] rf_r_data_0;
	wire [15:0] rf_r_data_1;
	reg [15:0] alu_output;
	reg [15:0] alu_a;
	reg [15:0] alu_b;
	wire[0:4] offset5;
	wire [0:5] offset6;
	wire [0:8] offset9;
	wire [0:10] offset11;
	reg [15:0] mux_1;
	reg [15:0] mux_2;
	reg [15:0] mux_ret;
	reg [15:0] pc_adder_output;


	// Assign PC debug net
	assign pc_debug_data = pc;

	//----------------------------------------------------------------------
	// Memory Module
	//----------------------------------------------------------------------

	// 1024-entry 16-bit memory (connect other ports)
	Memory mem(
		.clk      (clk),
		.rst      (rst),
		.r_addr_0 (d_r_addr),
		.r_addr_1 (mem_debug_addr),
		.w_addr   (d_w_addr),
		.w_data   (d_w_data),
		.w_en     (d_w_en),
		.r_data_0 (d_r_data),
		.r_data_1 (mem_debug_data)
	);

	//----------------------------------------------------------------------
	// Register File Module
	//----------------------------------------------------------------------

	// 8-entry 16-bit register file (connect other ports)
	RegisterFile rfile(
		.clk      (clk),
		.rst      (rst),
		.r_addr_0 (rf_r_addr_0),
		.r_addr_1 (rf_r_addr_1),
		.r_addr_2 (rf_debug_addr),
		.w_addr   (rf_w_addr),
		.w_data   (rf_w_data),
		.w_en     (rf_w_en),
		.r_data_0 (rf_r_data_0),
		.r_data_1 (rf_r_data_1),
		.r_data_2 (rf_debug_data)
	);

	//----------------------------------------------------------------------
	// Internal Logic -- Manipulate Registers, ALU's, Memories Local to
	// the Datapath
	//----------------------------------------------------------------------

	always @(posedge clk) begin
		// 16 bit 2x1 mux selector
		if (en_3 == 0) begin
			rf_w_data = alu_output;
		end
		else if (en_3 == 1) begin
			rf_w_data = d_r_data;
		end

		// ALU input A mux
		if (ir[5] == 0) begin
			alu_a = rf_r_data_1;
		end
		if (ir[5] == 1) begin
			alu_a = ir[4:0];
		end

		// ALU Logic
		if (alu_s0 == 0 && alu_s1 == 0) begin
			alu_output = rf_r_data_0;
		end
		if (alu_s0 == 0 && alu_s1 == 1) begin
			alu_output = alu_a + rf_r_data_0;
		end
		if (alu_s0 == 1 && alu_s1 == 0) begin
			alu_output = alu_a & rf_r_data_0;
		end
		if (alu_s0 == 1 && alu_s1 == 1) begin
			alu_output = !rf_r_data_0;
		end

		// Mux 1 Logic (controls which offset is passed through)
		if (en_1 == 00) begin
			mux_1 = ir[10:0];
		end
		else if (en_1 == 01) begin
			mux_1 = ir[8:0];
		end
		else if (en_1 == 10) begin
			mux_1 = ir[5:0];
		end

		// Mux 2 Logic (controls which value is fed into the PC adder)
		if (en_2 == 00) begin
			mux_2 = 1;
		end
		else if (en_2 == 01) begin
			mux_2 = alu_output;
		end
		else if (en_2 == 10) begin
			mux_2 = mux_1;
		end

		// PC adder
		pc_adder_output = pc + mux_2;

		// PC input value
		if (ret == 0) begin
			mux_ret = pc_adder_output;
		end
		else if (ret == 1) begin
			mux_ret = rf_r_data_0;
		end

		// PC logic
		if (pc_ld == 1) begin
			pc = mux_ret;
		end

	end

	//----------------------------------------------------------------------
	// Output Logic -- Set Datapath Outputs
	//----------------------------------------------------------------------

	always @( * ) begin
		// NPZ Logic
		// loading registers N, P, and Z
		if (alu_output > 0) begin
			p = 1;
			z = 0;
			n = 0;
		end
		else if (alu_output == 0) begin
			p = 0;
			z = 1;
			n = 0;
		end
		else if (alu_output < 0) begin
			p = 0;
			z = 0;
			n = 1;
		end

		// determining npz value
		npz =  (p & p_value) | (z & z_value) | (n & n_value);

		// FIGURE OUT HOW TO GET INSTRUCTIONS!!
		
	end

endmodule
