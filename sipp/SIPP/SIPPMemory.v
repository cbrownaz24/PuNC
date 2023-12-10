//==============================================================================
// Memory with 1 read ports, 1 write port
//==============================================================================

module SIPPMemory
#(
	parameter N_ELEMENTS = 256,   	   	// Number of Memory Elements
	parameter ADDR_WIDTH = 8,     	   	// Address Width (in bits)
	parameter DATA_WIDTH = 16     	   	// Data Width (in bits)
)(
	// Clock + Reset
	input                   clk,   		// Clock
	input                   rst,      	// Reset (All entries -> 0)

	// Address Channel
	input  [ADDR_WIDTH-1:0] addr, 		// Read or Write Address 

	// Enable Signal
	input                   wr,     	// Write Enable
	input                   rd,     	// Read Enable

	// Write Data Channel
	input  [DATA_WIDTH-1:0] w_data,   	// Write Data

	// Read Data Channel
	output [DATA_WIDTH-1:0] r_data		// Read Data 

);

	// Memory Unit
	reg [DATA_WIDTH-1:0] mem[N_ELEMENTS-1:0];


	//---------------------------------------------------------------------------
	// BEGIN MEMORY INITIALIZATION BLOCK
	//   - Paste the code you generate for memory initialization in synthesis
	//     here, deleting the current code.
	//   - Use the LC3 Assembler on Blackboard to generate your Verilog.
	//---------------------------------------------------------------------------
	localparam PROGRAM_LENGTH = 0;
	wire [DATA_WIDTH-1:0] mem_init[0:0];
	//---------------------------------------------------------------------------
	// END MEMORY INITIALIZATION BLOCK
	//---------------------------------------------------------------------------

	// Continuous Read
	assign r_data = (rd) ? mem[addr] : {(DATA_WIDTH){1'd0}};

	// Synchronous Reset + Write
	genvar i;
	generate
		for (i = 0; i < N_ELEMENTS; i = i + 1) begin : wport
			always @(posedge clk) begin
				if (rst) begin
					if (i < PROGRAM_LENGTH) begin
						`ifndef SIM
							mem[i] <= mem_init[i];
						`endif
					end
					else begin
						`ifndef SIM
							mem[i] <= 0;
						`endif
					end
				end
				else if (wr && addr == i) begin
					mem[i] <= w_data;
				end
			end
		end
	endgenerate

endmodule
