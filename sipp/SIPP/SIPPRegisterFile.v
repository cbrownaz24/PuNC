//==============================================================================
// Register File with 2 read ports, 1 write port
//==============================================================================

module SIPPRegisterFile
#(
	parameter N_ELEMENTS = 16,       	// Number of Memory Elements
	parameter ADDR_WIDTH = 4,         	// Address Width (in bits)
	parameter DATA_WIDTH = 16         	// Data Width (in bits)
)(
	// Clock + Reset
	input                   clk,      	// Clock
	input                   rst,      	// Reset (All entries -> 0)

	// Read Address Channel
	input  [ADDR_WIDTH-1:0] p_addr, 	// Read Address p
	input  [ADDR_WIDTH-1:0] q_addr, 	// Read Address q
	input					p_rd,		// Read p Enable
	input					q_rd,		// Read q Enable

	// Write Address, Data Channel
	input  [ADDR_WIDTH-1:0] w_addr,   	// Write Address
	input  [DATA_WIDTH-1:0] w_data,   	// Write Data
	input                   w_wr,     	// Write Enable

	// Read Data Channel
	output [DATA_WIDTH-1:0] p_data, 	// Read Data p
	output [DATA_WIDTH-1:0] q_data 	 	// Read Data q

);

	// Memory Unit
	reg [DATA_WIDTH-1:0] rfile[N_ELEMENTS-1:0];

	// Continuous Read
	assign p_data = (p_rd) ? rfile[p_addr] : {(DATA_WIDTH){1'd0}};
	assign q_data = (q_rd) ? rfile[q_addr] : {(DATA_WIDTH){1'd0}};

	// Synchronous Reset + Write
	genvar i;
	generate
		for (i = 0; i < N_ELEMENTS; i = i + 1) begin : wport
			always @(posedge clk) begin
				if (rst) begin
					rfile[i] <= 0;
				end
				else if (w_wr && w_addr == i) begin
					rfile[i] <= w_data;
				end
			end
		end
	endgenerate

endmodule

