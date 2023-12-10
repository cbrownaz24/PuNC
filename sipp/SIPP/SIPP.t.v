//===============================================================================
// Testbench Module for SIPP 
//===============================================================================
`timescale 1ns/100ps

`include "SIPP.v"

// Start a test using the image file from the /images folder given in NAME
`define START_TEST(NAME)                                                    \
	begin                                                                   \
		$display("\nBeginning Test: %s", (NAME));                           \
		$display("----------------------------");    						\
		fp_r = $fopen({"images/", (NAME), ".txt"}, "r");                    \
		#1;                                                                 \
		rst = 1;                                                            \
		@(posedge clk);                                                     \
		#1;                                                                 \
		rst = 0;                                                            \
		test_cnt = test_cnt + 1;     										\
		@(posedge clk);    													\
		while(!$feof(fp_r)) begin                                           \
			cnt = $fscanf(fp_r, "%h", instruction);							\
			$display("-----------%h-------------", instruction);			\
			@(posedge clk);    												\
			@(posedge clk);    												\
			while(sipp.ctrl.next_state != 3'b001)begin 						\			
				@(posedge clk);    											\
			end 		    												\
		end 																\
	end #0


// Print error message if MEM[ADDR] !== VAL
`define ASSERT_MEM_EQ(ADDR, VAL)                                          \
	begin                                                                 \
		#1;                                                               \
		if (sipp.dpath.mem.mem[ADDR] !== (VAL)) begin                 	  \
			$display("\t[FAILURE]: Expected REG[%h] == %h, but found %h", \
			         (ADDR), (VAL), sipp.dpath.mem.mem[ADDR]);        	  \
			err_cnt = err_cnt + 1;                                        \
		end                                                               \                                                          
	end #0

// Print error message if REG[ADDR] !== VAL
`define ASSERT_REG_EQ(ADDR, VAL)                                          \
	begin                                                                 \
		#1;                                                               \
		if (sipp.dpath.rfile.rfile[ADDR] !== (VAL)) begin                 \
			$display("\t[FAILURE]: Expected REG[%h] == %h, but found %h", \
			         (ADDR), (VAL), sipp.dpath.rfile.rfile[ADDR]);        \
			err_cnt = err_cnt + 1;                                        \
		end                                                               \
	end #0

// Print error message if PC !== VAL
`define ASSERT_PC_EQ(VAL)                                                 \
	begin                                                                 \
		if (sipp.dpath.pc !== (VAL)) begin                                \
			$display("\t[FAILURE]: Expected PC == %d, but found %d",      \
			         (VAL), sipp.dpath.pc);                               \
			err_cnt = err_cnt + 1;                                        \
		end                                                               \
	end #0

module SIPPTest;

	localparam CLK_PERIOD = 10;

	// Testing Variables
	reg  [5:0]  test_cnt = 0;
	reg  [5:0]  err_cnt = 0;


	// // PUnC Interface
	reg         clk = 0;
	reg         rst = 0;
	reg   [15:0]instruction = 0;


	// Clock
	always begin
		#5 clk = ~clk;
	end

	// integer m_i, r_i;
	// initial begin
	// 	$dumpfile("SIPPTest.vcd");
	// 	$dumpvars;
	// 	for (m_i = 0; m_i < 256; m_i = m_i + 1) begin
	// 		$dumpvars(0, sipp.dpath.mem.mem[m_i]);
	// 	end
	// 	for (r_i = 0; r_i < 16; r_i = r_i + 1) begin
	// 		$dumpvars(1, sipp.dpath.rfile.rfile[r_i]);
	// 	end
	// end



	// PUnC Main Module
	SIPP sipp(
		.clk              (clk),
		.rst              (rst),
		.instruction	  (instruction)

	);
	
	integer fp_r, fp_w, cnt;

	initial begin
		$display("\n\n\n===========================");
		$display("=== Beginning All Tests ===");
		$display("===========================");
 
		// @ (posedge clk)
		// $display("--- pc: %d   ----", sipp.dpath.pc);
		`START_TEST("Load");
		`ASSERT_REG_EQ(4'h2, 16'h0008);
		`ASSERT_REG_EQ(4'h3, 16'h00A8);
		`ASSERT_REG_EQ(4'h4, 16'h00F8);
		
		`START_TEST("Store");
		@ (posedge clk)
		`ASSERT_MEM_EQ(8'h05, 16'h0008);
		`ASSERT_MEM_EQ(8'hAA, 16'h00A8);
		`ASSERT_MEM_EQ(8'hFF, 16'h00F8);

		`START_TEST("Loadc");
		@ (posedge clk)
		`ASSERT_REG_EQ(4'h0, 16'h0005);
		`ASSERT_REG_EQ(4'h1, 16'h00AA);
		`ASSERT_REG_EQ(4'hF, 16'h00FF);

		`START_TEST("Add");
		@ (posedge clk)
		`ASSERT_REG_EQ(4'h2, 16'h00B0);
		`ASSERT_REG_EQ(4'h5, 16'h0100);

		`START_TEST("Sub");
		@ (posedge clk)
		`ASSERT_REG_EQ(4'h2, 16'h00A0);
		`ASSERT_REG_EQ(4'h5, 16'h00FE);

		`START_TEST("Jumpz");
		@ (posedge clk)
		`ASSERT_PC_EQ(16'h0056);


		$display("\n----------------------------");
		$display("--- Completed %d Tests  ----", test_cnt);
		$display("--- Found     %d Errors ----", err_cnt);
		$display("----------------------------");
		$display("\n============================");
		$display("===== End of All Tests =====");
		$display("============================");
		$finish;
	end


endmodule
