//==============================================================================
// Example Datapath for Six-Instruction Programmable Processor
//==============================================================================

`include "SIPPMemory.v"       
`include "SIPPRegisterFile.v"  
`include "SIPPDefines.v"

module SIPPDatapath(
   // External Inputs
   input  wire        clk,            // Clock
   input  wire        rst,            // Reset
   input  wire [15:0] instruction,    // For simplicity, assume instruction is just an input.

   // In reality, this instruction is coming from a separate program memory. In PUnC,
   // it comes from a shared memory.

   // Memory Controls
   input  wire        d_rd,
   input  wire        d_wr,

   // Register File Controls
   input  wire        rf_w_wr,
   input  wire        rf_p_rd,
   input  wire        rf_q_rd,
   input  wire        rf_p_addr_sel,
   input  wire [1:0]  rf_w_data_sel,

   // Instruction Register Controls
   input  wire        ir_ld,

   // Program Counter Controls
   input  wire        pc_ld,
   input  wire        pc_clr,
   input  wire        pc_inc,

   // ALU Controls
   input  wire [1:0]  alu_s,

   // Output Signals to Control
   output reg [15:0]  ir,
   output wire        rf_p_zero
);

   // Local Registers
   reg  [15:0] pc;

   // Memory Read/Write Channels
   wire [7:0]  d_addr;
   wire [15:0] d_w_data;
   wire [15:0] d_r_data;

   // Register File Read/Write Channels
   wire [3:0]  rf_w_addr;
   wire [3:0]  rf_p_addr;
   wire [3:0]  rf_q_addr;
   wire [15:0] rf_w_data;
   wire [15:0] rf_p_data;
   wire [15:0] rf_q_data;

   // ALU Wires
   wire [15:0] alu_out;

   //----------------------------------------------------------------------
   // Memory Module
   //----------------------------------------------------------------------

   assign d_addr = ir[`MEM_ADDR];
   assign d_w_data = rf_p_data;

   // 256-entry 16-bit memory
   SIPPMemory mem(
      .clk      (clk),
      .rst      (rst),
      .addr     (d_addr),
      .rd       (d_rd),
      .wr       (d_wr),
      .w_data   (d_w_data),
      .r_data   (d_r_data)
   );

   //----------------------------------------------------------------------
   // Register File Module
   //----------------------------------------------------------------------

   assign rf_p_addr = (rf_p_addr_sel == `RF_RP_ADDR_SEL_A) ? ir[`REG_A] :
                      (rf_p_addr_sel == `RF_RP_ADDR_SEL_B) ? ir[`REG_B] : ir[`REG_C];

   assign rf_q_addr = ir[`REG_C];


   assign rf_w_data = (rf_w_data_sel == `RF_W_DATA_SEL_ALU) ? alu_out  :
                      (rf_w_data_sel == `RF_W_DATA_SEL_MEM) ? d_r_data :
                      (rf_w_data_sel == `RF_W_DATA_SEL_IR)  ? {{8{1'b0}}, ir[`CONSTANT]} : 16'd0;

   // HINT: {{8{1'b0}}, ir[`CONSTANT]} pads the most significant bits with zeros
   // You can use this syntax to sign-extend.

   assign rf_w_addr = ir[`REG_A];

   // 8-entry 16-bit register file
   SIPPRegisterFile rfile(
      .clk      (clk),
      .rst      (rst),
      .w_addr   (rf_w_addr),
      .p_addr   (rf_p_addr),
      .q_addr   (rf_q_addr),
      .p_rd     (rf_p_rd),
      .q_rd     (rf_q_rd),
      .w_wr     (rf_w_wr),
      .w_data   (rf_w_data),
      .p_data   (rf_p_data),
      .q_data   (rf_q_data)
   );

   //----------------------------------------------------------------------
   // Instruction Register
   //----------------------------------------------------------------------

   always @(posedge clk) begin
      if (rst) begin
         ir <= 16'd0;
      end
      else if (ir_ld) begin
         ir <= instruction;
      end
   end

   //----------------------------------------------------------------------
   // Program Counter
   //----------------------------------------------------------------------

   always @(posedge clk) begin
      if (pc_clr) begin
         pc <= 16'd0;
      end
      else if (pc_ld) begin
         pc <= pc + ir[`OFFSET] - 16'd1;
      end
      else if (pc_inc) begin
         pc <= pc + 16'd1;
      end
   end

   //----------------------------------------------------------------------
   // Comparator ==0?
   //----------------------------------------------------------------------

   assign rf_p_zero = (rf_p_data == 16'd0);

   //----------------------------------------------------------------------
   // ALU
   //----------------------------------------------------------------------

   assign alu_out = (alu_s == `ALU_FN_ADD)     ? (rf_p_data + rf_q_data) :
                    (alu_s == `ALU_FN_SUBTR)   ? (rf_p_data - rf_q_data) :
                    (alu_s == `ALU_FN_PASS)    ? rf_p_data               : rf_p_data;
endmodule
