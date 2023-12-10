//==============================================================================
// Example Controller for Six-Instruction Programmable Processor
//==============================================================================

`include "SIPPDefines.v"

module SIPPControl(
   // External Inputs
   input  wire            clk,            // Clock
   input  wire            rst,            // Reset

   // Input Signals from Datapath
   input [15:0]           ir,
   input                  rf_p_zero,

   // Memory Controls
   output reg             d_rd,
   output reg             d_wr,

   // Register File Controls
   output reg             rf_w_wr,
   output reg             rf_p_rd,
   output reg             rf_q_rd,
   output reg             rf_p_addr_sel,
   output reg      [1:0]  rf_w_data_sel,

   // Instruction Register Controls
   output reg             ir_ld,

   // Program Counter Controls
   output reg             pc_ld,
   output reg             pc_clr,
   output reg             pc_inc,

   // ALU Controls
   output reg      [1:0]  alu_s
);

   // FSM States
   localparam STATE_INIT      = 3'b000;
   localparam STATE_FETCH     = 3'b001;
   localparam STATE_DECODE    = 3'b010;
   localparam STATE_EXECUTE   = 3'b011;
   localparam STATE_EXECUTE_I = 3'b100;
   localparam STATE_HALT      = 3'b101;

   // State, Next State
   reg [2:0] state, next_state;

   // Output Combinational Logic
   always @( * ) begin

      // Prevent implicit latching
      d_rd           = 1'd0;
      d_wr           = 1'd0;
      rf_w_wr        = 1'd0;
      rf_p_rd        = 1'd0;
      rf_q_rd        = 1'd0;
      rf_p_addr_sel  = 1'd0;
      rf_w_data_sel  = 2'd0;
      ir_ld          = 1'd0;
      pc_ld          = 1'd0;
      pc_clr         = 1'd0;
      pc_inc         = 1'd0;
      alu_s          = 2'd0;

      case (state)
         STATE_INIT: begin
            pc_clr     = 1'd1;
         end
         STATE_FETCH: begin
            ir_ld      = 1'd1;
            pc_inc     = 1'd1; // NOTE: In PUnC, this happens after Fetch AND Decode
         end
         STATE_DECODE: begin

         end
         STATE_EXECUTE: begin
            case (ir[`OC])
               `OC_LOAD: begin
                  d_rd           = 1'd1;
                  rf_w_wr        = 1'd1;
                  rf_w_data_sel  = `RF_W_DATA_SEL_MEM;
               end
               `OC_STORE: begin
                  d_wr           = 1'd1;
                  rf_p_rd        = 1'd1;
                  rf_p_addr_sel  = `RF_RP_ADDR_SEL_A;
               end
               `OC_ADD: begin
                  rf_w_wr        = 1'd1;
                  rf_p_rd        = 1'd1;
                  rf_q_rd        = 1'd1;
                  rf_p_addr_sel  = `RF_RP_ADDR_SEL_B;
                  rf_w_data_sel  = `RF_W_DATA_SEL_ALU;
                  alu_s          = `ALU_FN_ADD;
               end
               `OC_LOADC: begin
                  rf_w_wr        = 1'd1;
                  rf_w_data_sel  = `RF_W_DATA_SEL_IR;
               end
               `OC_SUBTR: begin
                  rf_w_wr        = 1'd1;
                  rf_p_rd        = 1'd1;
                  rf_q_rd        = 1'd1;
                  rf_p_addr_sel  = `RF_RP_ADDR_SEL_B;
                  rf_w_data_sel  = `RF_W_DATA_SEL_ALU;
                  alu_s          = `ALU_FN_SUBTR;
               end
               `OC_JMPZ: begin
                  rf_p_rd        = 1'd1;
                  rf_p_addr_sel  = `RF_RP_ADDR_SEL_A;
               end             
               default: begin

               end
            endcase
         end
         STATE_EXECUTE_I: begin
            case (ir[`OC])
               `OC_JMPZ: begin
                  pc_ld = 1'd1;
               end
            endcase
         end
         STATE_HALT: begin

         end
         default: begin

         end
      endcase
   end

   // Next State Combinational Logic
   always @( * ) begin
      next_state = state;

      case (state)
         STATE_INIT: begin
            next_state = STATE_FETCH;
         end
         STATE_FETCH: begin
            next_state = STATE_DECODE;
         end
         STATE_DECODE: begin
            next_state = STATE_EXECUTE;
         end
         STATE_EXECUTE: begin
            if (ir[`OC] == `OC_JMPZ) begin
               if (rf_p_zero) begin
                  next_state = STATE_EXECUTE_I;
               end
               else begin
                  next_state = STATE_FETCH;
               end
            end
            else begin
               next_state = STATE_FETCH;
            end
         end
         STATE_EXECUTE_I: begin
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
