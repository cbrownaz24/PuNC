//==============================================================================
// Global Defines for Six-Instruction Programmable Processor
//==============================================================================

//------------------------------------------------------------------------------
// Control Signals
//------------------------------------------------------------------------------

// Register File
`define RF_RP_ADDR_SEL_A 	 1'b0
`define RF_RP_ADDR_SEL_B	 1'b1

`define RF_W_DATA_SEL_ALU    2'b00
`define RF_W_DATA_SEL_MEM    2'b01
`define RF_W_DATA_SEL_IR     2'b10

// ALU
`define ALU_FN_PASS          2'b00
`define ALU_FN_ADD           2'b01
`define ALU_FN_SUBTR         2'b10

//------------------------------------------------------------------------------
// Opcodes
//------------------------------------------------------------------------------
`define OC 15:12

`define OC_LOAD 	4'b0000
`define OC_STORE 	4'b0001
`define OC_ADD 		4'b0010
`define OC_LOADC 	4'b0011
`define OC_SUBTR 	4'b0100
`define OC_JMPZ 	4'b0101

`define REG_A		11:8
`define REG_B		7:4
`define REG_C		3:0
`define MEM_ADDR 	7:0
`define CONSTANT 	7:0
`define OFFSET 		7:0
