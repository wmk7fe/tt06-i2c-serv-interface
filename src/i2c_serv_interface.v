// issues: 
// concurrent accesses to different memory buses --> connect multiple memory devices
// concurrent reads and writes to the rf --> need to include rf in design, can shrink size of rf
// confirm rst active low in and active high within design
// check wb to i2c addr bits parameter
// wb cyc, stall not defined
// don't understand timer interrupt
// why is there input and output scl wire for i2c
// using only one reset signal, not one for cpu, one for mem i2c, one for ext i2c

module i2c_serv_interface
#(
  	 parameter	    RF_WIDTH = 8,
	 parameter MEM_ADDR_BITS = 32,
    parameter	    reset_pc = 32'h00000000,
    parameter	    reset_strategy = "MINI",
    parameter [0:0] sim = 1'b0,
    parameter [0:0] with_c = 1'b0,
    parameter [0:0] with_csr = 1'b0,
    parameter [0:0] with_mdu = 1'b0,
    //Internally calculated. Do not touch
    parameter	    regs = 32+with_csr*4,
    parameter	    rf_l2d = $clog2(regs*32/rf_width)
)
( 
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // will go high when the design is enabled
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

	wire mem_i_i2c_sda = ui_in[0];
	wire mem_i_i2c_scl = ui_in[1];
	wire mem_o_i2c_sda = uo_out[0];
	wire mem_o_i2c_scl = uo_out[1];
	
	wire ext_i_i2c_sda = ui_in[2];
	wire ext_i_i2c_scl = ui_in[3];
	wire ext_o_i2c_sda = uo_out[2];
	wire ext_o_i2c_scl = uo_out[3];

/* module servile
  #(
    parameter	    reset_pc = 32'h00000000,
    parameter	    reset_strategy = "MINI",
    parameter	    rf_width = 8,
    parameter [0:0] sim = 1'b0,
    parameter [0:0] with_c = 1'b0,
    parameter [0:0] with_csr = 1'b0,
    parameter [0:0] with_mdu = 1'b0,
    //Internally calculated. Do not touch
    parameter	    regs = 32+with_csr*4,
    parameter	    rf_l2d = $clog2(regs*32/rf_width))
  (
   input wire		      i_clk,
   input wire		      i_rst,
   input wire		      i_timer_irq,

   //Memory (WB) interface
   output wire [31:0]	      o_wb_mem_adr,
   output wire [31:0]	      o_wb_mem_dat,
   output wire [3:0]	      o_wb_mem_sel,
   output wire		      o_wb_mem_we ,
   output wire		      o_wb_mem_stb,
   input wire [31:0]	      i_wb_mem_rdt,
   input wire		      i_wb_mem_ack,

   //Extension (WB) interface
   output wire [31:0]	      o_wb_ext_adr,
   output wire [31:0]	      o_wb_ext_dat,
   output wire [3:0]	      o_wb_ext_sel,
   output wire		      o_wb_ext_we ,
   output wire		      o_wb_ext_stb,
   input wire [31:0]	      i_wb_ext_rdt,
   input wire		      i_wb_ext_ack,

   //RF (SRAM) interface
   output wire [rf_l2d-1:0]   o_rf_waddr,
   output wire [rf_width-1:0] o_rf_wdata,
   output wire		      o_rf_wen,
   output wire [rf_l2d-1:0]   o_rf_raddr,
   input wire [rf_width-1:0]  i_rf_rdata,
   output wire		      o_rf_ren); */
	
	/*
	module	wbi2cmaster #(
		// {{{
		parameter [0:0]		CONSTANT_SPEED = 1'b0, READ_ONLY = 1'b0,
		parameter [5:0]		TICKBITS = 6'd20,
		parameter [(TICKBITS-1):0]	CLOCKS_PER_TICK = 20'd1000,
		parameter 		MEM_ADDR_BITS = 7
		// }}}
	) (
		// {{{
		input	wire		i_clk, i_reset,
		// Wishbone
		// {{{
		// Input bus wires
		input	wire		i_wb_cyc, i_wb_stb, i_wb_we,
		input	wire	[(MEM_ADDR_BITS-2):0]	i_wb_addr,
		input	wire	[31:0]	i_wb_data,
		input	wire	[3:0]	i_wb_sel,
		// Output bus wires
		output	wire		o_wb_stall,
		output	reg		o_wb_ack,
		output	reg	[31:0]	o_wb_data,
		// }}}
		// I2C clock and data wires
		input	wire		i_i2c_scl, i_i2c_sda,
		output	wire		o_i2c_scl, o_i2c_sda,
		// And our output interrupt
		output	wire		o_int,
		// And some debug wires
		output	wire	[31:0]	o_dbg
		// }}}
	);
	*/
	
	// combining rf, mem, and extension memory into one memory block for external memory board
	// memory range of 16 Kb memory is 0x000-0x7FF
	// 32 8 bit registers -> 5 bits to index into rf -> 0x7E0 - 0x7FF reserved for registers
	// rest of memory used for general memory. extensions not used, because connecting other peripherals is used more for fpgas than for asics
	// 0x000-0x7DF memory, 0x7E0-0x7FF rf
	
	// wbi2c io ibus
	reg	ibusi_wb_cyc, ibusi_wb_stb, ibusi_wb_we;
	reg	[(MEM_ADDR_BITS-2):0]	ibusi_wb_addr;
	reg	[31:0]	ibusi_wb_data;
	reg	[3:0]	ibusi_wb_sel;
	reg		ibuso_wb_stall, ibuso_wb_ack;
	reg	[31:0]	ibuso_wb_data;
	reg		ibusi_i2c_scl, ibusi_i2c_sda;
	reg		ibuso_i2c_scl, ibuso_i2c_sda;
	reg		ibuso_int;
	reg	[31:0]	ibuso_dbg;
	
	// serv io
	wire		      serv_i_timer_irq;
   //Memory (WB) interface
   wire [31:0]	      serv_o_wb_mem_adr;
   wire [31:0]	      serv_o_wb_mem_dat;
   wire [3:0]	      serv_o_wb_mem_sel;
   wire		      serv_o_wb_mem_we;
   wire		      serv_o_wb_mem_stb;
   wire [31:0]	      serv_i_wb_mem_rdt;
   wire		      serv_i_wb_mem_ack;
   //Extension (WB) interface
   wire [31:0]	      serv_o_wb_ext_adr;
   wire [31:0]	      serv_o_wb_ext_dat;
   wire [3:0]	      serv_o_wb_ext_sel;
   wire		      serv_o_wb_ext_we;
   wire		      serv_o_wb_ext_stb;
   wire [31:0]	      serv_i_wb_ext_rdt;
   wire		      serv_i_wb_ext_ack;
   //RF (SRAM) interface
   wire [rf_l2d-1:0]   serv_o_rf_waddr;
   wire [RF_WIDTH-1:0] serv_o_rf_wdata;
   wire		      serv_o_rf_wen;
   wire [rf_l2d-1:0]   serv_o_rf_raddr;
   wire [RF_WIDTH-1:0]  serv_i_rf_rdata;
   wire		      serv_o_rf_ren; 
	
	servile #(
	.rf_width(RF_WIDTH),
   .reset_pc(reset_pc),
   .reset_strategy(reset_strategy),
   .sim(sim),
   .with_c(with_c),
   .with_csr(with_csr),
   .with_mdu(with_mdu)) 
	cpu (
   .i_clk(clk),
   .i_rst( ~ rst_n ),
   .i_timer_irq(serv_i_timer_irq),
	.o_wb_mem_adr(serv_o_wb_mem_adr),
   .o_wb_mem_dat(serv_o_wb_mem_dat),
   .o_wb_mem_sel(serv_o_wb_mem_sel),
   .o_wb_mem_we(serv_o_wb_mem_we),
   .o_wb_mem_stb(serv_o_wb_mem_stb),
   .i_wb_mem_rdt(serv_i_wb_mem_rdt),
   .i_wb_mem_ack(serv_i_wb_mem_ack),
	.o_wb_ext_adr(serv_o_wb_ext_adr),
   .o_wb_ext_dat(serv_o_wb_ext_dat),
   .o_wb_ext_sel(serv_o_wb_ext_sel),
   .o_wb_ext_we(serv_o_wb_ext_we),
   .o_wb_ext_stb(serv_o_wb_ext_stb),
   .i_wb_ext_rdt(serv_i_wb_ext_rdt),
   .i_wb_ext_ack(serv_i_wb_ext_ack),
	.o_rf_waddr(serv_o_rf_waddr),
   .o_rf_wdata(serv_o_rf_wdata),
   .o_rf_wen(serv_o_rf_wen),
   .o_rf_raddr(serv_o_rf_raddr),
   .i_rf_rdata(serv_i_rf_rdata),
   .o_rf_ren(serv_o_rf_ren));
	
		// hanging wires
	wire	[31:0]	mem_o_dbg;
	wire mem_int;
	wire mem_stall;
	
	wbi2cmaster #(.MEM_ADDR_BITS(MEM_ADDR_BITS)) mem_i2c (
		.i_clk(clk), 
		.i_reset( ~ rst_n ),
		.i_wb_cyc(1'b0), 
		.i_wb_stb(serv_o_wb_mem_stb), 
		.i_wb_we(serv_o_wb_mem_we),
		.i_wb_addr(serv_o_wb_mem_adr),
		.i_wb_data(serv_o_wb_mem_dat),
		.i_wb_sel(serv_o_wb_mem_sel),
		.o_wb_stall(mem_stall),
		.o_wb_ack(serv_i_wb_mem_ack),
		.o_wb_data(serv_i_wb_mem_rdt),
		.i_i2c_scl(mem_i_i2c_scl), 
		.i_i2c_sda(mem_i_i2c_sda),
		.o_i2c_scl(mem_o_i2c_scl), 
		.o_i2c_sda(mem_o_i2c_sda),
		.o_int(mem_int),
		.o_dbg(mem_o_dbg));
		
	// hanging wires
	wire	[31:0]	ext_o_dbg;
	wire ext_int;
	wire ext_stall;
		
	wbi2cmaster #(.MEM_ADDR_BITS(MEM_ADDR_BITS)) ext_i2c (
		.i_clk(clk), 
		.i_reset( ~ rst_n ),
		.i_wb_cyc(1'b0), 
		.i_wb_stb(serv_o_wb_ext_stb), 
		.i_wb_we(serv_o_wb_ext_we),
		.i_wb_addr(serv_o_wb_ext_adr),
		.i_wb_data(serv_o_wb_ext_dat),
		.i_wb_sel(serv_o_wb_ext_sel),
		.o_wb_stall(ext_stall),
		.o_wb_ack(serv_i_wb_ext_ack),
		.o_wb_data(serv_i_wb_ext_rdt),
		.i_i2c_scl(ext_i_i2c_scl), 
		.i_i2c_sda(ext_i_i2c_sda),
		.o_i2c_scl(ext_o_i2c_scl), 
		.o_i2c_sda(ext_o_i2c_sda),
		.o_int(ext_int),
		.o_dbg(ext_o_dbg));
	
	serv_rf_ram
     #(.width (RF_WIDTH),
       .csr_regs (0))
   rf_ram
     (.i_clk    (clk),
      .i_waddr (serv_o_rf_waddr),
      .i_wdata (serv_o_rf_wdata),
      .i_wen   (serv_o_rf_wen),
      .i_raddr (serv_o_rf_raddr),
      .i_ren    (serv_o_rf_ren),
      .o_rdata (serv_i_rf_rdata));

endmodule
