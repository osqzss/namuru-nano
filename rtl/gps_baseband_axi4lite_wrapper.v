// gps_baseband_axi4lite_wrapper_fixed4d.v
// AXI4-Lite wrapper for gps_baseband (Icarus-friendly, deterministic)
// Fixes stale read_data issues by inserting an extra wait cycle between issuing
// internal read/write strobes and sampling/acknowledging the result.
// - IDLE: READY asserted
// - ISSUE: assert chip_select+rd_stb/wr_stb for one cycle (registered)
// - WAIT: allow gps_baseband to process strobe at the next posedge
// - CAP/RESP: capture read_data and respond; hold VALID until READY

`timescale 1ns/1ps

module gps_baseband_axi4lite_wrapper #(
  parameter integer AXI_ADDR_WIDTH = 32,
  parameter integer AXI_DATA_WIDTH = 32,
  parameter integer ADDR_LSB       = 2
) (
  input  wire                        aclk,
  input  wire                        aresetn,

  input  wire                        sign_in,
  input  wire                        mag_in,
  output wire                        accum_int,

  input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,
  input  wire                        s_axi_awvalid,
  output reg                         s_axi_awready,

  input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata,
  input  wire [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,
  input  wire                        s_axi_wvalid,
  output reg                         s_axi_wready,

  output reg  [1:0]                  s_axi_bresp,
  output reg                         s_axi_bvalid,
  input  wire                        s_axi_bready,

  input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_araddr,
  input  wire                        s_axi_arvalid,
  output reg                         s_axi_arready,

  output reg  [AXI_DATA_WIDTH-1:0]   s_axi_rdata,
  output reg  [1:0]                  s_axi_rresp,
  output reg                         s_axi_rvalid,
  input  wire                        s_axi_rready
);

  // Internal bus to gps_baseband
  reg         chip_select;
  reg         wr_stb;
  reg         rd_stb;
  reg  [7:0]  av_address;
  reg  [31:0] av_wdata;
  wire [31:0] av_rdata;

  gps_baseband u_baseband (
    .clk         (aclk),
    .hw_rstn     (aresetn),
    .sign_in     (sign_in),
    .mag_in      (mag_in),
    .chip_select (chip_select),
    .write       (wr_stb),
    .read        (rd_stb),
    .address     (av_address),
    .write_data  (av_wdata),
    .read_data   (av_rdata),
    .accum_int   (accum_int)
  );

  wire [7:0] aw_word = s_axi_awaddr[ADDR_LSB + 7 : ADDR_LSB];
  wire [7:0] ar_word = s_axi_araddr[ADDR_LSB + 7 : ADDR_LSB];

  // WRITE FSM
  localparam [1:0] WR_IDLE  = 2'd0;
  localparam [1:0] WR_ISSUE = 2'd1;
  localparam [1:0] WR_WAIT  = 2'd2;
  localparam [1:0] WR_RESP  = 2'd3;

  reg [1:0]  wr_state;
  reg [7:0]  wr_addr_lat;
  reg [31:0] wr_data_lat;

  // READ FSM
  localparam [2:0] RD_IDLE  = 3'd0;
  localparam [2:0] RD_ISSUE = 3'd1;
  localparam [2:0] RD_WAIT  = 3'd2;
  localparam [2:0] RD_CAP   = 3'd3;
  localparam [2:0] RD_RESP  = 3'd4;

  reg [2:0]  rd_state;
  reg [7:0]  rd_addr_lat;
  reg [31:0] rd_data_lat;

  always @(posedge aclk) begin
    if (!aresetn) begin
      s_axi_awready <= 1'b0;
      s_axi_wready  <= 1'b0;
      s_axi_bresp   <= 2'b00;
      s_axi_bvalid  <= 1'b0;

      s_axi_arready <= 1'b0;
      s_axi_rdata   <= 32'h0;
      s_axi_rresp   <= 2'b00;
      s_axi_rvalid  <= 1'b0;

      chip_select   <= 1'b0;
      wr_stb        <= 1'b0;
      rd_stb        <= 1'b0;
      av_address    <= 8'h00;
      av_wdata      <= 32'h0;

      wr_state      <= WR_IDLE;
      wr_addr_lat   <= 8'h00;
      wr_data_lat   <= 32'h0;

      rd_state      <= RD_IDLE;
      rd_addr_lat   <= 8'h00;
      rd_data_lat   <= 32'h0;
    end else begin
      // Default: deassert internal strobes
      chip_select <= 1'b0;
      wr_stb      <= 1'b0;
      rd_stb      <= 1'b0;

      // Defaults for responses (held in RESP states)
      s_axi_bresp  <= 2'b00;
      s_axi_rresp  <= 2'b00;

      // ----------------------------
      // WRITE FSM
      // ----------------------------
      case (wr_state)
        WR_IDLE: begin
          // Keep READY asserted while idle
          s_axi_awready <= 1'b1;
          s_axi_wready  <= 1'b1;

          if (s_axi_awvalid && s_axi_wvalid) begin
            wr_addr_lat <= aw_word;
            wr_data_lat <= s_axi_wdata;
            wr_state    <= WR_ISSUE;
          end

          // No response while idle
          s_axi_bvalid <= 1'b0;
        end

        WR_ISSUE: begin
          // Deassert READY while busy
          s_axi_awready <= 1'b0;
          s_axi_wready  <= 1'b0;

          // Assert internal write strobe for one registered cycle
          chip_select <= 1'b1;
          wr_stb      <= 1'b1;
          av_address  <= wr_addr_lat;
          av_wdata    <= wr_data_lat;

          // Wait one more cycle so gps_baseband can commit the write
          wr_state    <= WR_WAIT;
          s_axi_bvalid<= 1'b0;
        end

        WR_WAIT: begin
          s_axi_awready <= 1'b0;
          s_axi_wready  <= 1'b0;
          // Now we can raise BVALID
          s_axi_bvalid  <= 1'b1;
          wr_state      <= WR_RESP;
        end

        WR_RESP: begin
          s_axi_awready <= 1'b0;
          s_axi_wready  <= 1'b0;
          s_axi_bvalid  <= 1'b1;
          if (s_axi_bready) begin
            wr_state <= WR_IDLE;
            // BVALID will be cleared in WR_IDLE
          end
        end

        default: wr_state <= WR_IDLE;
      endcase

      // ----------------------------
      // READ FSM
      // ----------------------------
      case (rd_state)
        RD_IDLE: begin
          // Keep ARREADY asserted while idle
          s_axi_arready <= 1'b1;
          s_axi_rvalid  <= 1'b0;

          if (s_axi_arvalid) begin
            rd_addr_lat <= ar_word;
            rd_state    <= RD_ISSUE;
          end
        end

        RD_ISSUE: begin
          s_axi_arready <= 1'b0;
          // Assert internal read strobe for one registered cycle
          chip_select <= 1'b1;
          rd_stb      <= 1'b1;
          av_address  <= rd_addr_lat;
          // Allow gps_baseband to see rd_stb at the *next* posedge
          rd_state    <= RD_WAIT;
          s_axi_rvalid <= 1'b0;
        end

        RD_WAIT: begin
          s_axi_arready <= 1'b0;
          // gps_baseband processes the rd_stb from previous cycle at this posedge.
          // Capture data in the following state.
          rd_state <= RD_CAP;
          s_axi_rvalid <= 1'b0;
        end

        RD_CAP: begin
          s_axi_arready <= 1'b0;
          // Now av_rdata is stable (updated by gps_baseband in the previous cycle's posedge).
          rd_data_lat <= av_rdata;
          rd_state    <= RD_RESP;
          s_axi_rvalid <= 1'b0;
        end

        RD_RESP: begin
          s_axi_arready <= 1'b0;
          s_axi_rvalid  <= 1'b1;
          s_axi_rdata   <= rd_data_lat;
          if (s_axi_rready) begin
            rd_state <= RD_IDLE;
            // RVALID will be cleared in RD_IDLE
          end
        end

        default: rd_state <= RD_IDLE;
      endcase

    end
  end

endmodule