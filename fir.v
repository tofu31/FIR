module fir 
#(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)(
    // AXI-Lite Write
    output wire                     awready,    // Address write ready
    output wire                     wready,     // Data write ready
    input  wire                     awvalid,    // Address write valid
    input  wire                     wvalid,     // Data write valid
    input  wire [(pADDR_WIDTH-1):0] awaddr,     // Address write data
    input  wire [(pDATA_WIDTH-1):0] wdata,      // Data write data
    
    // AXI-Lite Read
    output wire                     arready,    // Address read ready
    input  wire                     rready,     // Data read ready
    input  wire                     arvalid,    // Address read valid
    output wire                     rvalid,     // Data read valid
    input  wire [(pADDR_WIDTH-1):0] araddr,     // Address read data
    output reg  [(pDATA_WIDTH-1):0] rdata,      // Data read data
    
    // AXI-Stream Input
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  reg                      ss_tready, 
    
    // AXI-Stream Output
    input   wire                     sm_tready, 
    output  reg                      sm_tvalid, 
    output  reg  [(pDATA_WIDTH-1):0] sm_tdata, 
    output  reg                      sm_tlast, 
    
    // BRAM for Tap RAM
    output  reg  [3:0]               tap_WE,
    output  reg                      tap_EN,
    output  reg  [(pDATA_WIDTH-1):0] tap_Di,
    output  reg  [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // BRAM for Data RAM
    output  reg  [3:0]               data_WE,
    output  reg                      data_EN,
    output  reg  [(pDATA_WIDTH-1):0] data_Di,
    output  reg  [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

localparam [1:0] 
    IDLE = 2'b00,
    WAIT = 2'b01,
    IN   = 2'b10,
    COMP = 2'b11;

localparam [2:0]
    AXI_R_IDLE = 3'b000,
    AXI_R_ADDR = 3'b010,
    AXI_R_WAIT = 3'b100, 
    AXI_R_DATA = 3'b001;

localparam [1:0]
    AXI_W_IDLE = 2'b00,
    AXI_W_ADDR = 2'b10, 
    AXI_W_DATA = 2'b01;

// State Registers
reg [1:0] state, state_n;
reg signed [31:0] Yn, Xn, Hn, Yn_n;
reg [(pADDR_WIDTH-1):0] tap_wa, tap_ra, tap_a_r;
reg last;
reg [5:0] cnt11, rst_cnt;
reg [1:0] axi_w_state, axi_w_state_n;
reg [2:0] axi_r_state, axi_r_state_n;
reg ap_start, ap_idle, ap_done;
reg ap_start_n, ap_idle_n, ap_done_n;
reg [31:0] data_length;
reg [(pADDR_WIDTH-1):0] data_wa, data_wa_n, data_ra, data_ra_n;
wire cnt_en = (state == COMP && cnt11 != 11);

//===========Block Level Signal (ap_hs)=============

// ap_start
always @(*) begin
    if (~&tap_wa && wready && wvalid && wdata == 32'h0000_0001 && state == IDLE && rst_cnt == 10)
        ap_start_n = 1'b1; 
    else if (state == IN)
        ap_start_n = 1'b0;
    else 
        ap_start_n = ap_start;
end

// ap_idle 
always @(*) begin
    if (~&tap_wa && wready && wvalid && wdata == 32'h0000_0001 && state == IDLE && rst_cnt == 10)
        ap_idle_n = 1'b0;
    else if (cnt11 == 11 && state == COMP && last)
        ap_idle_n = 1'b1;
    else 
        ap_idle_n = ap_idle;
end

// ap_done 
always @(*) begin
    if (cnt11 == 11 && state == COMP && last)
        ap_done_n = 1'b1;
    else if (state == IN)
        ap_done_n = 1'b0;
    else    
        ap_done_n = ap_done;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        ap_start    <= 1'b0;
        ap_idle     <= 1'b1;
        ap_done     <= 1'b0;
        data_length <= 32'd0;
    end else begin
        ap_start    <= ap_start_n;
        ap_idle     <= ap_idle_n;
        ap_done     <= ap_done_n;
        data_length <= (tap_wa == 12'h10 && wready && wvalid) ? wdata : data_length;
    end
end

//===========AXI-Lite Read=============

always @(*) begin
    case (axi_r_state)
        AXI_R_IDLE : axi_r_state_n = (arvalid) ? AXI_R_ADDR : AXI_R_IDLE;
        AXI_R_ADDR : axi_r_state_n = AXI_R_WAIT;
        AXI_R_WAIT : axi_r_state_n = AXI_R_DATA;
        AXI_R_DATA : axi_r_state_n = (rvalid) ? AXI_R_IDLE : AXI_R_DATA;
        default    : axi_r_state_n = AXI_R_IDLE;
    endcase
end

assign {arready, rvalid} = axi_r_state[1:0];

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        axi_r_state <= AXI_R_IDLE;
    else 
        axi_r_state <= axi_r_state_n;
end

always @(*) begin
    case (tap_ra)
        12'h00 : rdata = {ap_idle, ap_done, ap_start};
        12'h10 : rdata = data_length;
        default: rdata = tap_Do;
    endcase
end

//==========AXI-Lite Write===========

always @(*) begin
    case (axi_w_state)
        AXI_W_IDLE : axi_w_state_n = (awvalid) ? AXI_W_ADDR : AXI_W_IDLE;
        AXI_W_ADDR : axi_w_state_n = AXI_W_DATA;
        AXI_W_DATA : axi_w_state_n = (wvalid) ? AXI_W_IDLE : AXI_W_DATA;
        default    : axi_w_state_n = AXI_W_IDLE;
    endcase
end

assign {awready, wready} = axi_w_state[1:0];

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        axi_w_state <= AXI_W_IDLE;
    else 
        axi_w_state <= axi_w_state_n;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        tap_wa <= 0;
        tap_ra <= 0;
    end else begin
        tap_wa <= (awvalid && awready) ? awaddr : tap_wa;
        tap_ra <= (arvalid && arready) ? araddr : tap_ra;
    end
end

//==============StreamIn============

always @(*) begin
    ss_tready = (state == IN) ? 1 : 0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        last <= 0;
    else begin
        if (state == IDLE) 
            last <= 0;
        else if (state == IN)
            last <= ss_tlast;
        else 
            last <= last;
    end
end

// ======================= Stream-Out ===========

always @(*) begin
    if (cnt11 == 11 && state == COMP) begin
        sm_tvalid = 1;
        sm_tdata = Yn;
        sm_tlast = last;
    end else begin
        sm_tvalid = 0;
        sm_tdata = 0;
        sm_tlast = 0;
    end
end

//==============Tap RAM===============

// tap_A
always @(*) begin
    if (wready && wvalid && state != COMP)
        tap_a_r = tap_wa;
    else if (axi_r_state == AXI_R_WAIT && tap_ra != 12'h00)
        tap_a_r = tap_ra;
    else if (cnt11 < Tape_Num)
        tap_a_r = cnt11;
    else 
        tap_a_r = tap_wa;
end

// tap_WE
always @(*) begin
    tap_WE = (tap_wa != 12'h00 && wready && wvalid) ? 4'hf : 4'h0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        tap_A <= 0;
    else 
        tap_A <= tap_a_r;
end

always @(*) begin
    tap_Di = wdata;
    tap_EN = (wready && wvalid) || (axi_r_state == AXI_R_WAIT) || (state == COMP && cnt_en);
end

//=================Data RAM=================

always @(*) begin
    if (state == COMP) begin
        data_WE = (cnt11 != 11) ? 4'h0 : 4'hf;
    end else if (state == IN) begin
        data_WE = (ss_tvalid) ? 4'hf : 4'h0;
    end else begin
        data_WE = 4'h0;
    end
end

always @(*) begin
    data_EN = (ss_tvalid || cnt_en);
    data_Di = ss_tdata;
end

// data_A
always @(*) begin
    if (state == COMP && cnt11 != 11)
        data_A = cnt11;
    else if (state == IN)
        data_A = data_wa;
    else 
        data_A = data_ra;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        data_wa <= 0;
    else if (ss_tvalid)
        data_wa <= data_wa + 1;
    else 
        data_wa <= data_wa;
end

//==================State Machine============

always @(*) begin
    case (state)
        IDLE: state_n = (ap_start) ? WAIT : IDLE;
        WAIT: state_n = IN;
        IN  : state_n = (ss_tvalid) ? WAIT : COMP;
        COMP: state_n = (cnt11 == 11) ? IDLE : COMP;
        default: state_n = IDLE;
    endcase
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        state <= IDLE;
    else 
        state <= state_n;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n)
        cnt11 <= 0;
    else if (cnt_en)
        cnt11 <= cnt11 + 1;
    else 
        cnt11 <= 0;
end

always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
        Yn <= 0;
        Yn_n <= 0;
    end else if (cnt_en) begin
        Xn <= data_Do;
        Hn <= tap_Do;
        Yn_n <= Yn + Xn * Hn;
    end else begin
        Yn <= Yn_n;
    end
end

endmodule
