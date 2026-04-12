// ============================================================================
// pia6821_sv.sv — MC6821 PIA, SystemVerilog implementation
//
// Drop-in replacement for pia6821.vhd on the Qix audio board (sndPIA1).
// Quartus does not honor reset/initial values in the VHDL, which leaves
// portb_ddr stuck at $00 and kills the DAC output.  This SV version uses
// standard always_ff with synchronous reset that Quartus does respect.
//
// Interface is identical to pia6821.vhd.
// All registers reset to $00 — the CPU init sequence sets them correctly.
//
// Timing model (matching pia6821.vhd):
//   Write registers  : posedge clk, gated by cs & ~rw
//   IRQ / edge flags : negedge clk  (tracks real 6821 E-clock edge)
//   Read mux         : combinatorial
//   Port direction   : combinatorial
// ============================================================================

module pia6821_sv (
    input        clk,
    input        rst,
    input        cs,
    input        rw,           // 1 = read, 0 = write
    input  [1:0] addr,
    input  [7:0] data_in,
    output logic [7:0] data_out,
    output logic irqa,
    output logic irqb,

    input  [7:0] pa_i,
    output logic [7:0] pa_o,
    output logic [7:0] pa_oe,
    input        ca1,
    input        ca2_i,
    output logic ca2_o,
    output logic ca2_oe,

    input  [7:0] pb_i,
    output logic [7:0] pb_o,
    output logic [7:0] pb_oe,
    input        cb1,
    input        cb2_i,
    output logic cb2_o,
    output logic cb2_oe
);

// ---------------------------------------------------------------------------
// Registers — all cleared on reset, CPU init writes the correct values
// ---------------------------------------------------------------------------
logic [7:0] porta_ddr,  porta_data;
logic [5:0] porta_ctrl;
logic [7:0] portb_ddr,  portb_data;
logic [5:0] portb_ctrl;

// IRQ flags
logic irqa1, irqa2, irqb1, irqb2;

// Edge-detect pipeline (negedge-registered)
logic ca1_del, ca1_rise_r, ca1_fall_r;
logic ca2_del, ca2_rise_r, ca2_fall_r;
logic cb1_del, cb1_rise_r, cb1_fall_r;
logic cb2_del, cb2_rise_r, cb2_fall_r;

// CA2 / CB2 output state
logic ca2_out_r, cb2_out_r;

// portb_write pulse: captured on posedge, consumed on following negedge
logic portb_write_r;

// ---------------------------------------------------------------------------
// Combinatorial edge selects (one-negedge delay, matches pia6821.vhd)
// ---------------------------------------------------------------------------
wire ca1_edge = porta_ctrl[1] ? ca1_rise_r : ca1_fall_r;
wire ca2_edge = porta_ctrl[4] ? ca2_rise_r : ca2_fall_r;
wire cb1_edge = portb_ctrl[1] ? cb1_rise_r : cb1_fall_r;
wire cb2_edge = portb_ctrl[4] ? cb2_rise_r : cb2_fall_r;

// ---------------------------------------------------------------------------
// Read side-effect strobes (combinatorial — no rw check, matches pia6821.vhd)
// ---------------------------------------------------------------------------
wire porta_read = cs & (addr == 2'b00) & porta_ctrl[2];
wire portb_read = cs & (addr == 2'b10) & portb_ctrl[2];

// ---------------------------------------------------------------------------
// Write process — posedge clk
// ---------------------------------------------------------------------------
always_ff @(posedge clk) begin
    if (rst) begin
        porta_ddr     <= 8'h00;
        porta_data    <= 8'h00;
        porta_ctrl    <= 6'h00;
        portb_ddr     <= 8'h00;
        portb_data    <= 8'h00;
        portb_ctrl    <= 6'h00;
        portb_write_r <= 1'b0;
    end else begin
        // Capture whether port B data was written this cycle (for CB2 handshake)
        portb_write_r <= cs & ~rw & (addr == 2'b10) & portb_ctrl[2];

        if (cs && !rw) begin
            case (addr)
                2'b00: if (!porta_ctrl[2]) porta_ddr  <= data_in;
                       else                porta_data <= data_in;
                2'b01: porta_ctrl <= data_in[5:0];
                2'b10: if (!portb_ctrl[2]) portb_ddr  <= data_in;
                       else                portb_data <= data_in;
                2'b11: portb_ctrl <= data_in[5:0];
            endcase
        end
    end
end

// ---------------------------------------------------------------------------
// CA1 edge detect + irqa1 flag — negedge clk
// ---------------------------------------------------------------------------
always_ff @(negedge clk) begin
    if (rst) begin
        ca1_del <= 0; ca1_rise_r <= 0; ca1_fall_r <= 0; irqa1 <= 0;
    end else begin
        ca1_del    <= ca1;
        ca1_rise_r <= ca1  & ~ca1_del;
        ca1_fall_r <= ~ca1 & ca1_del;
        if      (ca1_edge)   irqa1 <= 1'b1;
        else if (porta_read) irqa1 <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// CA2 edge detect + irqa2 flag + CA2 output — negedge clk
// ---------------------------------------------------------------------------
always_ff @(negedge clk) begin
    if (rst) begin
        ca2_del <= 0; ca2_rise_r <= 0; ca2_fall_r <= 0;
        irqa2 <= 0; ca2_out_r <= 0;
    end else begin
        ca2_del    <= ca2_i;
        ca2_rise_r <= ca2_i  & ~ca2_del;
        ca2_fall_r <= ~ca2_i & ca2_del;

        // irqa2 active only when CA2 is input (ctrl[5]=0) and IRQ enabled (ctrl[3]=1)
        if      (~porta_ctrl[5] & porta_ctrl[3] & ca2_edge) irqa2 <= 1'b1;
        else if (porta_read)                                  irqa2 <= 1'b0;

        // CA2 output modes — only update when CA2 is output (ctrl[5]=1)
        if (porta_ctrl[5]) begin
            case (porta_ctrl[5:3])
                3'b100: begin           // read PA clears, CA1 edge sets
                    if      (porta_read) ca2_out_r <= 1'b0;
                    else if (ca1_edge)   ca2_out_r <= 1'b1;
                end
                3'b101: ca2_out_r <= ~porta_read;   // pulse low on PA read
                3'b110: ca2_out_r <= 1'b0;          // force low
                3'b111: ca2_out_r <= 1'b1;          // force high
                default: ;                           // retain (unreachable when ctrl[5]=1)
            endcase
        end
    end
end

// ---------------------------------------------------------------------------
// CB1 edge detect + irqb1 flag — negedge clk
// ---------------------------------------------------------------------------
always_ff @(negedge clk) begin
    if (rst) begin
        cb1_del <= 0; cb1_rise_r <= 0; cb1_fall_r <= 0; irqb1 <= 0;
    end else begin
        cb1_del    <= cb1;
        cb1_rise_r <= cb1  & ~cb1_del;
        cb1_fall_r <= ~cb1 & cb1_del;
        if      (cb1_edge)   irqb1 <= 1'b1;
        else if (portb_read) irqb1 <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// CB2 edge detect + irqb2 flag + CB2 output — negedge clk
// ---------------------------------------------------------------------------
always_ff @(negedge clk) begin
    if (rst) begin
        cb2_del <= 0; cb2_rise_r <= 0; cb2_fall_r <= 0;
        irqb2 <= 0; cb2_out_r <= 0;
    end else begin
        cb2_del    <= cb2_i;
        cb2_rise_r <= cb2_i  & ~cb2_del;
        cb2_fall_r <= ~cb2_i & cb2_del;

        // irqb2 active only when CB2 is input (ctrl[5]=0) and IRQ enabled (ctrl[3]=1)
        if      (~portb_ctrl[5] & portb_ctrl[3] & cb2_edge) irqb2 <= 1'b1;
        else if (portb_read)                                  irqb2 <= 1'b0;

        // CB2 output modes — only update when CB2 is output (ctrl[5]=1)
        if (portb_ctrl[5]) begin
            case (portb_ctrl[5:3])
                3'b100: begin           // write PB clears, CB1 edge sets
                    if      (portb_write_r) cb2_out_r <= 1'b0;
                    else if (cb1_edge)      cb2_out_r <= 1'b1;
                end
                3'b101: cb2_out_r <= ~portb_write_r;  // pulse low on PB write
                3'b110: cb2_out_r <= 1'b0;            // force low
                3'b111: cb2_out_r <= 1'b1;            // force high
                default: ;
            endcase
        end
    end
end

// ---------------------------------------------------------------------------
// Port direction outputs (combinatorial)
// ---------------------------------------------------------------------------
assign pa_o  = porta_ddr & porta_data;
assign pa_oe = porta_ddr;
assign pb_o  = portb_ddr & portb_data;
assign pb_oe = portb_ddr;

// ---------------------------------------------------------------------------
// Read mux (combinatorial)
// ---------------------------------------------------------------------------
always_comb begin
    case (addr)
        2'b00: data_out = porta_ctrl[2]
                        ? (porta_ddr & porta_data) | (~porta_ddr & pa_i)
                        : porta_ddr;
        2'b01: data_out = {irqa1, irqa2, porta_ctrl};
        2'b10: data_out = portb_ctrl[2]
                        ? (portb_ddr & portb_data) | (~portb_ddr & pb_i)
                        : portb_ddr;
        2'b11: data_out = {irqb1, irqb2, portb_ctrl};
        default: data_out = 8'hff;
    endcase
end

// ---------------------------------------------------------------------------
// CA2 / CB2 direction outputs (combinatorial)
// ---------------------------------------------------------------------------
assign ca2_o  = porta_ctrl[5] ? ca2_out_r : 1'b0;
assign ca2_oe = porta_ctrl[5];
assign cb2_o  = portb_ctrl[5] ? cb2_out_r : 1'b0;
assign cb2_oe = portb_ctrl[5];

// ---------------------------------------------------------------------------
// IRQ outputs (combinatorial)
// ---------------------------------------------------------------------------
assign irqa = (irqa1 & porta_ctrl[0]) | (irqa2 & porta_ctrl[3]);
assign irqb = (irqb1 & portb_ctrl[0]) | (irqb2 & portb_ctrl[3]);

endmodule
