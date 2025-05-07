// === Module: racod_tb ===
module racod_tb;
    reg clk, rst, cfg_valid;
    reg [191:0] cfg_data;
    wire collision;

    // Instantiate CODAcc top module
    racod_top DUT (
        .clk(clk),
        .rst(rst),
        .cfg_data(cfg_data),
        .cfg_valid(cfg_valid),
        .collision(collision)
    );

    initial begin
        $dumpfile("racod_tb.vcd");
        $dumpvars(0, racod_tb);

        // Initialize signals
        clk = 0; rst = 1; cfg_valid = 0; cfg_data = 0;
        #10 rst = 0;

        // Send configuration (origin_x=1, origin_y=2, len=3, width=4, sin=5, cos=6)
        cfg_data = {
            32'd6,   // cos_theta
            32'd5,   // sin_theta
            32'd4,   // width
            32'd3,   // length
            32'd2,   // origin_y
            32'd1    // origin_x
        };
        cfg_valid = 1;
        #10 cfg_valid = 0;

        // Wait to observe effects
        #100;

        $finish;
    end

    // Clock generator
    always #5 clk = ~clk;
endmodule
