// === Module: config_register ===
module config_register(
    input clk,
    input rst,
    input [191:0] cfg_data,
    input cfg_valid,
    output reg [31:0] origin_x,
    output reg [31:0] origin_y,
    output reg [31:0] length,
    output reg [31:0] width,
    output reg [31:0] sin_theta,
    output reg [31:0] cos_theta
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            origin_x <= 0;
            origin_y <= 0;
            length <= 0;
            width <= 0;
            sin_theta <= 0;
            cos_theta <= 0;
        end else if (cfg_valid) begin
            origin_x <= cfg_data[191:160];
            origin_y <= cfg_data[159:128];
            length <= cfg_data[127:96];
            width <= cfg_data[95:64];
            sin_theta <= cfg_data[63:32];
            cos_theta <= cfg_data[31:0];
        end
    end
endmodule


// === Module: agu (Address Generation Unit with 2D Rotation) ===
module agu(
    input [31:0] origin_x,
    input [31:0] origin_y,
    input [31:0] length,
    input [31:0] width,
    input [31:0] sin_theta,
    input [31:0] cos_theta,
    output reg [959:0] addr_out_flat
);
    integer i;
    reg [31:0] x, y;
    reg [31:0] rotated_x, rotated_y;
    reg [31:0] mem_addr;

    always @(*) begin
        for (i = 0; i < 30; i = i + 1) begin
            // Simple 2D transformation (fixed origin and incremental offset)
            x = origin_x + i;
            y = origin_y;
            // Rotation (cos, sin) approximated as integer operations
            rotated_x = x * cos_theta - y * sin_theta;
            rotated_y = x * sin_theta + y * cos_theta;
            // Flattened memory address using row-major order: mem[y][x] => y * row_width + x
            mem_addr = rotated_y * 32 + rotated_x;
            addr_out_flat[32*i +: 32] = mem_addr;
        end
    end
endmodule


// === Module: ru (Reduction Unit) ===
module ru(
    input [959:0] addr_in_flat,
    output reg [959:0] addr_unique_flat
);
    integer i;
    always @(*) begin
        for (i = 0; i < 30; i = i + 1) begin
            addr_unique_flat[32*i +: 32] = addr_in_flat[32*i +: 32];
        end
    end
endmodule


// === Module: l0_cache ===
module l0_cache(
    input [31:0] addr,
    output reg valid_bit
);
    reg [0:255] memory;
    integer j;
    initial begin
        for (j = 0; j < 256; j = j + 1) begin
            memory[j] = (j % 5 == 0) ? 1'b1 : 1'b0;
        end
    end

    always @(*) begin
        valid_bit = memory[addr[7:0]];
    end
endmodule


// === Module: collision_or ===
module collision_or(
    input [29:0] bits_in,
    output reg collision
);
    integer i;
    always @(*) begin
        collision = 0;
        for (i = 0; i < 30; i = i + 1) begin
            if (bits_in[i])
                collision = 1;
        end
    end
endmodule


// === Module: racod_top (Top Level Integration) ===
module racod_top(
    input clk,
    input rst,
    input [191:0] cfg_data,
    input cfg_valid,
    output collision
);
    wire [31:0] origin_x, origin_y, length, width, sin_theta, cos_theta;
    wire [959:0] agu_addr_flat;
    wire [959:0] unique_addr_flat;
    wire [29:0] bits;

    config_register CFG(
        .clk(clk), .rst(rst), .cfg_data(cfg_data), .cfg_valid(cfg_valid),
        .origin_x(origin_x), .origin_y(origin_y), .length(length), .width(width),
        .sin_theta(sin_theta), .cos_theta(cos_theta)
    );

    agu AGU(
        .origin_x(origin_x), .origin_y(origin_y), .length(length), .width(width),
        .sin_theta(sin_theta), .cos_theta(cos_theta), .addr_out_flat(agu_addr_flat)
    );

    ru RU(
        .addr_in_flat(agu_addr_flat), .addr_unique_flat(unique_addr_flat)
    );

    genvar i;
    generate
        for (i = 0; i < 30; i = i + 1) begin : CACHE_INST
            l0_cache L0(.addr(unique_addr_flat[32*i +: 32]), .valid_bit(bits[i]));
        end
    endgenerate

    collision_or ORGATE(.bits_in(bits), .collision(collision));

endmodule

