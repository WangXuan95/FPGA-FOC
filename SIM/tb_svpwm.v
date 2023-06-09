
//--------------------------------------------------------------------------------------------------------
// Module  : tb_swpwm
// Type    : simulation, top
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: testbench for cartesian2polar.sv and swpwm.sv
//--------------------------------------------------------------------------------------------------------

module tb_swpwm();


initial $dumpvars(1, tb_swpwm);
initial $dumpvars(1, u_svpwm);
         

reg rstn = 1'b0;
reg clk  = 1'b1;
always #(13563) clk = ~clk;   // 36.864MHz
initial begin repeat(4) @(posedge clk); rstn<=1'b1; end


reg         [11:0] theta = 0;

wire signed [15:0] x, y;

wire        [11:0] rho;
wire        [11:0] phi;

wire pwm_en, pwm_a, pwm_b, pwm_c;


// 这里只是刚好借助了 sincos 模块来生成正弦波给 cartesian2polar ，只是为了仿真。在 FOC 设计中 sincos 模块并不是用来给 cartesian2polar 提供输入数据的，而是被 park_tr 调用。
sincos u_sincos (
    .rstn         ( rstn       ),
    .clk          ( clk        ),
    .i_en         ( 1'b1       ),
    .i_theta      ( theta      ),   // input : θ, 一个递增的角度值
    .o_en         (            ),
    .o_sin        ( y          ),   // output : y, 振幅为 ±16384 的正弦波
    .o_cos        ( x          )    // output : x, 振幅为 ±16384 的余弦波
);

cartesian2polar u_cartesian2polar (
    .rstn         ( rstn       ),
    .clk          ( clk        ),
    .i_en         ( 1'b1       ),
    .i_x          ( x / 16'sd5 ),  // input : 振幅为 ±3277 的余弦波
    .i_y          ( y / 16'sd5 ),  // input : 振幅为 ±3277 的正弦波
    .o_en         (            ),
    .o_rho        ( rho        ),  // output: ρ, 应该是一直等于或近似 3277
    .o_theta      ( phi        )   // output: φ, 应该是一个接近 θ 的角度值
);

svpwm u_svpwm (
    .rstn         ( rstn       ),
    .clk          ( clk        ),
    .v_amp        ( 9'd384     ),
    .v_rho        ( rho        ),  // input : ρ
    .v_theta      ( phi        ),  // input : φ
    .pwm_en       ( pwm_en     ),  // output
    .pwm_a        ( pwm_a      ),  // output
    .pwm_b        ( pwm_b      ),  // output
    .pwm_c        ( pwm_c      )   // output
);


integer i;

initial begin
    while(~rstn) @ (posedge clk);
    for(i=0; i<200; i=i+1) begin
        theta <= 25 * i;               // 让 θ 递增
        repeat(2048) @ (posedge clk);
        $display("%d/200", i);
    end
    $finish;
end

endmodule
