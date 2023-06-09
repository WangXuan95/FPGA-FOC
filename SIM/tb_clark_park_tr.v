
//--------------------------------------------------------------------------------------------------------
// Module  : tb_clark_park_tr
// Type    : simulation, top
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: testbench for sincos.sv , clark_tr.sv , park_tr.sv
//--------------------------------------------------------------------------------------------------------

module tb_clark_park_tr();


initial $dumpvars(1, tb_clark_park_tr);
         

reg rstn = 1'b0;
reg clk  = 1'b1;
always #(13563) clk = ~clk;   // 36.864MHz
initial begin repeat(4) @(posedge clk); rstn<=1'b1; end

reg                en_theta = 0;
reg         [11:0] theta = 0;       // 当前电角度（简记为 ψ）。取值范围0~4095。0对应0°；1024对应90°；2048对应180°；3072对应270°。

localparam  [11:0] PI_M2_D3 = (2*4096/3);     // (2/3)*π
localparam  [11:0] PI_D3    = (  4096/3);     // (1/3)*π

wire               en_iabc;
wire signed [15:0] ia, ib, ic;

wire               en_ialphabeta;
wire signed [15:0] ialpha, ibeta;

wire               en_idq;
wire signed [15:0] id;
wire signed [15:0] iq;

// 这里只是刚好借助了 sincos 模块来生成正弦波给 clark_tr ，只是为了仿真。在 FOC 设计中 sincos 模块并不是用来给 clark_tr 提供输入数据的，而是被 park_tr 调用。
sincos u1_sincos (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_theta                 ),
    .i_theta      ( theta + PI_M2_D3         ),   // input : θ + (2/3)*π
    .o_en         ( en_iabc                  ),
    .o_sin        ( ia                       ),   // output: Ia, 振幅为±16384，初相位为 (4/3)*π 的正弦波
    .o_cos        (                          )
);

sincos u2_sincos (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_theta                 ),
    .i_theta      ( theta + PI_D3            ),   // input : θ + (1/3)*π
    .o_en         (                          ),
    .o_sin        ( ib                       ),   // output: Ib, 振幅为±16384，初相位为 (2/3)*π 的正弦波
    .o_cos        (                          )
);

sincos u3_sincos (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_theta                 ),
    .i_theta      ( theta                    ),   // input : θ
    .o_en         (                          ),
    .o_sin        ( ic                       ),   // output: Ic, 振幅为±16384，初相位为 0 的正弦波
    .o_cos        (                          )
);

// clark 变换
clark_tr u_clark_tr (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .i_en         ( en_iabc                  ),
    .i_ia         ( ia / 16'sd2              ),  // input : 振幅为±8192，初相位为 (4/3)*π 的正弦波
    .i_ib         ( ib / 16'sd2              ),  // input : 振幅为±8192，初相位为 (2/3)*π 的正弦波
    .i_ic         ( ic / 16'sd2              ),  // input : 振幅为±8192，初相位为       0 的正弦波
    .o_en         ( en_ialphabeta            ),
    .o_ialpha     ( ialpha                   ),  // output: Iα ，应该为初相位为 (4/3)*π 的正弦波
    .o_ibeta      ( ibeta                    )   // output: Iβ ，相位应该比 Iα 滞后 (1/2)*π ，也就是与 Iα 正交
);

// park 变换
park_tr u_park_tr (
    .rstn         ( rstn                     ),
    .clk          ( clk                      ),
    .psi          ( theta + 12'd512          ),  // input : θ + (1/4)*π
    .i_en         ( en_ialphabeta            ),
    .i_ialpha     ( ialpha                   ),  // input : Iα
    .i_ibeta      ( ibeta                    ),  // input : Iβ
    .o_en         ( en_idq                   ),
    .o_id         ( id                       ),  // output: Id ，应该变为一个定值，因为 park 变换把转子又变回定子了
    .o_iq         ( iq                       )   // output: Iq ，应该变为一个定值，因为 park 变换把转子又变回定子了
);


integer i;

initial begin
    while(~rstn) @ (posedge clk);
    for (i=0; i<1000; i=i+1) @ (posedge clk) begin
        en_theta <= 1'b1;
        theta <= theta + 12'd10;
        @ (posedge clk);
        en_theta <= 1'b0;
        repeat (9) @ (posedge clk);
    end
    $finish;
end

endmodule
