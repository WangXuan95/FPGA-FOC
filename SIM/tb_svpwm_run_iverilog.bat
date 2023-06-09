del sim.out dump.vcd
iverilog  -g2001  -o sim.out  tb_svpwm.v  ../RTL/foc/sincos.v  ../RTL/foc/cartesian2polar.v  ../RTL/foc/svpwm.v
vvp -n sim.out
del sim.out
pause