del sim.out dump.vcd
iverilog  -g2005-sv  -o sim.out  tb_svpwm.sv  ../RTL/foc/sincos.sv  ../RTL/foc/cartesian2polar.sv  ../RTL/foc/svpwm.sv
vvp -n sim.out
del sim.out
pause