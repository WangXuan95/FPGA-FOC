del sim.out dump.vcd
iverilog  -g2001  -o sim.out  tb_clark_park_tr.v  ../RTL/foc/sincos.v  ../RTL/foc/clark_tr.v  ../RTL/foc/park_tr.v
vvp -n sim.out
del sim.out
pause