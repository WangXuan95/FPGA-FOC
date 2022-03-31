del sim.out dump.vcd
iverilog  -g2005-sv  -o sim.out  tb_clark_park_tr.sv  ../RTL/foc/sincos.sv  ../RTL/foc/clark_tr.sv  ../RTL/foc/park_tr.sv
vvp -n sim.out
del sim.out
pause