echo SYSTEMC_HOME: $SYSTEMC_HOME
make clean
make
./main test.csv --tlb-size 32