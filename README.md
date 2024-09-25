# Translation-Lookaside-Buffer-Simulation

This repo is a simulation of a Translation Lookaside Buffer in C and C++ using SystemC. It was created in group
project at university. To be able to run the sim locally you must install SystemC and paste the path of your 
SystemC folder into the Makefile.

A Translation Lookaside Buffer is a Cache in the Memory Management Unit of modern computers, which facilitates the
translation of virtual memory addresses into their corresponding physical memory addresses.

## File structure:
  * **main.c:** processes command line args, error handling, stores parameters, sets default values for simulation, parses csv file
  * **common.hpp:** contains important structs (Result, Request, SimulationConfig, run_simulation)
  * **modules.cpp:** configures/starts simulation
  * **modules.hpp:** contains SystemC modules (TLB, TOP, PROGRAM_MEMORY, DATA_STORAGE, CONTROL_UNIT)
    * **TOP:** constructs other modules with correct parameters, connects signals
    * **PROGRAM_MEMORY:** tracks the current instruction
    * **CONTROL_UNIT:** simulates CPU
    * **TLB:** simulates TLB (Translation Lookaside Buffer Cache)
    * **DATA_STORAGE:** simulates memory


To understand how to properly use the simulation run the following command in the terminal:
    
    ./main -h


For a valid simulation you must create a csv file with commands in the following format:
  
  * read/write, address, data
  1. read commands
     - r,0x00404080,
  3. write commands
     - w,0x00404080,49
  
  The data and address need to be in 32 bit format.


Once you create a csv file you can run a valid command such as:
  
    ./main test.csv --tlb-size 32

to run a simulation.
