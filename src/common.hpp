#pragma once

#include "stdint.h"
#include "stddef.h"

struct Result {
    size_t cycles; // Number cycles to process Requests
    size_t misses; // Number misses
    size_t hits; // Number hits
    size_t primitiveGateCount; // Approximate amount ghates to implement TLB
};

struct Request {
    uint32_t addr; // Virtual address
    uint32_t data; // Data to write
    int we; // 0 = Read, 1 = Write
};

struct SimulationConfig {
    int cycles;
    unsigned tlbSize;
    unsigned tlbLatency;
    unsigned blocksize;
    unsigned v2bBlockOffset;
    unsigned memoryLatency;
    size_t numRequests;
    struct Request *requests;
    const char *tracefile;
};



#ifndef __cplusplus

extern struct Result run_simulation(
        int cycles, unsigned tlbSize, unsigned tlbLatency, unsigned blocksize,
        unsigned v2bBlockOffset, unsigned memoryLatency, size_t numRequests,
        struct Request *requests, const char *tracefile
);


#endif
