#include "modules.hpp"
#include "common.hpp"
#include "systemc.h"

// TLB direct mapped. Same equation as in readme
size_t calc_gates(unsigned tlbSize, unsigned blockSize) {
  unsigned int s = tlbSize;
  long double n32 = 32;
  // number of non-tag bits
  //unsigned int a = (n32 - log2l(blockSize) - log2l(s));
  //return (-3 - 3 * a + 10 * s + 20 * a * s) - 9 * s * log(s);
  // number of non-tag bits
  unsigned int a = n32 - log2l(blockSize);
  return (-3 - 3 * a + 10 * s + 20 * a * s) - 9 * s * log2(s);
}

extern "C" struct Result run_simulation(
    int cycles,               // Number cycles to simulate
    unsigned tlbSize,         // Size TLB in Entries
    unsigned tlbLatency,      // Latency TLB
    unsigned blocksize,       // Size block in Byte
    unsigned v2bBlockOffset,  // Offset virtual address to physical address
    unsigned memoryLatency,   // Latency main storage
    size_t numRequests,       // Number requests
    struct Request *requests, // Array of Requests
    const char *tracefile     // Filename Tracefile (null if no tracefile)
) {

  // Initialize and connects signals and data
  sc_clock clk("in_clk", sc_time(1, SC_SEC));
  SimulationConfig config = {
      .cycles = cycles,
      .tlbSize = tlbSize,
      .tlbLatency = tlbLatency,
      .blocksize = blocksize,
      .v2bBlockOffset = v2bBlockOffset,
      .memoryLatency = memoryLatency,
      .numRequests = numRequests,
      .requests = requests,
      .tracefile = tracefile};

  sc_trace_file *trace = nullptr;

  TOP top("top", &config);
  top.in_clk.bind(clk);

  // setup tracefile
  if (tracefile != nullptr) {
    trace = sc_create_vcd_trace_file(tracefile);
    if (trace == nullptr) {
      std::cerr << "Tracefile " << tracefile << " could not be created";
      throw;
    }

    top.addTraces(trace);
    trace->set_time_unit(1, SC_NS);
  }

  //sc_start(SC_ZERO_TIME);
  sc_start(cycles + 1, SC_SEC);
  sc_stop();

  if (trace != nullptr)
    sc_close_vcd_trace_file(trace);

  std::cout << "(debug) completed: " << top._prog_complete.read() << " runtime: " << sc_time_stamp() << std::endl;


  size_t cycles_res = top._prog_complete.read() ? sc_min(top.cycles - 1, top.cycles) : SIZE_MAX;


  Result result{
      .cycles = top._prog_complete.read() ? cycles_res : SIZE_MAX,
      .misses = top.misses,
      .hits = top.hits,
      .primitiveGateCount = calc_gates(tlbSize, blocksize)
  };

  return result;
}



int sc_main(int argc, char *argv[]) {
  std::cout << "This entrypoint must not be called\n"
            << std::endl;
  return 1;
}
