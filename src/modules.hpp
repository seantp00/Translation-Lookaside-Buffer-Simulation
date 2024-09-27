#pragma once

#include "common.hpp"
#include <systemc.h>
#include <unordered_map>

// https://forums.accellera.org/topic/6296-making-a-port-optional/
template <typename T>
using sc_in_opt =
    sc_core::sc_port<sc_signal_in_if<T>, 1, SC_ZERO_OR_MORE_BOUND>;
template <typename T>
using sc_inout_opt =
    sc_core::sc_port<sc_signal_inout_if<T>, 1, SC_ZERO_OR_MORE_BOUND>;
template <typename T>
using sc_out_opt =
    sc_core::sc_port<sc_signal_inout_if<T>, 1, SC_ZERO_OR_MORE_BOUND>;

// Modules
SC_MODULE(TLB) {
  sc_in_clk in_clk;
  sc_in<uint32_t> in_virt;
  sc_in<uint32_t> in_phys;
  sc_in<bool> in_WE;
  sc_out<uint32_t> out_phys;
  sc_out<bool> out_isHit;

  struct TLB_Entry {
    uint32_t virt_addr;
    uint32_t phys_addr;
    bool valid;
  };

  unsigned tlb_size;
  TLB_Entry *tlb_entries;

  unsigned blocksize;

  SC_HAS_PROCESS(TLB);

  TLB(sc_module_name name, unsigned tlb_size, unsigned blocksize) {
    this->blocksize = blocksize;
    this->tlb_size = tlb_size;

    tlb_entries = new TLB_Entry[tlb_size]();

    for (unsigned i = 0; i < tlb_size; i++) {
      tlb_entries[i].virt_addr = 0;
      tlb_entries[i].phys_addr = 0;
      tlb_entries[i].valid = false;
    }

    SC_THREAD(behavior);
    sensitive << in_virt << in_clk.pos();
  };

  ~TLB() { delete[] tlb_entries; }

  void behavior() {
    int offset_bits = std::log2l(blocksize);
    int index_bits = std::log2l(tlb_size);
    while (true) {
      uint32_t idx = (in_virt >> offset_bits) % tlb_size;
      TLB_Entry *entry_ptr = &tlb_entries[idx];

      uint32_t in_tag = in_virt.read() >> (offset_bits + index_bits);

      if (in_clk.posedge() && in_WE.read()) {
        // store the tag
        entry_ptr->virt_addr = in_tag;
        // store the beginning of the page
        entry_ptr->phys_addr = in_phys.read() >> offset_bits;
        // mark entry as valid
        entry_ptr->valid = true;
      }

      // sets the output tag
      uint32_t entry_tag = entry_ptr->virt_addr;
      out_isHit.write(entry_tag == in_tag && entry_ptr->valid);
      out_phys.write((entry_ptr->phys_addr << offset_bits) +
                     in_virt.read() % blocksize);

      /*
      std::cout << "addr: " << in_virt << " idx: " << idx
                << " in_tag: " << in_tag << " entry_tag: " << entry_tag
                << std::endl;
*/
      wait();
    }
  }
};

SC_MODULE(Control_Unit) {
  sc_in<bool> in_clk;
  // we from instruction
  sc_in<bool> in_program_we;
  // virtual addr from addr (for calculation)
  sc_in<uint32_t> in_program_addr;

  // tlb hit
  sc_in<bool> in_tlb_hit;

  sc_out<bool> out_program_next;
  // addr for use case tlb missed
  sc_out<uint32_t> out_physical_addr_calc;
  // we for tlb
  sc_out<bool> out_tlb_we;
  // we for data storage
  sc_out<bool> out_data_we;
  sc_in<bool> in_program_complete;

  unsigned tlbLatency;
  unsigned memoryLatency;
  unsigned v2bBlockOffset;

  // Needs to be defined when using custom constructor
  SC_HAS_PROCESS(Control_Unit);

  Control_Unit(sc_module_name name, unsigned tlbLatency, unsigned memoryLatency,
               unsigned v2bBlockOffset)
      : tlbLatency(tlbLatency),

        memoryLatency(memoryLatency), v2bBlockOffset(v2bBlockOffset) {
    SC_THREAD(behavior)
    sensitive << in_clk.pos();

    SC_THREAD(calc_phys);
    sensitive << in_program_addr;
  }

  void behavior() {
    wait();
    while (true) {
      if (in_program_complete.read()) {
        sc_stop();
        wait();
        continue;
      }
      // simulate tlb latency
      for (unsigned i = 0; i < tlbLatency; ++i) {
        wait();
      }

      // if tlb did not hit, calculate physical address
      if (!in_tlb_hit.read()) {
        // simulate memory latency for fetching address
        for (unsigned i = 0; i < memoryLatency; ++i)
          wait();
      }

      // simulate memory latency for reading/writing data by waiting one less
      // than memoryLatency
      for (unsigned i = 1; i < memoryLatency; ++i)
        wait();

      // instruct tlb to write data if tlb did not hid
      out_tlb_we.write(!in_tlb_hit.read());
      // tell data storage if we are reading or writing
      out_data_we.write(in_program_we.read());
      out_program_next.write(true);
      wait();
      wait(SC_ZERO_TIME);

      auto complete = in_program_complete.read();
      auto addr = in_program_addr.read();
      out_program_next.write(false);
      out_data_we.write(false);
      out_tlb_we.write(false);
    }
  }

  void calc_phys() {
    while (true) {
      out_physical_addr_calc.write(in_program_addr.read() + v2bBlockOffset);
      wait();
    }
  }
};

SC_MODULE(DATA_STORAGE) {
  sc_in_clk in_clk;
  sc_in<uint32_t> inPhys;
  sc_in<uint32_t> in_CalcPhys;
  // If true, use inPhys. Else use in_CalcPhys;
  sc_in<bool> in_tlbHit;

  sc_in<uint32_t> inData;
  sc_in<bool> inWR;
  sc_out<uint32_t> outData;

  unsigned blockSize;
  std::unordered_map<uint32_t, uint8_t *> storage;

  SC_HAS_PROCESS(DATA_STORAGE);

  DATA_STORAGE(sc_module_name name, unsigned blockSize)
      : sc_module(name), blockSize(blockSize) {
    SC_THREAD(behavior);
    sensitive << in_clk.pos() << in_tlbHit << inWR;
  }

  ~DATA_STORAGE() {
    for (auto &pair : storage) {
      delete[] pair.second;
    }
  }

  // Check for new input from CONTROL_UNIT
  void behavior() {

    while (true) {

      uint32_t physicalAddress_ = in_tlbHit->read() ? inPhys->read() : in_CalcPhys->read();
      uint32_t data = inData->read();
      int readWrite = inWR->read() && in_clk.posedge();

      uint32_t blockIndex = physicalAddress_ / blockSize; // Calculate block index
      uint32_t offsetSize = physicalAddress_ % blockSize; // Calculate byte offset in block
      uint8_t *block_data = nullptr;

      // Case Memory access is larger than the memory for blockSizes under 4 bytes
      if ((blockSize == 1 && blockIndex > (0xFFFFFFFF - 3))) {
        outData->write(0);
      }
      else if ((blockSize == 2 && blockIndex == 0xFFFFFFFF) || (blockSize == 2 && blockIndex == (0xFFFFFFFF - 1) && offsetSize != 0)) {
        outData->write(0);
      }
      // Case Memory access is larger than the memory for blockSizes >= 4 bytes
      else if (blockIndex == (0xFFFFFFFF / blockSize) && (blockSize - offsetSize < 4)) {
        outData->write(0);
      }
      else {
        if (readWrite) { // Case write
        // Check if block to be written is in storage
          if (storage.find(blockIndex) == storage.end()) {
            // If not initialize new block with all 0s
            storage[blockIndex] = new uint8_t[blockSize]();
          }
          // Get current block
          block_data = storage.at(blockIndex);
          // If write only in 1 block, do so
          if (blockSize - offsetSize >= 4) {
            int n = 3;
            for (int i = 0; i < 4; i++) {
              // Correctly shift each byte into storage
              block_data[offsetSize + i] = (data >> (8 * n)) & 0xFF;
              --n;
            }
          } else {
            // If writes accross multiple blocks
            int amountBytesWritten = 0;
            int n = 3;
            while (amountBytesWritten < 4) {
              // Calculate current block
              int currentBlock = blockIndex + (offsetSize + amountBytesWritten) / blockSize;
              // Calculate current offset
              int currentOffset = (offsetSize + amountBytesWritten) % blockSize;
              if (storage.find(currentBlock) == storage.end()) {
                // If block to be written doesn't exist in storage, create it and initialize with all 0s
                storage[currentBlock] = new uint8_t[blockSize]();
              }
              block_data = storage.at(currentBlock);
              // Correctly shift byte of data into storage
              block_data[currentOffset] = (data >> (8 * n)) & 0xFF;
              --n;
              ++amountBytesWritten;
            }
          }
          // outData->write(0);
        }
        // Read Step
        // Check if data to be read in storage
        if (storage.find(blockIndex) != storage.end()) {
          block_data = storage.at(blockIndex);
          // If all 4 bytes to be read are in current block (no need to read
          // from adjacent blocks)
          if (blockSize - offsetSize >= 4) {
            uint32_t data = 0;
            int n = 3;
            // Read data from all blocks that are involved
            for (int i = 0; i < 4; i++) {
              // correctly shift byte into data
              data |= block_data[offsetSize + i] << (8 * n);
              --n; // Decrement shift index into data
            }
            outData->write(data);
          } else {
            // If data has to be read from multiple blocks
            uint32_t data = 0;
            int amountBytesRead = 0;
            int n = 3;
            while (amountBytesRead < 4) {
              // Calculate current block and current offset within that block
              int currentBlock = blockIndex + (offsetSize + amountBytesRead) / blockSize;
              int currentOffset = (offsetSize + amountBytesRead) % blockSize;
              // Check if block exists in storage
              if (storage.find(currentBlock) == storage.end()) {
                // If it doesn't exist -> read fails -> data = 0
                data = 0;
                break;
              }
              // Correctly shift byte into data
              block_data = storage.at(currentBlock);
              data |= block_data[currentOffset] << (8 * n);
              --n;
              ++amountBytesRead;
            }
            outData->write(data);
          }
        } else {
          // If data to be read not in storage data = 0
          outData->write(0);
        }
      }
      wait();
    }
  }
};

// Emits the current instruction
SC_MODULE(PROGRAM_MEMORY) {
  sc_in_clk in_clk;
  // Request to emit the next instruction for the next cycle
  sc_in<bool> in_program_next;
  // Instruction:
  sc_out<bool> out_program_we;
  sc_out<uint32_t> out_program_addr;
  sc_out<uint32_t> out_program_data;
  // Signals, that there are no more instructions.
  // When this is, the current "instruction" should be ignored.
  sc_out<bool> out_program_complete;

  Request *_requests;
  size_t _request_size;
  // Internal counter for current instruction
  size_t _current = 0;

  SC_HAS_PROCESS(PROGRAM_MEMORY);

  PROGRAM_MEMORY(sc_module_name name, Request * request, size_t request_size) {
    this->_request_size = request_size;
    this->_requests = request;

    SC_THREAD(behavior);
    sensitive << in_clk.pos();
  };

  void behavior() {
    while (true) {
      // std::cout << "program: " << _current << " next: " << in_program_next.read() << std::endl;
      // Increases the current instruction, if in_program_next is set on positive clock edge
      if (in_clk.posedge() && in_program_next.read()) {
        if (_current < _request_size)
          _current++;
      }

      // Outputs the current or last instruction
      if (_current < _request_size) {
        out_program_we.write(_requests[_current].we);
        out_program_addr.write(_requests[_current].addr);
        out_program_data.write(_requests[_current].data);
      } else {
        out_program_complete.write(true);
      }
      wait();
    }
  }
};

// Contains and connects all Subcircuits
SC_MODULE(TOP) {
  sc_in_clk in_clk;

  TLB tlb;
  Control_Unit control_unit;
  DATA_STORAGE data_storage;
  PROGRAM_MEMORY program_memory;

  // Intermediate signals
  sc_signal<bool> _prog_complete;
  sc_signal<uint32_t> _data_read;
  sc_signal<uint32_t> _prog_virt_addr;
  sc_signal<uint32_t> _prog_phys_calc_addr;
  sc_signal<uint32_t> _prog_phys_tlb_addr;
  sc_signal<uint32_t> _prog_data;
  sc_signal<bool> _prog_instr_we;
  sc_signal<bool> _tlb_hit;
  sc_signal<bool> _tlb_we;
  sc_signal<bool> _data_we;
  sc_signal<bool> _program_next;

  size_t hits = 0;
  size_t misses = 0;
  size_t cycles = 0;

  SC_HAS_PROCESS(TOP);

  TOP(sc_module_name name, SimulationConfig * config)
      : control_unit("control", config->tlbLatency, config->memoryLatency,
                     config->v2bBlockOffset),
        data_storage("data_storage", config->blocksize),
        program_memory("program_memory", config->requests, config->numRequests),
        tlb("tlb", config->tlbSize, config->blocksize) {

    tlb.in_phys.bind(_prog_phys_calc_addr);
    tlb.in_virt.bind(_prog_virt_addr);
    tlb.in_WE.bind(_tlb_we);
    tlb.in_clk.bind(in_clk);

    tlb.out_isHit.bind(_tlb_hit);
    tlb.out_phys.bind(_prog_phys_tlb_addr);

    program_memory.in_clk.bind(in_clk);
    program_memory.in_program_next.bind(_program_next);

    program_memory.out_program_addr.bind(_prog_virt_addr);
    program_memory.out_program_complete.bind(_prog_complete);
    program_memory.out_program_data.bind(_prog_data);
    program_memory.out_program_we.bind(_prog_instr_we);

    control_unit.in_clk.bind(in_clk);
    control_unit.in_program_we.bind(_prog_instr_we);
    control_unit.in_program_addr.bind(_prog_virt_addr);
    control_unit.in_tlb_hit.bind(_tlb_hit);
    control_unit.in_program_complete.bind(_prog_complete);

    control_unit.out_data_we.bind(_data_we);
    control_unit.out_physical_addr_calc(_prog_phys_calc_addr);
    control_unit.out_program_next.bind(_program_next);
    control_unit.out_tlb_we.bind(_tlb_we);

    data_storage.inWR.bind(_data_we);
    data_storage.inPhys.bind(_prog_phys_tlb_addr);
    data_storage.inData.bind(_prog_data);
    data_storage.in_CalcPhys.bind(_prog_phys_calc_addr);
    data_storage.in_tlbHit.bind(_tlb_hit);

    data_storage.in_clk.bind(in_clk);
    data_storage.outData.bind(_data_read);

    SC_THREAD(logging)
    sensitive << _program_next << in_clk.pos();
  }

  void logging() {
    while (true) {
      if (in_clk.posedge() && _program_next.read()) {
        // Count hits and misses while the instruction is valid
        if (_prog_complete.read() == 0) {
          hits += _tlb_hit.read() == 1;
          misses += _tlb_hit.read() == 0;
        }
        // Logs state
        std::cout << "#" << hits + misses << ": " << (_prog_instr_we ? "W, " : "R, ") << std::hex << "0x" << _prog_virt_addr << ", 0x" << _prog_data << "; Hit: " << _tlb_hit.read() << ", data_read: 0x" << _data_read  << ", program_complete: " << _prog_complete.read() << std::dec << std::endl;
      }
      // Count cycles
      if (in_clk.posedge() && !_prog_complete.read()) {
        cycles++;
      }
      wait();
    }
  }

  void addTraces(sc_trace_file * trace) {
    // Adds all signals. No duplicates
    sc_trace(trace, _prog_complete, "prog_complete");
    sc_trace(trace, _data_read, "data_read");
    sc_trace(trace, _prog_virt_addr, "prog_virt_addr");
    sc_trace(trace, _prog_phys_calc_addr, "prog_phys_calc_addr");
    sc_trace(trace, _prog_phys_tlb_addr, "prog_phys_tlb_addr");
    sc_trace(trace, _prog_data, "prog_data");
    sc_trace(trace, _prog_instr_we, "prog_instr_we");
    sc_trace(trace, _tlb_hit, "tlb_hit");
    sc_trace(trace, _tlb_we, "tlb_we");
    sc_trace(trace, _data_we, "data_we");
    sc_trace(trace, _program_next, "program_next");
    sc_trace(trace, in_clk, "clk");
  }
};
