#include "common.hpp"
#include <errno.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

int parseCsv(char *fileName, struct Request **requests_2ptr, size_t *numRequests_ptr, FILE **file);
void print_csv_help();

// Wrapper around parseCsv
int parseCsvWrapper(char *fileName, struct Request **requests_2ptr, size_t *numRequests_ptr) {
  FILE *file;
  int result = parseCsv(fileName, requests_2ptr, numRequests_ptr, &file);
  fclose(file);
  return result;
}

const char *usage_msg =
    "Usage: %s [options] FileName   Complete simulation using input file \"FileName\"\n"
    "   or: %s -h/--help            Show help message and exit\n\n"
    "ATTENTION: A valid input to run a simulation has following required parameters:\n"
    "\t1. a valid input has to at least include a valid csv file,\n"
    "\t2. if v2b-block-offset is specified, then blockSize must also be specified\n"
    "\t3. the rest of the parameters are not mandatory, as they will be defined\n\t   using standardized values.\n\n"
    "\tcycles, blocksize, memory-latency, tlb-latency must be >= 1\n"
    "\ttlb-size must be >= 2"
    "\tblocksize, tlb-size must be a power of 2\n"
    "\tv2b-block-offset must be 0 or a multiple of blocksize\n";

const char *help_msg =
    "\nPlease make sure to use the following permitted command line arguments:\n"
    "Positional argument:\n"
    "<File name>                        Define file name of input csv file\n"
    "\n"
    "Optional arguments [default in square braces]:\n"
    "-c <Number>/--cycles <Number>      Define number of cycles to be simulated [INT_MAX]\n"
    "--blocksize <Number>               Define blocksize of pages in bytes [4096]\n"
    "--v2b-block-offset <Number>        Define offset of physical address [blocksize]\n"
    "--tlb-size <Number>                Define size of TLB with amount of entries [32]\n"
    "--tlb-latency <Number>             Define latency of TLB in cycles [1]\n"
    "--memory-latency <Number>          Define latency of main storage in cycles [tlb-latency * 50]\n"
    "--tf=<File name>                   Define file name of trace file. Omit to disable tracing\n"
    "-h/--help                          Show help message and exit\n";

// Print usage message
void print_usage(const char *name) {
  fprintf(stderr, usage_msg, name, name, name);
}

// Print help message
void print_help(const char *name) {
  print_usage(name);
  fprintf(stderr, "\n%s\n", help_msg);

  print_csv_help();
}

int convert_to_unsigned_base(char *c, unsigned *u, int base) {
  // Convert string to unsigned int safely
  errno = 0;
  char *endptr;

  unsigned long tmp = strtoul(c, &endptr, base);

  if (endptr == c || *endptr != '\0') {
    fprintf(stderr, "Invalid number: %s could not be converted to unsigned int\n", c);
    return 1;
  } else if (errno == ERANGE || tmp > __UINT32_MAX__) {
    fprintf(stderr, "Invalid number: %s over- or underflows unsigned int\n", c);
    return 1;
  }
  *u = (unsigned)tmp;

  return 0;
}

// Enum for the result of the conversion
enum IntResult {
  Success = 0,
  InvalidBase = 1,
  InvalidRange = 2,
  InvalidChars = 3
};

/**
 * Wrapper for strtoul to convert a string to an unsigned long
 *
 * @param str_ptr Pointer to the string
 * @param end_2ptr Optional double pointer to the end of the string. If present, set on Success or InvalidChars
 * @param base Base of the number (0, 2-36)
 * @param number_ptr Optional pointer to the result. If present, set on Success
 * @return Indicating success or failure
 */
enum IntResult convert_to_unsigned_long(char *str_ptr, char **end_2ptr, int base, unsigned long *number_ptr) {
  if (str_ptr == NULL)
    fprintf(stderr, "Critical: str_ptr is NULL in convert_to_unsigned_long. Segfault expected\n");
  // If base is invalid (not 0 and outside 2-36)
  if (!(base == 0 || (base >= 2 && base <= 36)))
    return InvalidBase;

  errno = 0;
  char *end_ptr;
  unsigned long number;
  number = strtoul(str_ptr, &end_ptr, base);
  if (number == 0 && str_ptr == end_ptr)
    return InvalidChars;

  if (errno == ERANGE) {
    if (number_ptr != NULL)
      *number_ptr = number;
    return InvalidRange;
  }

  if (number_ptr != NULL)
    *number_ptr = number;
  if (end_2ptr != NULL)
    *end_2ptr = end_ptr;
  return Success;
}

int convert_to_unsigned(char *c, unsigned *u) {
  return convert_to_unsigned_base(c, u, 10);
}

int convert_to_int(char *c, int *i) {
  // Convert string to int safely
  errno = 0;
  char *end;

  long tmp = strtol(c, &end, 10);

  if (end == c || *end != '\0' || tmp <= 0) {
    fprintf(stderr, "Invalid number: %s could not be converted to int\n", c);
    return 1;
  } else if (errno == ERANGE || tmp > __INT_MAX__) {
    fprintf(stderr, "Invalid number: %s over- or underflows int\n", c);
    return 1;
  }
  *i = (int)tmp;

  return 0;
}
int check_valid_file(const char *filename) {
  // Check if file from command line is valid
  FILE *file = fopen(filename, "r");
  if (file) {
    fclose(file);
    return 0; // File valid and can be opened
  } else {
    return 1; // File invalid and cannot be opened
  }
}
// Function to get log base 2 of an unsigned int
unsigned log2_func(unsigned value) {
    unsigned log = 0;
    // Shift right, check if value stil != 0
    while (value >>= 1) {
        log++;
    }
    return log;
}

int main(int argc, char *argv[]) {
  // Check for invalid case (no arguments)
  const char *name = argv[0];
  if (argc == 1) {
    print_usage(name);
    return EXIT_FAILURE;
  }

  // Definition struct für getopt_long
  static struct option long_options[] = {
      {"cycles", required_argument, 0, 'c'},
      {"help", no_argument, 0, 'h'},
      {"blocksize", required_argument, 0, 'b'},
      {"v2b-block-offset", required_argument, 0, 'v'},
      {"tlb-size", required_argument, 0, 't'},
      {"tlb-latency", required_argument, 0, 's'},
      {"memory-latency", required_argument, 0, 'm'},
      {"tf=", required_argument, 0, 'f'},
      {0, 0, 0, 0}};

  int opt_nums;
  // Definition variables
  int cycles = 0;
  unsigned tlbSize = 0;
  unsigned tlbLatency = 0;
  unsigned blocksize = 0;
  unsigned v2bBlockOffset = 0;
  unsigned memoryLatency = 0;
  struct Request *requests_ptr = NULL;
  size_t numRequests2 = 0;
  char *tracefile = 0;
  char *fileName = NULL;

  // Booleans for each var to check if it is set
  bool set_cycles = false;
  bool set_tlbSize = false;
  bool set_tlbLatency = false;
  bool set_blocksize = false;
  bool set_v2bBlockOffset = false;
  bool set_memoryLatency = false;

  // Parsing optional args
  while ((opt_nums = getopt_long(argc, argv, "c:b:v:t:s:m:f:h", long_options, 0)) != -1) {
    switch (opt_nums) {
    case 'c': // Case cycles
      if (convert_to_int(optarg, &cycles) != 0) {
        fprintf(stderr, "There was an Error parsing cycles...\n");
        return EXIT_FAILURE;
      }
      set_cycles = true;
      break;
    case 'b': // Case blockSize
      if (convert_to_unsigned(optarg, &blocksize) != 0) {
        fprintf(stderr, "There was an Error parsing blocksize...\n");
        return EXIT_FAILURE;
      }
      set_blocksize = true;
      break;
    case 'v': // Case v2bBlockOffset
      if (convert_to_unsigned(optarg, &v2bBlockOffset) != 0) {
        fprintf(stderr, "There was an Error parsing v2b-Block-Offset...\n");
        return EXIT_FAILURE;
      }
      set_v2bBlockOffset = true;
      break;
    case 's': // Case tlbLatency
      if (convert_to_unsigned(optarg, &tlbLatency) != 0) {
        fprintf(stderr, "There was an Error parsing tlb-Latency...\n");
        return EXIT_FAILURE;
      }
      set_tlbLatency = true;
      break;
    case 'm': // Case memoryLatency
      if (convert_to_unsigned(optarg, &memoryLatency) != 0) {
        fprintf(stderr, "There was an Error parsing memory-Latency...\n");
        return EXIT_FAILURE;
      }
      set_memoryLatency = true;
      break;
    case 't': // Case tlbSize
      if (convert_to_unsigned(optarg, &tlbSize) != 0) {
        fprintf(stderr, "There was an Error parsing tlb-Size...\n");
        return EXIT_FAILURE;
      }
      set_tlbSize = true;
      break;
    case 'f': // Case tracefile
      if (optarg == NULL || *optarg == '\0') {
        fprintf(stderr, "Invalid tracefile name\n");
        return EXIT_FAILURE;
      }
      unsigned len_str = strlen(optarg);
      tracefile = malloc(len_str + 1);
      if (tracefile == NULL) {
        fprintf(stderr, "Error allocating memory for tracefile\n");
        return EXIT_FAILURE;
      }
      memset(tracefile, 0, len_str + 1);
      strncpy(tracefile, optarg, len_str);
      break;
    case 'h': // Case help
      print_help(name);
      free(tracefile);
      return EXIT_SUCCESS;
    default:
      print_usage(name);
      return EXIT_FAILURE;
    }
  }

  // Check if positional argument for file name is present
  if (optind == argc) {
    fprintf(stderr, "Missing argument for file name...\n");
    print_usage(name);
    return EXIT_FAILURE;
  }
  // Check if there are too many positional args
  if (argc - optind > 1) {
    fprintf(stderr, "Too many positional arguments...\n");
    print_usage(name);
    return EXIT_FAILURE;
  }
  // Check for valid file name
  if (argv[optind] == NULL || *argv[optind] == '\0') {
    fprintf(stderr, "Invalid file name\n");
    return EXIT_FAILURE;
  }

  // Allocate memory and set filename
  unsigned len_str = strlen(argv[optind]);
  // Allocate space for string + '\0'
  fileName = malloc(len_str + 1);
  // Check file memory allocation worked
  if (fileName == NULL) {
    fprintf(stderr, "Invalid file\n");
    free(tracefile);
    return EXIT_FAILURE;
  }
  strncpy(fileName, argv[optind], len_str);
  // Null terminate string
  fileName[len_str] = '\0';


  // Check whether filename is a valid file
  if (check_valid_file(fileName) == 1) {
    fprintf(stderr, "Invalid file name.\n");
    free(tracefile);
    free(fileName);
    print_usage(name);
    return EXIT_FAILURE;
  }

  // Set default tlbSize
  if (!set_tlbSize) {
    tlbSize = 32;
    set_tlbSize = true;
  }

  // Check arguments that have to be provided by user
  if (set_v2bBlockOffset && !set_blocksize) {
    fprintf(stderr, "Request can not be processed because v2bBlockOffset\n" 
                    "is set without blockSize being set.\n");
    free(fileName);
    free(tracefile);
    print_usage(name);
    return EXIT_FAILURE;
  }
  // If v2b and blockSize have not been set
  if (!set_v2bBlockOffset && !set_blocksize) {
    // Set v2b and blocksize to standards
    blocksize = 4096;
    v2bBlockOffset = blocksize;
    set_v2bBlockOffset = set_blocksize = true;
  }
  // If v2b has not been set
  else if (!set_v2bBlockOffset) {
    v2bBlockOffset = blocksize;
    set_v2bBlockOffset = true;
  }
  // Check requirement blockSize and tlbSize -> log2(blockSize) + log2(tlbSize) = offsetbits + indexbits <= 32
  if (log2_func(blocksize) + log2_func(tlbSize) > 32) {
    fprintf(stderr, "Request can not be processed because relation between\n" 
                    "blockSize and tlbSize is invalid.\n");
    free(fileName);
    free(tracefile);
    print_usage(name);
    return EXIT_FAILURE;
  }
  // Set cycles if not set
  if (!set_cycles) {
    cycles = INT_MAX - 1;
    set_cycles = true;
  }
  // Set latencies if not set
  if (!set_tlbLatency && !set_memoryLatency) {
    tlbLatency = 1;
    memoryLatency = 50;
    set_tlbLatency = set_memoryLatency = true;
  }
  // Set tlbLatency if not set
  else if (!set_tlbLatency) {
    tlbLatency = 1;
    set_tlbLatency = true;
  }
  // Set memoryLatency if not set
  else if (!set_memoryLatency) {
    if (0xFFFFFFFF / 50 > tlbLatency) {
      memoryLatency = tlbLatency * 50;
    }
    else {
      memoryLatency = 0xFFFFFFFF;
    }
    set_memoryLatency = true;
  }

  // Check if filename can be successfully parsed
  if (parseCsvWrapper(fileName, &requests_ptr, &numRequests2) != 0) {
    fprintf(stderr, "Error parsing CSV file.\n");
    free(fileName);
    free(tracefile);
    free(requests_ptr);
    print_usage(name);
    return EXIT_FAILURE;
  }

  // Check if blocksize/tlbsize/v2b fit the mathematical requirements
  if ((blocksize & (blocksize - 1)) != 0 || (tlbSize & (tlbSize - 1)) != 0 || (v2bBlockOffset % blocksize) != 0) {
    fprintf(stderr, "One or more args of (blocksize, tlbSize, v2bBlockOffset) are invalid, \n"
                    "please check the requirements in the usage/help message.\n");
    free(fileName);
    free(tracefile);
    free(requests_ptr);
    print_usage(name);
    return EXIT_FAILURE;
  }
  // Check if all parameters fit the rest of the requirements
  if (cycles > 0 && tlbSize > 1 && tlbLatency > 0 && blocksize > 0 && v2bBlockOffset >= 0 && memoryLatency > 0) {
    printf("Running with Parameters:\n");
    printf(" Input FileName %s\n", fileName);
    printf(" tf %s\n", tracefile);
    printf(" cycles %d\n", cycles);
    printf(" tlbSize %u\n", tlbSize);
    printf(" tlbLatency %u\n", tlbLatency);
    printf(" blocksize %u\n", blocksize);
    printf(" v2bBlockOffset %u\n", v2bBlockOffset);
    printf(" memoryLatency %u\n", memoryLatency);

    // If they all fit the requirements, run simulation
    printf("\nSimulating instructions:\n");

    for (size_t i = 0; i < numRequests2; i++) {
      struct Request *request = &requests_ptr[i];
      printf(" %zu: %s %u %u\n", i, request->we ? "W" : "R", request->addr, request->data);
    }
    struct Result sim_result = run_simulation(cycles, tlbSize, tlbLatency, blocksize, v2bBlockOffset, memoryLatency, numRequests2, requests_ptr, tracefile);
    printf("\nSimulation Result:\n");
    if (sim_result.cycles == SIZE_MAX)
      printf(" simulation did not finish within cycle limit.\n");
    else
      printf(" simulation ran to completion.\n cycles: %zu\n", sim_result.cycles);
    printf(" hits: %zu\n misses: %zu\n gates: %zu\n", sim_result.hits, sim_result.misses, sim_result.primitiveGateCount);
  }
  // If they don't fit requirements -> FAIL
  else {
    fprintf(stderr, "One or more args are invalid, please check the requirements in the usage/help message.\n");
    free(fileName);
    free(tracefile);
    free(requests_ptr);
    print_usage(name);
    return EXIT_FAILURE;
  }

  // Free allocated memory
  free(tracefile);
  free(fileName);
  free(requests_ptr);
  return 0;
}


// Prints detailed error information for csv parsing errors
// Marks the unreadable part
void print_csv_err(char *fileName, char *line_str, int line_number, int cursor, char *error_msg) {
  unsigned long len = strlen(line_str);
  bool newline = false;
  if (line_str[len - 1] == '\n') {
    line_str[len - 1] = '\0';
    newline = true;
  }
  fprintf(stderr, "Error in \"%s:%i:%i\":\n%s\n%*s^\n%s\n\n", fileName, line_number, (cursor + 1), line_str, cursor, "",
          error_msg);
  print_csv_help();
  if (newline)
    line_str[len - 1] = '\n';
}


int parseCsv(char *fileName, struct Request **requests_2ptr, size_t *numRequests_ptr, FILE **file_2ptr) {
  // open file
  *file_2ptr = fopen(fileName, "r");
  FILE *file = *file_2ptr;
  if (file == NULL) {
    fprintf(stderr, "Error opening file \"%s\":\n", fileName);
    perror(NULL);
    return 1;
  }

  // read line
  char line_str[1000];
  int line_number = 0;
  while (true) {
    line_number++;
    char *result = fgets(line_str, 1000, file);
    if (result == NULL) {
      if (feof(file))
        return 0;
      fprintf(stderr, "Error reading file \"%s\":\n", fileName);
      perror(NULL);
      return 1;
    }

    // handle too long lines
    size_t len = strlen(line_str);
    if (line_str[len - 1] != '\n' && !feof(file)) {
      fprintf(stderr, "Line too long in file \"%s:%i\":\n%s\n", fileName, line_number, line_str);
      print_csv_help();
      return 1;
    }

    // ignore last line if empty
    if (len == 0 && feof(file))
      return 0;

    // Instruction variables
    int we = 0;
    uint32_t addr = 0;
    uint32_t data = 0;

    // Read instruction type
    char instr_type = 0;
    int n = -1; // Number of recently read chars
    int cursor = 0; // Current read progress
    int read_cnt = 0; // Number of read params by sscannf
    read_cnt = sscanf(&line_str[0], " %c %n", &instr_type, &n);
    if (read_cnt == 0 || n <= 0) {
      print_csv_err(fileName, line_str, line_number, cursor, "Invalid instruction type");
      return 1;
    }

    switch (instr_type) {
    case 'w':
    case 'W':
      we = 1;
      break;
    case 'r':
    case 'R':
      we = 0;
      break;
    default:
      print_csv_err(fileName, line_str, line_number, cursor, "Invalid instruction type");
      return 1;
    }

    // Read comma
    cursor += n;
    n = -1;
    read_cnt = sscanf(&line_str[cursor], ", %n", &n);
    if (n <= 0) {
      print_csv_err(fileName, line_str, line_number, cursor, "Missing comma after instruction type");
      return 1;
    }

    // Read address
    cursor += n;
    n = -1;
    char number[1050];
    char x;
    int base = 0;

    read_cnt = sscanf(&line_str[cursor], "0%c%[0123456789ABCDEFabcdef] %n", &x, number, &n);
    if (n <= 0 || read_cnt == 0 || (x != 'X' && x != 'x')) {
      n = -1;
      read_cnt = sscanf(&line_str[cursor], "%[0123456789] %n", number, &n);
      if (n <= 0 || read_cnt == 0) {
        print_csv_err(fileName, line_str, line_number, cursor, "Missing valid address");
        return 1;
      }
      base = 10;
    } else {
      base = 16;
    }

    unsigned long convertedNumber;
    int conversionResult = convert_to_unsigned_long(number, NULL, base, &convertedNumber);
    if (conversionResult == InvalidChars) {
      print_csv_err(fileName, line_str, line_number, cursor, "Invalid characters for address");
      return 1;
    } else if (conversionResult == InvalidRange || convertedNumber > UINT32_MAX) {
      print_csv_err(fileName, line_str, line_number, cursor, "Invalid size for address");
      return 1;
    }

    addr = convertedNumber;

    // Read comma
    cursor += n;
    n = -1;
    read_cnt = sscanf(&line_str[cursor], ", %n", &n);
    if (n <= 0) {
      print_csv_err(fileName, line_str, line_number, cursor, "Missing comma after address");
      return 1;
    }

    convertedNumber = 0;
    cursor += n;
    if (we) {
      // Read data if write enabled
      n = -1;
      read_cnt = sscanf(&line_str[cursor], "0%c%[0123456789ABCDEFabcdef] %n", &x, number, &n);
      if (n <= 0 || read_cnt == 0 || (x != 'X' && x != 'x')) {
        n = -1;
        read_cnt = sscanf(&line_str[cursor], "%[0123456789] %n", number, &n);
        if (n <= 0 || read_cnt == 0) {
          print_csv_err(fileName, line_str, line_number, cursor, "Missing valid data value");
          return 1;
        }
      } else {
        base = 10;
      }

      conversionResult = convert_to_unsigned_long(number, NULL, base, &convertedNumber);
      if (conversionResult == InvalidChars) {
        print_csv_err(fileName, line_str, line_number, cursor, "Invalid characters for data value");
        return 1;
      } else if (conversionResult == InvalidRange || convertedNumber > UINT32_MAX) {
        print_csv_err(fileName, line_str, line_number, cursor, "Invalid size for data value");
        return 1;
      }
      cursor += n;
    } // If instr_type is 'R'

    // Ensure that end of line reached
    if (cursor != len) {
      print_csv_err(fileName, line_str, line_number, cursor, "Expected end of line");
      return 1;
    }

    data = convertedNumber;

    // Add new instruction to the list
    if (*numRequests_ptr == 0) {
      *requests_2ptr = malloc(sizeof(struct Request));
      *numRequests_ptr = 1;
    } else {
      *numRequests_ptr = *numRequests_ptr + 1;
      struct Request *new_ptr = realloc(*requests_2ptr, sizeof(struct Request) * (*numRequests_ptr));
      if (new_ptr == NULL) {
        fprintf(stderr, "Error reallocating memory for requests\n");
        return 1;
      }
      *requests_2ptr = new_ptr;
    }

    struct Request *request = *requests_2ptr + (*numRequests_ptr - 1);
    request->we = we;
    request->addr = addr;
    request->data = data;
  }
}
void print_csv_help() {
  fprintf(stderr,
          "Expected format for each line in the csv file:\n"
          "R,<Address>,\n"
          "or \n"
          "W,<Address>,<Data>\n"
          "Where:\n"
          "'R' indicates a read instruction and 'W' a write instruction.\n"
          "Address indicates the address to read to or write from respectively\n"
          "Both are stored as a 32 bit unsigned integer in decimal or, if prefixed with 0x, in hexadecimal notation\n"
          "The maximum line length is 1000 characters\n\n");
}
