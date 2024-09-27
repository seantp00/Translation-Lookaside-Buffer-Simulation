# MAKEFILE BASED ON TUTOR EXERCISE
# ---------------------------------------
# CONFIGURATION BEGIN
# ---------------------------------------

# entry point for the program and target name
C_SRCS = src/main.c
CPP_SRCS = src/modules.cpp

# Object files
C_OBJS = $(patsubst src/%.c,obj/%.o,$(C_SRCS))
CPP_OBJS = $(patsubst src/%.cpp,obj/%.o,$(CPP_SRCS))


# assignment task file
HEADERS := src/modules.hpp src/common.hpp

# target name
TARGET := main

# Path to your systemc installation
SYSTEMC_HOME ?= ../systemc

# Additional flags for the compiler
CXXFLAGS := -std=c++14  -I$(SYSTEMC_HOME)/include -L$(SYSTEMC_HOME)/lib -lsystemc -lm
#CXXFLAGS := -std=c++14  -I$(SYSTEMC_HOME)/include -L$(SYSTEMC_HOME)/lib -lsystemc -lm -fsanitize=address -fsanitize=undefined -fsanitize=leak -fsanitize=pointer-compare -fsanitize=pointer-subtract
#CXXFLAGS := -std=c++14  -I$(SCPATH)/include -L$(SCPATH)/lib -lm -Wall -Wextra #-lsystemc

# ---------------------------------------
# CONFIGURATION END
# ---------------------------------------

# Determine if clang or gcc is available
CXX := $(shell command -v g++ || command -v clang++)
ifeq ($(strip $(CXX)),)
    $(error Neither clang++ nor g++ is available. Exiting.)
endif

CC := $(shell command -v gcc || command -v clang)
ifeq ($(strip $(CC)),)
    $(error Neither clang nor g is available. Exiting.)
endif

# Add rpath except for MacOS
UNAME_S := $(shell uname -s)

ifneq ($(UNAME_S), Darwin)
    CXXFLAGS += -Wl,-rpath=$(SYSTEMC_HOME)/lib
endif


# Default to release build for both app and library
all: debug

# Rule to compile .c files to .o files
obj/%.o: src/%.c $(HEADERS)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

# Rule to compile .cpp files to .o files
obj/%.o: src/%.cpp $(HEADERS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Debug build
debug: CXXFLAGS += -g
debug: CFLAGS += -g
debug: $(TARGET)

# Release build
release: CXXFLAGS += -O2
release: $(TARGET)

# Rule to link object files to executable
$(TARGET): $(C_OBJS) $(CPP_OBJS)
	$(CXX) $(CXXFLAGS) $(C_OBJS) $(CPP_OBJS) $(LDFLAGS) -o $(TARGET)

# clean up
clean:
	rm -f $(TARGET)
	rm -rf obj/

.PHONY: all debug release clean