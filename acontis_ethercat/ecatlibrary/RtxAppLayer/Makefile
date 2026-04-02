# Makefile for ecatlibrary (acontis EC-Master Linux port)
# Builds the robot control application using the acontis EC-Master SDK

# Target binary
TARGET := ecatApp

# Toolchain
CROSS_COMPILE ?=
CC       := $(CROSS_COMPILE)gcc
CXX      := $(CROSS_COMPILE)g++
STRIP    := strip

# Paths
PROJPATH      := .
ECMASTER_ROOT := ../../../steven_ecmaster/EC-Master-V3.2.2.99-Linux-x86_64Bit-Eval-Intelgbe

# Compiler flags
CFLAGS := -Wall -O0 -g -D__LINUX_GCC__ -no-pie \
          -Wno-packed-not-aligned -Wno-stringop-truncation \
          -Wno-unused-variable -Wno-unused-but-set-variable

# Include directories
INC := -I$(ECMASTER_ROOT)/SDK/INC              \
       -I$(ECMASTER_ROOT)/SDK/INC/Linux        \
       -I$(ECMASTER_ROOT)/Sources/Common       \
       -I$(PROJPATH)

# Libraries
LIB := -L$(ECMASTER_ROOT)/SDK/LIB/Linux/x64   \
       -lEcMaster -lpthread -lm -ldl -lrt

# Project source files
OBJECTS := AppLayer.o          \
           Motor.o             \
           IOModule.o          \
           IOGroup.o           \
           MySystem.o          \
           MyCallback.o        \
           Robot.o             \
           Script.o            \
           Debug.o             \
           ErrorHandler.o      \
           LogWriter.o         \
           Simulator.o         \
           CmdContext.o        \
           CmdStrategy.o       \
           ComponentMerger.o   \
           Interpolator.o      \
           Intp_S.o            \
           Intp_T.o            \
           Jog.o               \
           Kinematics.o        \
           Kin_Cobot.o         \
           Kin_Ext.o           \
           KinSerial.o         \
           MathTool.o          \
           MathTool_Robot.o    \
           HG6Cobot.o          \
           PathLoader.o        \
           PathSetter.o        \
           TrajectoryPlanner.o

# Source search paths
vpath %.cpp $(PROJPATH)

# Default target
all: $(TARGET)

# Link
$(TARGET): $(OBJECTS)
	$(CXX) -o $@ $^ $(CFLAGS) $(LIB)

# Compile
%.o: %.cpp
	$(CXX) $(CFLAGS) $(INC) -c $< -o $@

# Strip
strip: $(TARGET)
	$(STRIP) --strip-all $<

# Clean
clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: all strip clean
