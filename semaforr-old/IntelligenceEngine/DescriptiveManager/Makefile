####################### File Info ############################
#
# Description: Make file for DescriptiveManager
#
# Author : Anoop Aroor
#	(extended from Tuna's make files)
#
# Usage :
#	> make
#		will build all
#	> make install
#		will build all and copy the executable
#		to the project dir's bin folder.
#	> make clean
#		will clean all *.o files and the executable
#	> make distclean
#		will clean and remove docs
#	> make uninstall
#		will remove the executable
#		(produced by this module) from project dir's bin folder.
#	> make docs
#		will build an html and a latex version of into
#		doc, using doxygen
#

# Toolchain/Environment
export SHELL 	:= /bin/bash
#export CPP    	:= clang
export CPP    	:= g++

PROJ_DIR	:= ../..

INSTALL_LIB_DIR	:= $(PROJ_DIR)/lib
INSTALL_BIN_DIR := $(PROJ_DIR)/bin
INSTALL_INC_DIR := $(PROJ_DIR)/include

BIN_DIR		:= bin
INC_DIR		:= include
SRC_DIR		:= src
OBJ_DIR		:= obj

IE		:= IntelligenceEngine
DM		:= DescriptiveManager
EXECUTABLE	:= $(DM)
OUTPUT    	:= $(BIN_DIR)/$(EXECUTABLE)

VPATH 		:= $(SRC_DIR)

# Dependency: Metrotimer
METROTIMER_A    := $(INSTALL_LIB_DIR)/libMetroTimer.a
METROTIMER_DIR	:= $(PROJ_DIR)/Utils/MetroTimer
METROTIMER_LIB 	:= $(INSTALL_LIB_DIR)
METROTIMER_INC 	:= $(INSTALL_INC_DIR)/MetroTimer

# Dependency: MetroCommunication
METROCOMMUNICATION_A    := $(INSTALL_LIB_DIR)/libMetroCommunication.a
METROCOMMUNICATION_DIR	:= $(PROJ_DIR)/Utils/MetroCommunication
METROCOMMUNICATION_LIB 	:= $(INSTALL_LIB_DIR)
METROCOMMUNICATION_INC 	:= $(INSTALL_INC_DIR)/MetroCommunication

# Dependency: SemaFORR
SEMAFORR_A	:= $(INSTALL_LIB_DIR)/libsemaFORR.a
SEMAFORR_DIR	:= $(PROJ_DIR)/IntelligenceEngine/semaFORR
SEMAFORR_LIB	:= $(INSTALL_LIB_DIR)
SEMAFORR_INC	:= $(INSTALL_INC_DIR)/semaFORR

# Dependency: ParticleFilter
PFILTER_A	:= $(INSTALL_LIB_DIR)/libParticleFilter.a
PFILTER_DIR	:= $(PROJ_DIR)/Utils/ParticleFilter
PFILTER_LIB	:= $(INSTALL_LIB_DIR)
PFILTER_INC	:= $(INSTALL_INC_DIR)/ParticleFilter


# Dependency: RSWL
RSWL_A	        := $(INSTALL_LIB_DIR)/librswl.a
RSWL_DIR	:= $(PROJ_DIR)/IntelligenceEngine/RSWL
RSWL_LIB	:= $(INSTALL_LIB_DIR)
RSWL_INC	:= $(INSTALL_INC_DIR)/RSWL


# Dependency: Ariadne
#ARIADNE_A	:= $(INSTALL_LIB_DIR)/libAriadne.a
#ARIADNE_DIR	:= $(PROJ_DIR)/IntelligenceEngine/Ariadne
#ARIADNE_LIB	:= $(INSTALL_LIB_DIR)
#ARIADNE_INC	:= $(INSTALL_INC_DIR)/Ariadne

# Options
INC_FLAGS	:= -I$(INC_DIR) -I$(METROCOMMUNICATION_INC) -I$(METROTIMER_INC) -I$(SEMAFORR_INC) -I$(PFILTER_INC) -I$(RSWL_INC) #-I$(ARIADNE_INC)
CPPFLAGS 	:= $(INC_FLAGS)
CXXFLAGS	:= -std=c++0x -c -g -Wall
LIBS   		:= -g -lboost_system-mt -lboost_thread-mt -lglut -lGLU
LIBS		+= -L$(METROCOMMUNICATION_LIB) -lMetroCommunication
LIBS		+= -L$(METROTIMER_LIB) -lMetroTimer
LIBS            += -L$(SEMAFORR_LIB) -lsemaFORR
LIBS            += -L$(PFILTER_LIB) -lParticleFilter
LIBS            += -L$(RSWL_LIB) -lrswl

# Objects ans Sources
SOURCE_PATHS 	:= $(shell find $(SRC_DIR) -name "*cpp")
SOURCES		:= $(foreach source, $(SOURCE_PATHS), $(subst $(SRC_DIR)/, , $(source) ))
OBJS	 	:= $(notdir $(SOURCES:.cpp=.o))
OBJECTS		:= $(addprefix $(OBJ_DIR)/, $(OBJS))

# Rule for building objects
$(OBJ_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

################### Phony targets ###########################
.PHONY: all install clean distclean uninstall docs

all: $(OUTPUT)

install: all
	cp $(OUTPUT) $(INSTALL_BIN_DIR)

clean:
	rm -f $(OUTPUT)
	rm -f $(OBJ_DIR)/*.o
	$(shell find ./ -name "*~" | xargs -r rm )

distclean: clean
	rm -rf doc/*

uninstall :
	rm -f $(INSTALL_BIN_DIR)/$(EXECUTABLE)

docs:
	@doxygen

################## Main Targets #############################

# build camera controller
$(OUTPUT): $(OBJECTS)
	$(CPP) -o $@ $(OBJECTS) $(LIBS)

################### Header Dependencies #####################

#%.cpp: %.h

rswl: rswl.cpp main.cpp
	gcc -o rswl rswl.cpp main.cpp
