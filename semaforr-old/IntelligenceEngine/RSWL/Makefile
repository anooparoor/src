####################### File Info ############################
# 
# Description: Make file for RSWL.
#
# Author : Anoop Aroor (modified from eric and tuna's makefiles)
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
export CPP    	:= g++

PROJ_DIR	:= ../..

INSTALL_LIB_DIR	:= $(PROJ_DIR)/lib
INSTALL_BIN_DIR := $(PROJ_DIR)/bin
INSTALL_INC_DIR := $(PROJ_DIR)/include

# Dependency: SemaFORR
SEMAFORR_A	:= $(INSTALL_LIB_DIR)/libsemaFORR.a
SEMAFORR_DIR	:= $(PROJ_DIR)/IntelligenceEngine/semaFORR
SEMAFORR_LIB	:= $(INSTALL_LIB_DIR)
SEMAFORR_INC	:= $(INSTALL_INC_DIR)/semaFORR

BIN_DIR		:= bin
INC_DIR		:= include
SRC_DIR		:= src
OBJ_DIR		:= obj
LIB_DIR         := lib

EXECUTABLE	:= rswlearning
OUTPUT    	:= $(BIN_DIR)/$(EXECUTABLE)

RSWL    := RSWL
RSWL_A  := librswl.a
OUTPUT      := $(LIB_DIR)/$(RSWL_A)

VPATH 		:= $(SRC_DIR) 

# Options
INC_FLAGS	:= -I$(INC_DIR) -I$(SEMAFORR_INC) 
CPPFLAGS 	:= $(INC_FLAGS)
CXXFLAGS	:= -std=c++0x -c -g -Wall
LIBS   		:= -lpthread
LIBS            += -L$(SEMAFORR_LIB) -lsemaFORR
 
# Objects ans Sources
SOURCE_PATHS    := $(shell find $(SRC_DIR) -name "*cpp") 
SOURCES     := $(foreach source, $(SOURCE_PATHS), $(subst $(SRC_DIR), , $(source) ))
OBJS        := $(notdir $(SOURCES:.cpp=.o))
OBJECTS     := $(addprefix $(OBJ_DIR)/, $(OBJS))

# Rule for building objects
$(OBJ_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

################### Phony targets ###########################
.PHONY: all install clean distclean uninstall docs

all: $(OUTPUT) 

install: all | $(INSTALL_INC_DIR)/$(RSWL)
	cp -rf $(LIB_DIR)/* $(INSTALL_LIB_DIR)
	cp -rf $(INC_DIR)/* $(INSTALL_INC_DIR)/$(RSWL)

clean:
	rm -f $(OBJ_DIR)/*.o
	$(shell find ./ -name "*~" | xargs -r rm )

distclean: clean
	rm -f $(OUTPUT)
	rm -rf doc/*

uninstall:
	rm -rf $(INSTALL_LIB_DIR)/$(RSWL_A)
	rm -rf $(INSTALL_INC_DIR)/$(RSWL)

docs:
	@doxygen

################## Main Targets #############################

$(OUTPUT): $(OBJECTS)
	$(AR) -rc $@ $(OBJECTS)

$(INSTALL_INC_DIR)/$(RSWL): 
	mkdir $(INSTALL_INC_DIR)/$(RSWL)

################### Header Dependencies #####################

%.cpp: %.h
