####################### File Info ############################
# 
# Description: Make file for Ariadne/FORR.
#
# Author : Eric Schneider 
#	(extended from Tuna Ozgelen's make files)
# 
# Usage : 
#   > make 
#       will build all 
#   > make install
#       will build all and copy the libraries, include 
#       files and the executable to the project dir's
#       lib, include and bin folders respectively
#   > make clean 
#       will clean all *.o files and the executable
#   > make distclean 
#       will clean and remove all libraries and docs from 
#       local lib and doc dirs respectively
#   > make uninstall
#       will remove all libraries, include
#       files and the executable (produced by this module)
#       from project dir's lib, include and bin folders. 
#   > make docs
#       will build an html and a latex version of into  
#       doc, using doxygen
#

# Toolchain/Environment
export SHELL    := /bin/bash
export CPP      := g++
export AR   := ar

PROJ_DIR    := ../..

INSTALL_LIB_DIR := $(PROJ_DIR)/lib
INSTALL_BIN_DIR := $(PROJ_DIR)/bin
INSTALL_INC_DIR := $(PROJ_DIR)/include

LIB_DIR     := lib
INC_DIR     := include
SRC_DIR     := src
OBJ_DIR     := obj

SEMAFORR    := semaFORR
SEMAFORR_A  := libsemaFORR.a
OUTPUT      := $(INSTALL_LIB_DIR)/$(SEMAFORR_A)

# Dependency: ParticleFilter
PFILTER_A	:= $(INSTALL_LIB_DIR)/libParticleFilter.a
PFILTER_DIR	:= $(PROJ_DIR)/Utils/ParticleFilter
PFILTER_LIB	:= $(INSTALL_LIB_DIR)
PFILTER_INC	:= $(INSTALL_INC_DIR)/ParticleFilter

# Dependency: utils semaFORR

USEMAFORR_A	:= $(INSTALL_LIB_DIR)/libsemaFORR.a
USEMAFORR_DIR	:= $(PROJ_DIR)/Utils/semaFORR
USEMAFORR_LIB	:= $(INSTALL_LIB_DIR)
USEMAFORR_INC	:= $(INSTALL_INC_DIR)/semaFORR

VPATH       := $(SRC_DIR) 

# Options
CPPFLAGS    := -I$(INC_DIR) -I$(PFILTER_INC) -I$(USEMAFORR_INC)
CXXFLAGS    := -c -g -Wall -std=c++0x

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

install: all | $(INSTALL_INC_DIR)/$(SEMAFORR)
	cp -rf $(INSTALL_LIB_DIR)/* $(LIB_DIR)
	cp -rf $(INC_DIR)/* $(INSTALL_INC_DIR)/$(SEMAFORR)

clean:
	rm -f $(OBJ_DIR)/*.o
	$(shell find ./ -name "*~" | xargs -r rm )

distclean: clean
	rm -f $(OUTPUT)
	rm -rf doc/*

uninstall:
	rm -rf $(INSTALL_LIB_DIR)/$(SEMAFORR_A)
	rm -rf $(INSTALL_INC_DIR)/$(SEMAFORR)

docs:
	@doxygen

################## Main Targets #############################

$(OUTPUT): $(OBJECTS)
	$(AR) -rc $@ $(OBJECTS)

$(INSTALL_INC_DIR)/$(SEMAFORR): 
	mkdir $(INSTALL_INC_DIR)/$(SEMAFORR)

################### Header Dependencies #####################

%.cpp: %.h

