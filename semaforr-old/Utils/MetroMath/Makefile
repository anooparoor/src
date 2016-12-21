####################### File Info ############################
# 
# Description: Make file for MetroMath.
#
# Author : A. Tuna Ozgelen 
#	(extended from George Rabanca 
#	and  Mark Manashirov's make files)
# 
# Usage : 
#	> make 
#		will build all 
#	> make install
#		will build all and copy the libraries, include 
#		files and the executable to the project dir's
#		lib, include and bin folders respectively
#	> make clean 
#		will clean all *.o files and the executable
#	> make distclean 
#		will clean and remove all libraries and docs from 
#		local lib and doc dirs respectively
#	> make uninstall
#		will remove all libraries, include
#		files and the executable (produced by this module)
#		from project dir's lib, include and bin folders. 
#	> make docs
#		will build an html and a latex version of into  
#		doc, using doxygen
#

# Toolchain/Environment
export SHELL 	:= /bin/bash
export CPP    	:= g++
export AR	:= ar

PROJ_DIR	:= ../..

INSTALL_LIB_DIR	:= $(PROJ_DIR)/lib
INSTALL_BIN_DIR := $(PROJ_DIR)/bin
INSTALL_INC_DIR := $(PROJ_DIR)/include

LIB_DIR		:= lib
INC_DIR		:= include
SRC_DIR		:= src
OBJ_DIR		:= obj

METROMATH	:= MetroMath
METROMATH_A	:= libMetroMath.a
OUTPUT    	:= $(LIB_DIR)/$(METROMATH_A)

VPATH 		:= $(SRC_DIR) 

# Options
CPPFLAGS 	:= -I$(INC_DIR) 
CXXFLAGS	:= -c -g -Wall

# Objects ans Sources
SOURCE_PATHS 	:= $(shell find $(SRC_DIR) -name "*cpp") 
SOURCES 	:= $(foreach source, $(SOURCE_PATHS), $(subst $(SRC_DIR), , $(source) ))
OBJS	 	:= $(notdir $(SOURCES:.cpp=.o))
OBJECTS		:= $(addprefix $(OBJ_DIR)/, $(OBJS))

# Rule for building objects
$(OBJ_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

################### Phony targets ###########################
.PHONY: all install clean distclean uninstall docs

all: $(OUTPUT) 

install: all | $(INSTALL_INC_DIR)/$(METROMATH)
	cp -rf $(LIB_DIR)/* $(INSTALL_LIB_DIR)
	cp -rf $(INC_DIR)/* $(INSTALL_INC_DIR)/$(METROMATH)

clean:
	rm -f $(OBJ_DIR)/*.o
	$(shell find ./ -name "*~" | xargs -r rm )

distclean: clean
	rm -f $(OUTPUT)
	rm -rf doc/*

uninstall:
	rm -rf $(INSTALL_LIB_DIR)/$(METROMATH_A)
	rm -rf $(INSTALL_INC_DIR)/$(METROMATH)

docs:
	@doxygen

################## Main Targets #############################

$(OUTPUT): $(OBJECTS)
	$(AR) -rc $@ $(OBJECTS)

$(INSTALL_INC_DIR)/$(METROMATH): 
	mkdir $(INSTALL_INC_DIR)/$(METROMATH)

################### Header Dependencies #####################

%.cpp: %.h
RealEquality.o: RealPredicate.h
RealLessThan.o: RealPredicate.h