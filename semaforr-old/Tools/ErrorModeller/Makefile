####################### File Info ############################
# 
# Description: Make file for ErrorModeller
#
# Author : A. Tuna Ozgelen 
#	(extended from George Rabanca 
#	and  Mark Manashirov's make files)
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

BIN_DIR		:= bin
INC_DIR		:= include
SRC_DIR		:= src
OBJ_DIR		:= obj

EXECUTABLE	:= ErrorModeller
OUTPUT    	:= $(BIN_DIR)/$(EXECUTABLE)

# LIBRARIES
# Dependency: MetroTimer MetroCommunication
METROTIMER     	:= MetroTimer
METROTIMER_INC 	:= $(INSTALL_INC_DIR)/MetroTimer
METROCOMM     	:= MetroCommunication
METROCOMM_INC 	:= $(INSTALL_INC_DIR)/MetroCommunication

VPATH 		:= $(SRC_DIR) 

# Options
INC_FLAGS	:= -I$(INC_DIR) -I$(METROTIMER_INC) -I$(METROCOMM_INC)
CPPFLAGS 	:= $(INC_FLAGS) `pkg-config --cflags playerc++` 
CXXFLAGS	:= -c -g -Wall
LIBS   		:= -lboost_system-mt `pkg-config --libs playerc++` -L$(INSTALL_LIB_DIR) -l$(METROTIMER) -L$(INSTALL_LIB_DIR) -l$(METROCOMM) -lpthread

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

# build Skygrid
$(OUTPUT): $(OBJECTS)
	$(CPP) -o $@ $(OBJECTS) $(LIBS)


################### Header Dependencies #####################


