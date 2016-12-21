####################### File Info ############################
# 
# Description: Make file for RobotController. It builds 
#		all local object files and two libraries:
#		libLocalization.a, libPathPlanner.a	
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

PROJ_DIR	:= ..

INSTALL_LIB_DIR	:= $(PROJ_DIR)/lib
INSTALL_BIN_DIR := $(PROJ_DIR)/bin
INSTALL_INC_DIR := $(PROJ_DIR)/include
INSTALL_ETC_DIR := $(PROJ_DIR)/etc

LIB_DIR 	:= lib/
BIN_DIR		:= bin
INC_DIR		:= include
SRC_DIR		:= src
OBJ_DIR		:= obj/
ETC_DIR		:= etc

RC		:= RobotController
EXECUTABLE	:= $(RC)
OUTPUT    	:= $(BIN_DIR)/$(EXECUTABLE)

# LIBRARIES
METROTIMER     	:= $(INSTALL_LIB_DIR)/libMetroTimer.a
METROTIMER_DIR	:= $(PROJ_DIR)/Utils/MetroTimer
METROTIMER_LIB 	:= $(INSTALL_LIB_DIR)
METROTIMER_INC 	:= $(INSTALL_INC_DIR)/MetroTimer

# Dependency: MetroCommunication
METROCOMMUNICATION     	:= $(INSTALL_LIB_DIR)/libMetroCommunication.a
METROCOMMUNICATION_DIR	:= $(PROJ_DIR)/Utils/MetroCommunication
METROCOMMUNICATION_LIB 	:= $(INSTALL_LIB_DIR)
METROCOMMUNICATION_INC 	:= $(INSTALL_INC_DIR)/MetroCommunication

# Dependency: ParticleFilter
PFILTER_A	:= $(INSTALL_LIB_DIR)/libParticleFilter.a
PFILTER_DIR	:= $(PROJ_DIR)/Utils/ParticleFilter
PFILTER_LIB	:= $(INSTALL_LIB_DIR)
PFILTER_INC	:= $(INSTALL_INC_DIR)/ParticleFilter

# Dependency: PathPlanner
PATHPLAN_A	:= libPathPlanner.a
PATHPLAN	:= $(LIB_DIR)/$(PATHPLAN_A)
PATHPLAN_DIR	:= PathPlanner
PATHPLAN_LIB	:= $(LIB_DIR)
PATHPLAN_INC	:= $(INC_DIR)/$(PATHPLAN_DIR)
PATHPLAN_SRC	:= $(SRC_DIR)/$(PATHPLAN_DIR)

# Dependency: SemaFORR
SEMAFORR_A   := libsemaFORR.a
SEMAFORR     := $(LIB_DIR)/$(SEMAFORR_A)

SEMAFORR_DIR := $(PROJ_DIR)/IntelligenceEngine/semaFORR
SEMAFORR_LIB := $(LIB_DIR)

USEMAFORR_DIR := $(PROJ_DIR)/Utils/semaFORR
USEMAFORR_LIB := $(USEMAFORR_DIR)/lib
USEMAFORR_INC := $(USEMAFORR_DIR)/include

SEMAFORR_INC := $(SEMAFORR_DIR)/include


# Dependency: RSWL
RSWL_A := librswl.a
RSWL   := $(LIB_DIR)/$(RSWL_A)
RSWL_DIR := $(PROJ_DIR)/IntelligenceEngine/RSWL
RSWL_LIB := $(LIB_DIR)
RSWL_INC := $(RSWL_DIR)/include

# Modules
CONTROLLER	:= Controller
VISDEBUG	:= VisualDebugger

VPATH 		:= $(SRC_DIR) $(SRC_DIR)/$(VISDEBUG) $(SRC_DIR)/$(CONTROLLER) 
VPATH		+= $(PATHPLAN_SRC)

# Options
INC_FLAGS	:= -I$(INC_DIR)/$(CONTROLLER) -I$(INC_DIR)/$(VISDEBUG) 
INC_FLAGS	+= -I$(METROTIMER_INC) -I$(PFILTER_INC) -I$(PATHPLAN_INC) 
INC_FLAGS	+= -I$(METROCOMMUNICATION_INC)
INC_FLAGS	+= -I$(SEMAFORR_INC) -I$(USEMAFORR_INC)
INC_FLAGS       += -I$(RSWL_INC)
CPPFLAGS 	:= $(INC_FLAGS) `pkg-config --cflags playerc++` `pkg-config --cflags opencv`
CXXFLAGS	:= -c -g -Wall 
LIBS   		:= -L$(METROCOMMUNICATION_LIB) -lMetroCommunication -L$(METROTIMER_LIB) -lMetroTimer 
LIBS		+= -L$(METROMATH_LIB) -lMetromath -L$(PFILTER_LIB) -lParticleFilter 
LIBS		+= -L$(PATHPLAN_LIB) -lPathPlanner
LIBS            += -L$(SEMAFORR_LIB) -lsemaFORR
LIBS            += -L$(USEMAFORR_LIB) -lsemaFORR
LIBS            += -L$(RSWL_LIB) -lrswl
LIBS            += -lboost_system-mt -lboost_date_time -lglut -lGLU -lGL -lpthread
LIBS		+= `pkg-config --libs playerc++` `pkg-config --libs opencv`


# Objects ans Sources
SOURCE_PATHS 	:= $(shell find $(SRC_DIR)/$(VISDEBUG) -name "*cpp") 
SOURCE_PATHS	+= $(shell find $(SRC_DIR)/$(CONTROLLER) -name "*cpp") 
SOURCE_PATHS	+= $(wildcard $(SRC_DIR)/*.cpp)
SRCS 		:= $(foreach source, $(SOURCE_PATHS), $(subst $(SRC_DIR)/$(VISDEBUG)/, , $(source) ))
SRCS 		:= $(foreach source, $(SRCS), $(subst $(SRC_DIR)/$(CONTROLLER)/, , $(source) ))
SOURCES		:= $(SRCS)
OBJS	 	:= $(notdir $(SOURCES:.cpp=.o))
OBJECTS		:= $(addprefix $(OBJ_DIR)/, $(OBJS))

# PathPlanner objects and sources
PP_SRC_PATH 	:= $(shell find $(PATHPLAN_SRC) -name "*cpp")
PP_SOURCES 	:= $(foreach source, $(PP_SRC_PATH), $(subst $(PATHPLAN_SRC)/, , $(source) ))
PP_OBJS 	:= $(notdir $(PP_SOURCES:.cpp=.o))
PP_OBJECTS	:= $(addprefix $(OBJ_DIR)/, $(PP_OBJS))
PP_INC_PATH	:= $(shell find $(PATHPLAN_INC) -name "*h" )
PP_HEADERS	:= $(foreach header, $(PP_INC_PATH), $(subst $(PATHPLAN_INC)/, , $(header) ))

# Rule for building objects
$(OBJ_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) $(CXXFLAGS) $< -o $@

################### Phony targets ###########################
.PHONY: all install clean distclean uninstall docs

all: $(OUTPUT) 

install: all
	cp $(OUTPUT) $(INSTALL_BIN_DIR)
	cp -rf $(LIB_DIR)* $(INSTALL_LIB_DIR)
	cp -rf $(PATHPLAN_INC) $(INSTALL_INC_DIR)

clean:
	rm -f $(OUTPUT)
	rm -f $(OBJ_DIR)*.o
	$(shell find ./ -name "*~" | xargs -r rm )

distclean: clean
	rm -rf lib/*
	rm -rf doc/*

uninstall: 
	rm -f $(INSTALL_BIN_DIR)/$(EXECUTABLE)
	rm -f $(INSTALL_LIB_DIR)/$(PATHPLAN_A)
	rm -rf $(INSTALL_INC_DIR)/$(PATHPLAN_DIR)

docs:
	@doxygen

################## Main Targets #############################

# build localization, pathplanner and the controller
$(OUTPUT): $(PATHPLAN) $(OBJECTS)
	$(CPP) -o $@ $(OBJECTS) $(LIBS)

# build dependent library: PathPlanner
$(PATHPLAN): $(PP_OBJECTS) 
	$(AR) -rc $@ $(PP_OBJECTS)

################### Header Dependencies #####################

%.cpp : %.h
#Aibo.o : 
#astar.o : 
#CollisionAvoider.o : 
#CommunicationManager.o : 
#Controller.o :
#Graph.o :
#GridCell.o :
#GridMap.o :
#Localization.o :
#main.o : 
#MapManager.o :
#MapMarker.o :
#Map.o : 
#MapWall.o : 
#MCLPositionEstimator.o : 
#MCPainter.o :
#MonteCarloDebugger.o : 
#MonteCarloInterface.o :
#MonteCarlo.o :
#Move.o :
#Observation.o :
#Particle.o :
#PathPlanner.o :
#Position.o :
#Surveyor.o :
#VisualDebugger.o : 
