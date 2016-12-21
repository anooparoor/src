####################### File Info ############################
# 
# Description: Make file for tools. Calls makefiles of
#	       subdirectories.
#
# Author : A. Tuna Ozgelen 
# 
# Usage : 
#	> make 
#		will call make all for the subdirectories 
#	> make install
#		will call make install for the subdirectories. 
#		this will copy binaries and libraries to project directories 
#	> make distclean 
#		will call make distclean for the subdirectories 
#		this will clean binaries, *.o, libraries and docs from local directories 
#	> make docs
#		will call make docs for the subdirectories 
#		will build an html and a latex version of into  
#		doc, using doxygen
#

OK_MSG		:= \033[1;32m[OK]\033[0m
FAILED_MSG	:= \033[1;31m[Failed]\033[0m
WARNING_MSG	:= \033[1;33m[Warning]\033[0m

SUB_DIRS	= ErrorModeller RobotAutoCalibrate

.PHONY: all install distclean uninstall docs

all:
	@for dir in $(SUB_DIRS); do \
		echo -e -n "  Building $$dir... "; \
		$(MAKE) -e -s --directory=$$dir; \
		if [ "$$?" -eq 0 ]; then \
			echo -e "$(OK_MSG)" ; \
		else \
			echo -e "$(FAILED_MSG)" ; \
		fi \
	done 

install: 
	@for dir in $(SUB_DIRS); do \
		echo -e -n "  Installing $$dir... "; \
		$(MAKE) -e -s --directory=$$dir install; \
		if [ "$$?" -eq 0 ]; then \
			echo -e "$(OK_MSG)" ; \
		else \
			echo -e "$(FAILED_MSG)" ; \
		fi \
	done 

distclean: 
	@for dir in $(SUB_DIRS); do \
		echo -e -n "  Cleaning $$dir... "; \
		$(MAKE) -e -s --directory=$$dir distclean; \
		if [ "$$?" -eq 0 ]; then \
			echo -e "$(OK_MSG)" ; \
		else \
			echo -e "$(FAILED_MSG)" ; \
		fi \
	done 

uninstall: 
	@for dir in $(SUB_DIRS); do \
		echo -e -n "  Uninstalling $$dir... "; \
		$(MAKE) -e -s --directory=$$dir uninstall; \
		if [ "$$?" -eq 0 ]; then \
			echo -e "$(OK_MSG)" ; \
		else \
			echo -e "$(FAILED_MSG)" ; \
		fi \
	done 

docs:
	@for dir in $(SUB_DIRS); do \
		echo -e -n "  Generating documents $$dir... "; \
		$(MAKE) -e -s --directory=$$dir docs; \
		if [ "$$?" -eq 0 ]; then \
			echo -e "$(OK_MSG)" ; \
		else \
			echo -e "$(FAILED_MSG)" ; \
		fi \
	done 

