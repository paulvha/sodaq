###############################################################
# makefile for ATToL. June 2020 / paulvha
#
# make clean : to remove previous files
#
# make to create ATToL and move to /usr/bin
###############################################################

# GCC flags
CXXFLAGS := -Wall -Werror

# install directory
InstallDir := /usr/bin

# set variables
CC := gcc

.PHONY : clean ATToL

ATToL :
	$(CC) $(CXXFLAGS) -o ATToL ATToL.cpp
	mv ATToL $(InstallDir)
	chmod +x $(InstallDir)/ATToL

clean :
	rm -f $(InstallDir)/ATToL ATToL.o ATToL


