###############################################################################
# File: math.mak
#
# Purpose:
#   Compile the gnc-fsw math library
#
# $Log: math.mak  $
# Revision 1.1 2008/05/21 15:00:19EDT dcmccomas 
# Initial revision
# Member added to project c:/MKSDATA/MKS-REPOSITORY/GNC-FSW-REPOSITORY/lib/math/fsw/project.pj
# Revision 1.2 2007/02/21 14:10:30EST myyang 
# - Add vec6d.o to list of OBJS.
# Revision 1.1 2006/11/17 12:05:53EST lvshackelford 
# Initial revision
# Member added to project d:/mksdata/MKS-LRO-REPOSITORY/build/rad750/math/project.pj
# Revision 1.1.1.3 2006/11/16 16:07:46EST lvshackelford 
# 2889:1 Prep *.mak files for move to RAD750 environment.
#
###############################################################################
#
# Subsystem produced by this makefile.
#
APPTARGET = math

# 
# Entry Point for task
# 
ENTRY_PT = Math_Init

#
# Object files required to build subsystem.
#
OBJS := matrix3x3d.o
OBJS += matrix3x4d.o
OBJS += matrix4x3d.o
OBJS += matrix6x6d.o
OBJS += matrixmxnd.o
OBJS += quaternion.o
OBJS += scalar.o
OBJS += vector3d.o
OBJS += vector4d.o
OBJS += vector6d.o
OBJS += math.o

#
# Source files required to build subsystem; used to generate dependencies.
# As long as there are no assembly files this can be automated.
#
SOURCES := $(OBJS:.o=.c)
#SOURCES +=

#
# Extra C flags and defines to pass into the build
#
LOCALCOPTS := 
#LOCALCOPTS += 

###########################################################################
# Architecture Specific Options:
# The following area is for architecture specific build options
###########################################################################

#
# Include the memory slot definitions.
# This defines where the apps are located in vxWorks 5.5 and RTEMS
#
include ../exe/memslots.mak

#
# Memory slot for application link. This is only for
#  implementations that have a static pre-defined memory
#  map for applications.
#
MEMSLOT = $(MATH_LIB_MEMSLOT)

###########################################################################
# Should not have to change the file below here
##########################################################################

#
# Set build type to CFE_APP. This allows us to 
# define different compiler flags for the cFE Core and Apps.
# 
BUILD_TYPE = CFE_APP

##
## Define the MISSIONBASE macro for the location of the 
## mission build tree. This allows us to get the mission configuration
## includes 
##
MISSIONBASE =   $(shell cd ../..; pwd)

##
## Define the CPUBASE macro for the location of the CPU
## build tree
##
CPUBASE =   $(shell cd ..; pwd)

#
# Include build-specific configuration information.
#
include $(CPUBASE)/cfe/config/prolog.mak 
include $(CONFIG_DEF)

#
# Source and include file directories that are specific to subsystem.
#
LOCALSRC := $(APPBASE)/$(APPTARGET)/src

LOCALINC = $(APPBASE)/$(APPTARGET)/inc

#
# Include Rules for building standard targets.
#
include $(RULE_DEF)

#
# Default rule for building app
#
default: $(APPTARGET).$(APP_EXT)

#
# Default rule for installing app
#
install::
	cp $(APPTARGET).$(APP_EXT) ../exe

#
# Include the dependancy list
#
-include $(APPTARGET).d

# eof
