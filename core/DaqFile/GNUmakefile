#
# This is an example GNUmakefile for my packages
#

# specific names for this package
DICT  = DaqFileCint
SHLIB = libDaqFile.so
SOURCES = $(wildcard *.cc)
HEADERS = $(wildcard *.hh)
OBJECTS = $(SOURCES:.cc=.o)

# include options for this package
INCFLAGS  = -I$(LITE_FMWK_COREDIR)
INCFLAGS += -I./                       #Include itself
INCFLAGS += -I$(BOOST_INC)
INCFLAGS += -I$(UBOONEDAQ_DATATYPES_INC)

# platform-specific options
OSNAME          = $(shell uname -s)
HOST            = $(shell uname -n)
OSNAMEMODE      = $(OSNAME)

include $(LITE_FMWK_BASEDIR)/Makefile/Makefile.${OSNAME}

# set compiler options for ROOT
CXXFLAGS +=  `root-config --cflags`

# call the common GNUmakefile
include $(LITE_FMWK_BASEDIR)/Makefile/GNUmakefile.CORE



