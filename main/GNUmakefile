# Include your header file location
CXXFLAGS += -I. $(shell root-config --cflags) -g

# Include your shared object lib location
LDFLAGS += $(shell root-config --libs) -lGeom -lPhysics -lMatrix -g

# platform-specific options
OSNAME = $(shell uname -s)
include $(MAKE_TOP_DIR)/Makefile/Makefile.${OSNAME}

# Add your program below with a space after the previous one.
# This makefile compiles all binaries specified below.

all: ../bin/myfmwk

../bin/myfmwk:
	@echo '<<compiling' $@'>>'
	@mkdir -p ../bin
	$(CXX) myfmwk.cc -o ../bin/$@ $(CXXFLAGS) $(LDFLAGS)
	@rm -rf ../bin/*.dSYM

clean:	
	rm -f ../bin/myfmwk
