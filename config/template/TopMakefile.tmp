#####################################################################################
#
# A top Makefile for building my project.
# One needs to define $LITE_FMWK_BASEDIR to build the sub directories.
#
#####################################################################################
#
# IMPOSE CONDITION BETWEEN LITE_FMWK_BASEDIR & PWD =>
#   do not compile if PWD !=$LITE_FMWK_BASEDIR is set elsewhere
#
ifndef LITE_FMWK_BASEDIR
 LITE_FMWK_BASEDIR := $(shell cd . && pwd -P)
endif
NORMALIZED_LITE_FMWK_BASEDIR := $(shell cd ${LITE_FMWK_BASEDIR} && pwd -P)
ifneq ($(NORMALIZED_LITE_FMWK_BASEDIR), $(shell cd . && pwd -P))
 ifneq ($(wildcard ./PMTDecoder/*),)
ERROR_MESSAGE := $(error Your source code detected in current dir, but LITE_FMWK_BASEDIR is not current dir.  \
   To avoid recompiling the wrong installation,\
   LITE_FMWK_BASEDIR must be set to the current directory when making.  \
   Currently it is set to ${LITE_FMWK_BASEDIR} [$(NORMALIZED_LITE_FMWK_BASEDIR)].  \
   Current directory is $(shell pwd).)
 endif
endif
export LITE_FMWK_BASEDIR

all:
	@mkdir -p $(LITE_FMWK_BASEDIR)/lib
	@echo "Start building core"
	@make --directory=$(MYFMWK_CORE_DIR)
	@echo
	@echo "Start building main"
	@make --directory=$(LITE_FMWK_BASEDIR)/main
	@echo 
	@echo "Exiting"

clean:
	@echo "Cleaning core"
	@make clean --directory=$(MYFMWK_CORE_DIR)
	@echo
	@echo "Cleaning main"
	@make clean --directory=$(LITE_FMWK_BASEDIR)/main
	@echo
	@echo "Cleaning lib"
	@rm -f  $(LITE_FMWK_LIBDIR)/*.so
	@rm -f  $(LITE_FMWK_LIBDIR)/*.rootmap
	@echo
	@echo "Exiting"

#####################################################################################
#
# DOCUMENTATION...
#
doxygen:
	@echo 'dOxygenising your code...'
	@doxygen $(LITE_FMWK_BASEDIR)/doc/doxygenMyProject.script

doxygen+:
	@echo 'dOxygenising MyProject + local-ROOT...'
	@doxygen $(LITE_FMWK_BASEDIR)/doc/doxygenMyProject+.script
#
#####################################################################################
