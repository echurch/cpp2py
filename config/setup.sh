#!/bin/bash

# If LITE_FMWK_BASEDIR not set, try to guess
if [[ -z $LITE_FMWK_BASEDIR ]]; then
    # Find the location of this script:
    me="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    # Find the directory one above.
    export LITE_FMWK_BASEDIR="$( cd "$( dirname "$me" )" && pwd )"
fi

echo "LITE_FMWK_BASEDIR = $LITE_FMWK_BASEDIR"
if [[ -z $LITE_FMWK_BASEDIR ]]; then
    echo \$LITE_FMWK_BASEDIR not set! 
    echo You have to set this first.
else 
    # Abort if ROOT not installed. Let's check rootcint for this.
    if [[ -z `command -v rootcint` ]]; then
	echo
	echo Looks like you do not have ROOT installed.
	echo You cannot use LArLight w/o ROOT!
	echo Aborting.
	echo
	return;
    fi
    LITE_FMWK_OS=`uname -s`
    # Set for core build
    export LITE_FMWK_LIBDIR=$LITE_FMWK_BASEDIR/lib
    export LITE_FMWK_COREDIR=$LITE_FMWK_BASEDIR/core

    # Set for user dev build
    export CORE_RELPATH=core
    export CORE_DIR=$LITE_FMWK_BASEDIR/$CORE_RELPATH

    if [[ -z $USER_MODULE ]]; then
	export USER_MODULE=""
    fi

    # Check compiler availability for clang++ and g++
    LITE_FMWK_CXX=clang++
    if [ `command -v $LITE_FMWK_CXX` ]; then
	export LITE_FMWK_CXX="clang++ -std=c++11";
    else
	LITE_FMWK_CXX=g++
	if [[ -z `command -v $LITE_FMWK_CXX` ]]; then
	    echo
	    echo Looks like you do not have neither clang or g++!
	    echo You need one of those to compile LArLight... Abort config...
	    echo
	    return;
	fi
	export LITE_FMWK_CXX;
	if [ $LITE_FMWK_OS = 'Darwin' ]; then
	    echo $LITE_FMWK_OS
	    echo
	    echo "***************** COMPILER WARNING *******************"
	    echo "*                                                    *"
	    echo "* You are using g++ on Darwin to compile LArLight.   *"
	    echo "* Currently LArLight assumes you do not have C++11   *"
	    echo "* in this combination. Contact the author if this is *"
	    echo "* not the case. At this rate you have no ability to  *"
	    echo "* compile packages using C++11 in LArLight.          *"
	    echo "*                                                    *"
	    echo "* Help to install clang? See manual/contact author!  *"
	    echo "*                                                    *"
	    echo "******************************************************"
	    echo 
	fi
    fi
    if [[ -z $ROOTSYS ]]; then
	case `uname -n` in
	    (houston.nevis.columbia.edu)
	    if [[ -z ${ROOTSYS} ]]; then
		source /usr/nevis/adm/nevis-init.sh
		setup root
		export PYTHONPATH=$ROOTSYS/lib:$PYTHONPATH;
	    fi
	    ;;
	    (*)
	    echo
	    echo "****************** PyROOT WARNING ********************"
	    echo "*                                                    *"
	    echo "* Did not find your \$ROOTSYS. To use PyROOT feature, *"
	    echo "* Make sure ROOT.py is installed (comes with ROOT).  *"
	    echo "* You need to export \$PYTHONPATH to include the dir  *"
	    echo "* where ROOT.py exists.                              *"
	    echo "*                                                    *"
	    echo "* Help to install PyROOT? See manual/contact author! *"
	    echo "*                                                    *"
	    echo "******************************************************"
	    echo
	    ;;
	esac
    else
	export PYTHONPATH=$ROOTSYS/lib:$PYTHONPATH;
    fi

    python $LITE_FMWK_BASEDIR/config/python/gen_coremakefile.py
    python $LITE_FMWK_BASEDIR/config/python/gen_topmakefile.py
    export LD_LIBRARY_PATH=$LITE_FMWK_LIBDIR:$LD_LIBRARY_PATH
    if [ $LITE_FMWK_OS = 'Darwin' ]; then
	export DYLD_LIBRARY_PATH=$LITE_FMWK_LIBDIR:$DYLD_LIBRARY_PATH
    fi
    mkdir -p $LITE_FMWK_BASEDIR/bin
    mkdir -p $LITE_FMWK_BASEDIR/lib

    export PATH=$LITE_FMWK_BASEDIR/bin:$PATH
    echo
    echo "Finish configuration. To build, type:"
    echo "> cd \$LITE_FMWK_BASEDIR"
    echo "> make"
    echo
fi
