#!/bin/bash

export USER_MODULE="DaqFile"
me="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Find the directory one above.
export LITE_FMWK_BASEDIR="$( cd "$( dirname "$me" )" && pwd )"

source $LITE_FMWK_BASEDIR/config/setup.sh

