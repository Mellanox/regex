#!/bin/bash -e
# Copyright (C) 2015-2020 Titan IC Systems Ltd. All rights reserved.

if [ ! -e hra_lite ]; then
    make
fi

mkdir -p rof
mkdir -p ruleset

# hra_lite is hard coded to use port zero
RXP_PORT=0

# Work out -V (version) option for rxpc command.
RXP_VERSION=""
VERSION_FILE=/sys/class/rxp/rxp$RXP_PORT/rxp_version
if test -f "$VERSION_FILE"; then
    RXP_VERSION=`cat $VERSION_FILE`
    if [ $RXP_VERSION == 5.7 ]; then
        RXP_VERSION="-V5.7"
    elif [ $RXP_VERSION == 5.8 ]; then
        RXP_VERSION="-V5.8"
    fi
fi

# Create a simple rules file, with a single rule "hello"
echo "1,/hello world/" > ruleset/synthetic.rules

# Compile the rules file. All output files will be prefixed by "rof/synthetic"
rxpc -f ruleset/synthetic.rules -o rof/synthetic $RXP_VERSION

# Run the hra application
sudo ./hra_lite
