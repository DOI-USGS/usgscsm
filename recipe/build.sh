#!/bin/bash

if [ `uname` == Darwin ]; then
  export MACOSX_DEPLOYMENT_TARGET=10.9;
  $PYTHON setup.py install;
else
  CC=gcc-6 CXX=g++-6 $PYTHON setup.py install;
fi
