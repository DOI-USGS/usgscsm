#!/bin/sh
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DUSGSCSM_BUILD_TESTS=OFF ..
make install
