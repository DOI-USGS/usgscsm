#!/bin/sh
git submodule update --init
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PREFIX -DUSGSCSM_EXTERNAL_DEPS=OFF -DUSGSCSM_BUILD_DOCS=OFF -DUSGSCSM_BUILD_TESTS=OFF ..
make install
