#!/bin/sh

# build project and examples
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4