#!/bin/sh

# build project and examples
mkdir -p build
ls
cd build
ls
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j4