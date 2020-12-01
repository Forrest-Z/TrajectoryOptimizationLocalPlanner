#!/bin/bash
rm -rf build
mkdir build

# threads to use for build
num_threads=$(($(grep -c ^processor /proc/cpuinfo)-2))
cd build
cmake ..
make -j$(num_threads)