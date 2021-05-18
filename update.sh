#!/usr/bin/env bash

git pull --recurse-submodules
#git submodule update --init --recursive
./downward/build.py
