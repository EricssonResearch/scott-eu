#!/bin/bash
echo "export PYTHONPATH=$PYTHONPATH:$PWD/lib" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PWD/lib" >> ~/.bashrc
source ~/.bashrc
