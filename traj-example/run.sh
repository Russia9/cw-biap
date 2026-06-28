#!/usr/bin/bash
cmake -B make_rocket
cmake --build make_rocket
make_rocket/main
python3 plots.py
echo "Done"