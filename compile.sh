#!/bin/bash

SOURCE_FILE1="main.cpp"
SOURCE_FILE2="convolutional_code.cpp"

SOURCE_DIR="src"

OUTPUT_DIR="bin"

CPP_FLAGS="-std=c++11 -Wall"

g++ $CPP_FLAGS -o $OUTPUT_DIR/conv_code -I $SOURCE_DIR $SOURCE_DIR/$SOURCE_FILE1 $SOURCE_DIR/$SOURCE_FILE2
