#!/bin/bash
#make COMPILE=gcc BOOT=new SPI_SIZE_MAP=6 APP=1
#make clean
make BOOT=new APP=1 SPI_SPEED=40 SPI_MODE=QIO SPI_SIZE_MAP=4
