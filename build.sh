#!/bin/bash

${CROSS_COMPILE}g++ -Wall -Wno-address-of-packed-member main.cpp SerialPort.cpp --std=gnu++11 -o mavlink_listener
