#!/usr/bin/bash

# allow user to use Navio2 i/o

# LSM
chmod a+rwX /dev/spidev0.3
chmod a+rwX /dev/spidev0.2

# MPU
chmod a+rwX /dev/spidev0.1

# LED


# PWM
