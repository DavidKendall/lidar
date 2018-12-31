# Primitive Lidar Program

## Introduction

This is a very basic program to read data from a RPLIDAR A1M8-R5 using a
FRDM-K64F with mbed-os. It needs significant work before use in a production system.

## Wiring

The cable of the RPLIDAR has the following pins:

1 - GND
2 - TX
3 - RX
4 - V5
5 - GND
6 - DTR
7 - V5

Pins 1-4 are for the sensor system and pins 5-7 are for the motor system.
The sensor system can be powered from the FRDM-K64F board. The motor system
should be powered separately. Pin 6 - DTR should be connected to a free PWM
pin, in case it is required to adjust the motor speed. Here, it is
assumed to be connected to D3 (PTA1). The sensor system pins should be connected
as follows: 1 - GND to GND, 2 - TX to D1, 3 - RX to D0, and 4 - V5 to 5v. Notice
here that TX goes to TX and RX goes to RX - they are not crossed over.

## Build and run

Open a terminal emulator at 115200 8N1. Import the repository, build and flash
as follows:

```sh
$ mbed import https://github.com/davidkendall/lidar
$ mbed compile -t GCC_ARM -m K64F -f
```

The program should report the health and information of the lidar, start a scan,
and then output the scan data.

Very little testing has been done. The angle data looks reasonable. I haven't
checked the plausibility of any other data. There is no error checking and,
consequently, no attempt at recovery from errors.



