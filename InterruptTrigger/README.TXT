Make Trigger Projekt
=================

To compile your source code, write following command:

   gcc -Wall -o InterruptTimer InterruptTimer.c -lwiringPi -lcrypt -lpthread -lwiringPiDev -lm -lrt -lpigiod_if2 -pthread

Before starting the script. Type following command to start GPIOs

   sudo pigpiod

Start script 

   ./InterruptTrigger

