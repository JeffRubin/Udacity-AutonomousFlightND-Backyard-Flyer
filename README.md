# Udacity-AutonomousFlightND-Backyard-Flyer

## Overview
Event-driven program to fly a simulated quadcopter in a square pattern.

## New Command Line Arguments
alt: altitude (meters) at which to fly the square; up is +
* Default is 10 m

side: size (meters) of each side of the square to fly 
* Default is 20 m

plot: turn on plotting (True/False) using visdom at http://localhost:8097
* Default is False (since slows execution down as explained below)
* Only recommending setting this True if your computer has a lot of resources otherwise it will slow everything down a lot
* Make sure to start the following first if plotting is set to True: python -m visdom.server
