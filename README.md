# SandTable

This repository contains the source code for controlling a custom-built kinetic art coffee table, similar to existing [Sisyphus](https://sisyphus-industries.com/) tables.

This project is currently a work in progress.

## Software Overview

The internal table mechanism is an RP robotic arm controlled by a Raspberry Pi and [Phidgets stepper motor controllers](https://www.phidgets.com/?tier=3&catid=23&pcid=20&prodid=1121).  The controller code executes forward kinematic trajectory following by maneuvering to different `(rho, theta)` joint positions specified in Theta Rho files (denoted with `.thr` extension) to generate different designs in the sand.

## Mechanical Overview

The frame of the table is a basic [Ikea Vittsjö](https://www.ikea.com/us/en/p/vittsjoe-coffee-table-black-brown-glass-80213309/) coffee table modified with a custom laser-cut RP robotic arm and mounting plates.  Two NEMA stepper motors drive each of the robot joints, and the end effector of the robot holds a strong neodymium magnet to attract and control the position of a steel ball bearing held on the sand surface above.  A ring of addressable RGBW LEDs wraps around the table and is controlled via GPIO pins on the Pi.

## TODO
- Implement Google Home Connectivity
- LED Light Ring Control
- Automated Theta Rho file generation
