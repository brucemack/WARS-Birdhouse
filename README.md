WARS LoRa Birdhouse Project
===========================

The objective of this project is to explore the potential of using low-cost LoRa radios to build simple mesh networks that can pass text messages across town.

Nodes on the network are autonomous, solar-powered birdhouses that each contain a small radio with 100mW of transmit power.  Our birdhouses can run 24x7 assuming reasonable weather conditions. 

Our software supports a simple message routing protocol that allows packets to "hop" between houses in the network to reach their final destination.

We have successfully routed messages between houses that were separated by approximately 1 kilometer.  Antenna hieght is important. 

An internet gateway is also under development.

Hardware Overview
=================

The birdhouse repeater (external view):

![house1](images/IMG_0645.jpg)

The birdhouse repeater (internal view):

![house2](images/IMG_0852.jpg)

A tower-mounted birdhouse repeater at the QTH of KC1FSZ.

![house3](images/IMG_0853.jpg)

A tree-mounted repeater installed in a tree inside of a cloverleaf on-/off-ramp.

![house4](images/IMG_0856.jpg)

Software Overview
=================

The birdhouse is controlled using an ESP32 at the moment, although this decision is under  consideration.  

All nodes support a serial interface for interacting with the network, but this is only connected for desktop nodes.  

We are not using off-the-shelf drivers for the LoRa radio - this is completely homebrew.  

The serial command processor is implemented using this [very good project](https://github.com/philj404/SimpleSerialShell).

A static routing mechanism is being used at the moment.  The routing table for each node can be changed remotely.  Dynamic routing will be developed in a future phase.

Reference Material
==================

* Reference for LoRa module (RFM95W): https://www.hoperf.com/modules/lora/RFM95.html
* Reference for 18650 battery: https://cdn.sparkfun.com/datasheets/Prototyping/ICR18650%202600mAH%20datasheet.pdf
* Reference for LDO Voltage Regulator: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP1700-Low-Quiescent-Current-LDO-20001826E.pdf

