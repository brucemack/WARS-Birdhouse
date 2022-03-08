WARS LoRa Birdhouse Project
===========================

This project explores the potential of using low-cost LoRa radios to build simple mesh networks that can pass text messages around town.  This project is being undertaken by members of the Wellesley Amateur Radio Society.

Nodes on the network are autonomous, solar-powered birdhouses that each contain a 100mW radio (Semtech SX1276).  These birdhouses can run 24x7 assuming reasonable weather conditions.  Desktop nodes are used to access the network from a computer.  An internet gateway node is also under development.

Commodity components are being used to keep birdhouse costs to a minimum.  

The software supports a simple message routing protocol that allows packets to "hop" between houses to reach their final destination.

A prototype network of 5 stations has been constructed.  Messages have been successfully routed between birdhouses that were separated by approximately 1 kilometer.  Antenna height is important. 

Related Technology
==================

* LoRaWAN: Uses LoRa stations organized in a star topology.  The hub of each star is a gateway to the public internet.  This project is not related to LoRaWAN in any way.

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

The LoRa software is completely homebrew - no off-the-shelf drivers were used.

The serial command processor is implemented using this [very good project](https://github.com/philj404/SimpleSerialShell).

A static routing mechanism is being used at the moment.  The routing table for each node can be changed remotely.  Dynamic routing will be developed in a future phase.

Reference Material
==================

* Reference for LoRa module (RFM95W): https://www.hoperf.com/modules/lora/RFM95.html
* Reference for 18650 battery: https://cdn.sparkfun.com/datasheets/Prototyping/ICR18650%202600mAH%20datasheet.pdf
* Reference for LDO Voltage Regulator: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP1700-Low-Quiescent-Current-LDO-20001826E.pdf

