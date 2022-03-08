WARS LoRa Birdhouse Project
===========================

This project explores the potential of using low-cost LoRa radios to build simple mesh networks that can pass text messages around town.  This project is being undertaken by members of the Wellesley Amateur Radio Society.

Nodes on the network are autonomous, solar-powered birdhouses that each contain a 100mW radio (Semtech SX1276).  These birdhouses can run 24x7 assuming reasonable weather conditions.  Desktop nodes are used to access the network from a computer.  An internet gateway node is also under development.

The birdhouse is run by an ESP32 microcontroller at the moment, although this decision is under  consideration.  A more power-efficient STM32 prototype is being worked on.

Commodity components are being used to keep birdhouse costs to a minimum.  Our goal is to keep the node cost under $50 USD. 

The software supports a simple message routing protocol that allows packets to "hop" between houses to reach their final destination.

The 915 MHz ISM band is used to simplify licensing considerationa.  

A prototype network of 5 stations has been constructed in Wellesley, MA.  Messages have been successfully routed between birdhouses that were separated by approximately 1 kilometer.  Antenna height is important. 

Related Technology
==================

* LoRaWAN: Uses LoRa stations organized in a star topology.  The hub of each star is a gateway to the public internet.  This project is not related to LoRaWAN in any way.

Hardware Overview
=================

The birdhouse repeater prototype (external view):

![house1](images/IMG_0645.jpg)

The birdhouse repeater prototype (internal view):

![house2](images/IMG_0852.jpg)

A tower-mounted birdhouse repeater prototype at the QTH of KC1FSZ.

![house3](images/IMG_0853.jpg)

A tree-mounted repeater prototype installed in a tree inside of a cloverleaf on-/off-ramp.

![house4](images/IMG_0856.jpg)

Hardware Notes
--------------
* Voltage readings are taken on solar panels and batteries.

Software Overview
=================

All nodes support a serial interface for interacting with the network, but this is only connected for desktop nodes.  

The LoRa software is completely homebrew - no off-the-shelf drivers were used.

The serial command processor is implemented using this [very good project](https://github.com/philj404/SimpleSerialShell).

A static routing mechanism is being used at the moment.  The routing table for each node can be changed remotely.  Dynamic routing will be developed in a future phase.

Reference Material
==================

* Reference for LoRa module (RFM95W): https://www.hoperf.com/modules/lora/RFM95.html
* Reference for 18650 battery: https://cdn.sparkfun.com/datasheets/Prototyping/ICR18650%202600mAH%20datasheet.pdf
* Reference for LDO Voltage Regulator: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP1700-Low-Quiescent-Current-LDO-20001826E.pdf

Detailed Parts List
-------------------
(To follow)

Areas for Experimentation
=========================

Software
--------
* Improved power efficiency using more agressive sleeping.  Leverage the SX1276 carrier detect interrupt to allow the system to sleep during periods of inactivity.
* Dynamic route discovery.
* A more user-friendly desktop application written in Python.
* Store and forward.

Hardware
--------
* Complete the packaging of the birdhouse to ensure full compatibilty with avian residency.
* Improved power efficiency using smaller microcontrollers.
* Cheaper antennas?
* Gain antennas for longer distance links.
* A custom PCM to simplify construction for club builds.
