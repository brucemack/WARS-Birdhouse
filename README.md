WARS LoRa Birdhouse Project
===========================

This project explores the potential of using low-cost/low-bandwidth LoRa radios to build simple mesh networks that can pass text messages around town.  This project is being undertaken by members of the Wellesley Amateur Radio Society.

Nodes on the network are autonomous, solar-powered birdhouses that each contain a 100mW radio (Semtech SX1276).  These birdhouses can run 24x7 assuming reasonable weather conditions.  USB-connected desktop nodes are used to access the network from a computer.  An internet gateway node is also under development. 

The birdhouse is run by an ESP32 microcontroller at the moment, although this decision is under  consideration.  A more power-efficient STM32 prototype is being worked on.

Commodity components are being used to keep birdhouse costs to a minimum.  Our goal is to keep the node cost under $50 USD. 

The software supports a simple message routing protocol that allows packets to "hop" between houses to reach their final destination.

The 915 MHz ISM band is used to simplify licensing considerations.  

A prototype network of 5 stations has been constructed in Wellesley, MA.  Messages have been successfully routed back and forth across the entire mesh, including hops between birdhouses that were separated by approximately 1 kilometer.  Antenna height is important.  The houses have been subjected to bad weather conditions.  

Related Technology
==================

* LoRaWAN: Uses LoRa stations organized in a star topology.  The hub of each star is a gateway to the public internet.  This project is not related to LoRaWAN in any way.

Hardware Overview
=================

The birdhouse repeater prototype (external view):

![house1](images/IMG_0645.jpg)

The birdhouse repeater prototype (internal electronics view):

![house2](images/IMG_0852.jpg)

The wood parts for the birdhouse repeater prototype:

![house4](images/IMG_0642.jpg)

A tower-mounted birdhouse repeater prototype at the QTH of KC1FSZ:

![house3](images/IMG_0853.jpg)

A tree-mounted repeater prototype installed in a tree inside of a cloverleaf on-/off-ramp:

![house4](images/IMG_0856.jpg)

Hardware Notes
--------------
* Voltage readings are taken on solar panels and batteries.

Software Overview
=================

The LoRa software is completely homebrew - no off-the-shelf drivers are being used.  

All nodes support a serial interface for interacting with the network, but this is only connected for desktop nodes.  

The serial command processor is implemented using this [very good project](https://github.com/philj404/SimpleSerialShell).

A static routing mechanism is being used at the moment.  The routing table for each node can be changed remotely.  Dynamic routing will be developed in a future phase.

Reference Material
==================

* LoRa rules, regulations, and terminology: https://lora.readthedocs.io/en/latest/#rules-and-regulations
* Reference for LoRa radio module (RFM95W): https://www.hoperf.com/modules/lora/RFM95.html
* Reference for 18650 battery: https://cdn.sparkfun.com/datasheets/Prototyping/ICR18650%202600mAH%20datasheet.pdf
* Reference for LDO Voltage Regulator: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP1700-Low-Quiescent-Current-LDO-20001826E.pdf
* Reference for STM32L031 microcontroller: https://www.st.com/resource/en/datasheet/stm32l031k6.pdf
* STM32L0 hardware design guide: https://www.st.com/resource/en/application_note/an4467-getting-started-with-stm32l0xx-hardware-development-stmicroelectronics.pdf
* Reference for the ESP32 D1 Mini: https://wiki.csgalileo.org/_media/projects/internetofthings/d1_mini_esp32_-_pinout.pdf
* An article on correcting non-linearity in the ESP32 AD converter: https://github.com/e-tinkers/esp32-adc-calibrate

Detailed Parts List
-------------------
(To follow)

Areas for Further Experimentation
=================================

Hardware 2.0
------------
* Fully integrated PCB using surface mount compontents
* Switch to a ultra-low-power microprocessor: STM32L031
* SMA connector for more robust antenna experimentation

Hardware (Future)
-----------------
* Replace the linear regulator with a boost converter to improve battery usage.
* Complete the packaging of the birdhouse to ensure weatherproofing and full compatibilty with avian residency.
* Improved power efficiency using smaller microcontrollers.
* Cheaper antennas.
* Gain antennas for longer distance links.
* RF switch to allow dynamic switching between two antennas.  This might faciliate A/B testing, or possibly stations that have a directional/gain antenna for trunking and an omni-directional antenna for local access.
* A custom PCB to simplify construction for club builds.

Software
--------
* Improved power efficiency using more agressive sleeping.  Leverage the SX1276 receive interrupt or channel activity detection (CAD) interrupt to allow the system to sleep during periods of inactivity.
* Dynamic route discovery. At the moment the routes are static.
* Store and forward for times when a node is offline.
* Message sequence to avoid duplicate delivery
* Network time synchronization
* A more user-friendly desktop application written in Python.  This will make it easer for 
end-users to interact with the network.
* Network security.




