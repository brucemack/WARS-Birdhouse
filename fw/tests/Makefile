test1:
	g++ -ggdb -DARDUINO -I./mocks -o unit-test-1 unit-test-1.cpp \
	../WARS-Birdhouse/OutboundPacket.cpp \
	../WARS-Birdhouse/OutboundPacketManager.cpp \
	../WARS-Birdhouse/RoutingTable.cpp \
	../WARS-Birdhouse/MessageProcessor.cpp
	./unit-test-1 

test2:
	g++ -ggdb -DARDUINO -I./mocks -o unit-test-2 unit-test-2.cpp \
	../WARS-Birdhouse/OutboundPacket.cpp \
	../WARS-Birdhouse/OutboundPacketManager.cpp \
	../WARS-Birdhouse/RoutingTable.cpp \
	../WARS-Birdhouse/MessageProcessor.cpp \
	../WARS-Birdhouse/CommandProcessor.cpp \
	../WARS-Birdhouse/Utils.cpp
	./unit-test-2 

test3:
	g++ -ggdb -DARDUINO -I./mocks -o unit-test-3 unit-test-3.cpp \
	../WARS-Birdhouse/Utils.cpp \
	./mocks/Arduino.cpp	
	./unit-test-3

