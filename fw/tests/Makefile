tests: test1 test2 test3

test1:
	g++ -ggdb -DARDUINO -I./mocks -o unit-test-1 unit-test-1.cpp \
	../station/Utils.cpp \
	../station/OutboundPacket.cpp \
	../station/OutboundPacketManager.cpp \
	../station/RoutingTable.cpp \
	../station/RoutingTableImpl.cpp \
	../station/MessageProcessor.cpp \
	./mocks/Arduino.cpp	
	./unit-test-1 

test2:
	g++ -ggdb -DARDUINO -I./mocks -o unit-test-2 unit-test-2.cpp \
	../station/OutboundPacket.cpp \
	../station/OutboundPacketManager.cpp \
	../station/RoutingTable.cpp \
	../station/RoutingTableImpl.cpp \
	../station/MessageProcessor.cpp \
	../station/CommandProcessor.cpp \
	../station/Utils.cpp \
	./mocks/Arduino.cpp	
	./unit-test-2 

test3:
	g++ -ggdb -DARDUINO -I./mocks -o unit-test-3 unit-test-3.cpp \
	../station/Utils.cpp \
	../station/ConfigurationImpl.cpp \
	./mocks/Arduino.cpp	
	./unit-test-3

