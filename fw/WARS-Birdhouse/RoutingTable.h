#ifndef _RoutingTable_h
#define _RoutingTable_h

#include "Utils.h"

class RoutingTable {
public:

    static nodeaddr_t NO_ROUTE;

    nodeaddr_t nextHop(nodeaddr_t finalDestAddr);
};

#endif
