#ifndef _RoutingTable_h
#define _RoutingTable_h

#include "Utils.h"

class RoutingTable {
public:

    static nodeaddr_t NO_ROUTE;

    virtual nodeaddr_t nextHop(nodeaddr_t finalDestAddr) = 0;

    virtual void setRoute(nodeaddr_t target, nodeaddr_t nextHop) = 0;
};

#endif
