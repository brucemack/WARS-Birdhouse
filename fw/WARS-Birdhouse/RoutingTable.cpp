#include "RoutingTable.h"

nodeaddr_t RoutingTable::NO_ROUTE = 0;

nodeaddr_t RoutingTable::nextHop(nodeaddr_t finalDestAddr) {
    return NO_ROUTE;
}

