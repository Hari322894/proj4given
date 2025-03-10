#include "BusSystemIndexer.h"

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SImplementation::SortedStopByIndex(std::size_t index) const {
    if (index >= BusSystem->StopCount()) {
        return nullptr;
    }
    for (std::size_t i = 0; i < BusSystem->StopCount(); ++i) {
        if (i == index) {
            return BusSystem->StopByIndex(i);
        }
    }
    return nullptr;
}

std::shared_ptr<CBusSystem::SRoute> CBusSystemIndexer::SImplementation::SortedRouteByIndex(std::size_t index) const {
    if (index >= BusSystem->RouteCount()) {
        return nullptr;
    }
    for (std::size_t i = 0; i < BusSystem->RouteCount(); ++i) {
        if (i == index) {
            return BusSystem->RouteByIndex(i);
        }
    }
    return nullptr;
}

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SImplementation::StopByNodeID(CBusSystemIndexer::TNodeID nodeid) const {
    for (std::size_t i = 0; i < BusSystem->StopCount(); ++i) {
        auto stop = BusSystem->StopByIndex(i);
        if (stop->NodeID() == nodeid) {
            return stop;
        }
    }
    return nullptr;
}

bool CBusSystemIndexer::SImplementation::RoutesByNodeIDs(CBusSystemIndexer::TNodeID src, CBusSystemIndexer::TNodeID dest, std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> &routes) const {
    for (std::size_t i = 0; i < BusSystem->RouteCount(); ++i) {
        auto route = BusSystem->RouteByIndex(i);
        for (std::size_t j = 0; j < route->StopCount(); ++j) {
            if (route->StopByIndex(j)->NodeID() == src) {
                for (std::size_t k = 0; k < route->StopCount(); ++k) {
                    if (route->StopByIndex(k)->NodeID() == dest) {
                        routes.insert(route);
                        break;
                    }
                }
            }
        }
    }
    return !routes.empty();
}