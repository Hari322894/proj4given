#include "BusSystemIndexer.h"

struct CBusSystemIndexer::SImplementation {
    std::shared_ptr<CBusSystem> BusSystem;

    SImplementation(std::shared_ptr<CBusSystem> busSystem) : BusSystem(busSystem) {}

    std::shared_ptr<CBusSystem::SStop> SortedStopByIndex(std::size_t index) const {
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

    std::shared_ptr<CBusSystem::SRoute> SortedRouteByIndex(std::size_t index) const {
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

    std::shared_ptr<CBusSystem::SStop> StopByNodeID(CBusSystemIndexer::TNodeID nodeid) const {
        for (std::size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto stop = BusSystem->StopByIndex(i);
            if (stop->NodeID() == nodeid) {
                return stop;
            }
        }
        return nullptr;
    }

    bool RoutesByNodeIDs(CBusSystemIndexer::TNodeID src, CBusSystemIndexer::TNodeID dest, std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> &routes) const {
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
};

CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> busSystem) : DImplementation(std::make_unique<SImplementation>(busSystem)) {}

CBusSystemIndexer::~CBusSystemIndexer() = default;

std::size_t CBusSystemIndexer::StopCount() const noexcept {
    return DImplementation->BusSystem->StopCount();
}

std::size_t CBusSystemIndexer::RouteCount() const noexcept {
    return DImplementation->BusSystem->RouteCount();
}

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedStopByIndex(index);
}

std::shared_ptr<CBusSystem::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedRouteByIndex(index);
}

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::StopByNodeID(TNodeID id) const noexcept {
    return DImplementation->StopByNodeID(id);
}

bool CBusSystemIndexer::RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> &routes) const noexcept {
    return DImplementation->RoutesByNodeIDs(src, dest, routes);
}

bool CBusSystemIndexer::RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const noexcept {
    std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> routes;
    return RoutesByNodeIDs(src, dest, routes);
}