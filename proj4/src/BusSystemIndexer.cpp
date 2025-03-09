#include "BusSystemIndexer.h"
#include <algorithm>

struct CBusSystemIndexer::SImplementation {
    std::shared_ptr<CBusSystem> DBusSystem;
    std::vector<std::shared_ptr<CBusSystem::SStop>> DSortedStops;
    std::vector<std::shared_ptr<CBusSystem::SRoute>> DSortedRoutes;
    std::unordered_map<CStreetMap::TNodeID, std::shared_ptr<CBusSystem::SStop>> DNodeToStop;
    std::unordered_map<std::pair<CStreetMap::TNodeID, CStreetMap::TNodeID>, std::unordered_set<std::shared_ptr<CBusSystem::SRoute>>, PairHash> DConnectedRoutes;

    SImplementation(std::shared_ptr<CBusSystem> busSystem) : DBusSystem(busSystem) {
        // Initialize stops
        for (size_t i = 0; i < DBusSystem->StopCount(CBusSystem::InvalidRouteID); ++i) {
            auto stop = DBusSystem->StopByIndex(i);
            DSortedStops.push_back(stop);
        }
        std::sort(DSortedStops.begin(), DSortedStops.end(),
            [](const std::shared_ptr<CBusSystem::SStop> &lhs, const std::shared_ptr<CBusSystem::SStop> &rhs) {
                return lhs->ID < rhs->ID;
            });

        // Initialize routes
        for (size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
            auto route = DBusSystem->RouteByIndex(i);
            DSortedRoutes.push_back(route);
        }
        std::sort(DSortedRoutes.begin(), DSortedRoutes.end(),
            [](const std::shared_ptr<CBusSystem::SRoute> &lhs, const std::shared_ptr<CBusSystem::SRoute> &rhs) {
                return lhs->Name < rhs->Name;
            });

        // Initialize node to stop mapping and connected routes
        for (const auto &stop : DSortedStops) {
            DNodeToStop[stop->ID] = stop;
        }

        for (const auto &route : DSortedRoutes) {
            for (size_t i = 0; i < route->Stops.size() - 1; ++i) {
                auto srcStopID = route->Stops[i];
                auto destStopID = route->Stops[i + 1];
                auto srcStop = DBusSystem->StopByID(srcStopID);
                auto destStop = DBusSystem->StopByID(destStopID);

                if (srcStop && destStop) {
                    auto srcNodeID = srcStop->ID;
                    auto destNodeID = destStop->ID;
                    DConnectedRoutes[{srcNodeID, destNodeID}].insert(route);
                    DConnectedRoutes[{destNodeID, srcNodeID}].insert(route);
                }
            }
        }
    }
};

CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> busSystem)
    : DImplementation(std::make_unique<SImplementation>(busSystem)) {
}

CBusSystemIndexer::~CBusSystemIndexer() {
}

std::size_t CBusSystemIndexer::StopCount() const {
    return DImplementation->DSortedStops.size();
}

std::size_t CBusSystemIndexer::RouteCount() const {
    return DImplementation->DSortedRoutes.size();
}

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->DSortedStops.size()) {
        return nullptr;
    }
    return DImplementation->DSortedStops[index];
}

std::shared_ptr<CBusSystem::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->DSortedRoutes.size()) {
        return nullptr;
    }
    return DImplementation->DSortedRoutes[index];
}

std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::StopByNodeID(CStreetMap::TNodeID id) const noexcept {
    auto it = DImplementation->DNodeToStop.find(id);
    if (it != DImplementation->DNodeToStop.end()) {
        return it->second;
    }
    return nullptr;
}

bool CBusSystemIndexer::RoutesByNodeIDs(CStreetMap::TNodeID src, CStreetMap::TNodeID dest, std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> &routes) const noexcept {
    auto it = DImplementation->DConnectedRoutes.find({src, dest});
    if (it != DImplementation->DConnectedRoutes.end()) {
        routes = it->second;
        return true;
    }
    return false;
}

bool CBusSystemIndexer::RouteBetweenNodeIDs(CStreetMap::TNodeID src, CStreetMap::TNodeID dest) const noexcept {
    return DImplementation->DConnectedRoutes.find({src, dest}) != DImplementation->DConnectedRoutes.end();
}