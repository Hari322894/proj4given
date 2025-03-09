#include "BusSystemIndexer.h"
#include "GeographicUtils.h"
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <memory>

// Define PairHash to be used in unordered_map
struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

struct CBusSystemIndexer::SImplementation {
    std::shared_ptr<CBusSystem> DBusSystem;
    std::vector<std::shared_ptr<CBusSystem::SStop>> DSortedStops;
    std::vector<std::shared_ptr<CBusSystem::SRoute>> DSortedRoutes;
    std::unordered_map<CStreetMap::TNodeID, std::shared_ptr<CBusSystem::SStop>> DNodeToStop;
    std::unordered_map<CStreetMap::TNodeID, std::unordered_set<CStreetMap::TNodeID>> DConnectedStops;
    std::unordered_map<std::pair<CStreetMap::TNodeID, CStreetMap::TNodeID>, 
                      std::unordered_set<std::shared_ptr<CBusSystem::SRoute>>, 
                      PairHash> DConnectedRoutes;

    SImplementation(std::shared_ptr<CBusSystem> bussystem) 
        : DBusSystem(bussystem) {
        
        // Index all stops
        for (size_t i = 0; i < DBusSystem->StopCount(); ++i) {
            auto stop = DBusSystem->StopByIndex(i);
            DSortedStops.push_back(stop);
            DNodeToStop[stop->ID] = stop;
        }
        
        // Sort stops by ID
        std::sort(DSortedStops.begin(), DSortedStops.end(), 
                 [](const std::shared_ptr<CBusSystem::SStop> &lhs, 
                    const std::shared_ptr<CBusSystem::SStop> &rhs) {
                     return lhs->ID < rhs->ID;
                 });
        
        // Index all routes
        for (size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
            auto route = DBusSystem->RouteByIndex(i);
            DSortedRoutes.push_back(route);
            
            // Build connection maps between stops
            for (size_t j = 0; j < route->Stops.size() - 1; ++j) {
                auto srcStopID = route->Stops[j];
                auto srcStop = DBusSystem->StopByID(srcStopID);
                auto srcNodeID = srcStop->ID;
                
                for (size_t k = j + 1; k < route->Stops.size(); ++k) {
                    auto destStopID = route->Stops[k];
                    auto destStop = DBusSystem->StopByID(destStopID);
                    auto destNodeID = destStop->ID;
                    
                    // Add connected stops
                    DConnectedStops[srcNodeID].insert(destNodeID);
                    DConnectedStops[destNodeID].insert(srcNodeID);
                    
                    // Add routes between these stops
                    DConnectedRoutes[std::make_pair(srcNodeID, destNodeID)].insert(route);
                    DConnectedRoutes[std::make_pair(destNodeID, srcNodeID)].insert(route);
                }
            }
        }
        
        // Sort routes by name
        std::sort(DSortedRoutes.begin(), DSortedRoutes.end(), 
                 [](const std::shared_ptr<CBusSystem::SRoute> &lhs, 
                    const std::shared_ptr<CBusSystem::SRoute> &rhs) {
                     return lhs->Name < rhs->Name;
                 });
    }
};

// Constructor implementation
CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> bussystem) {
    DImplementation = std::make_unique<SImplementation>(bussystem);
}

// Destructor implementation
CBusSystemIndexer::~CBusSystemIndexer() {
}

// Returns the number of stops in the bus system
std::size_t CBusSystemIndexer::StopCount() const noexcept {
    return DImplementation->DSortedStops.size();
}

// Returns the number of routes in the bus system
std::size_t CBusSystemIndexer::RouteCount() const noexcept {
    return DImplementation->DSortedRoutes.size();
}

// Returns the stop by index from the sorted stops
std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->DSortedStops.size()) {
        return nullptr;
    }
    return DImplementation->DSortedStops[index];
}

// Returns the route by index from the sorted routes
std::shared_ptr<CBusSystem::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    if (index >= DImplementation->DSortedRoutes.size()) {
        return nullptr;
    }
    return DImplementation->DSortedRoutes[index];
}

// Returns the stop associated with the node ID
std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::StopByNodeID(CStreetMap::TNodeID id) const noexcept {
    auto it = DImplementation->DNodeToStop.find(id);
    if (it == DImplementation->DNodeToStop.end()) {
        return nullptr;
    }
    return it->second;
}

// Returns true if at least one route exists between the stops at src and dest node IDs
bool CBusSystemIndexer::RoutesByNodeIDs(CStreetMap::TNodeID src, CStreetMap::TNodeID dest,
                                       std::unordered_set<std::shared_ptr<CBusSystem::SRoute>>& routes) const noexcept {
    auto it = DImplementation->DConnectedRoutes.find(std::make_pair(src, dest));
    if (it == DImplementation->DConnectedRoutes.end()) {
        return false;
    }
    
    routes = it->second;
    return !routes.empty();
}

// Returns true if at least one route exists between the stops at src and dest node IDs
bool CBusSystemIndexer::RouteBetweenNodeIDs(CStreetMap::TNodeID src, CStreetMap::TNodeID dest) const noexcept {
    std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> routes;
    return RoutesByNodeIDs(src, dest, routes);
}