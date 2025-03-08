#include "BusSystemIndexer.h"
#include <algorithm>
#include <unordered_map>
#include <vector>

struct CBusSystemIndexer::SImplementation {
    std::shared_ptr<CBusSystem> DBusSystem;
    std::vector<std::shared_ptr<SStop>> DStops;
    std::vector<std::shared_ptr<SRoute>> DRoutes;
    std::unordered_map<TNodeID, std::shared_ptr<SStop>> DNodeIDToStop;
    std::unordered_map<TNodeID, std::unordered_set<std::shared_ptr<SRoute>>> DNodeIDToRoutes;
    
    SImplementation(std::shared_ptr<CBusSystem> busSystem) : DBusSystem(busSystem) {
        // Index stops and routes for faster lookups
        IndexStops();
        IndexRoutes();
    }
    
    void IndexStops() {
        // Get all stops from the bus system
        for(std::size_t i = 0; i < DBusSystem->StopCount(); i++) {
            auto stop = DBusSystem->StopByIndex(i);
            DStops.push_back(stop);
            DNodeIDToStop[stop->NodeID()] = stop;
        }
        
        // Sort stops by ID for easier lookup
        std::sort(DStops.begin(), DStops.end(), 
                 [](const std::shared_ptr<SStop>& a, const std::shared_ptr<SStop>& b) {
                     return a->ID() < b->ID();
                 });
    }
    
    void IndexRoutes() {
        // Get all routes from the bus system
        for(std::size_t i = 0; i < DBusSystem->RouteCount(); i++) {
            auto route = DBusSystem->RouteByIndex(i);
            DRoutes.push_back(route);
            
            // Create node ID to routes mapping
            for(std::size_t j = 0; j < route->StopCount(); j++) {
                auto stop = route->StopByIndex(j);
                DNodeIDToRoutes[stop->NodeID()].insert(route);
                
                // Map adjacent stops for fast route finding
                if(j > 0) {
                    auto prevStop = route->StopByIndex(j-1);
                    DNodeIDToRoutes[prevStop->NodeID()].insert(route);
                }
            }
        }
        
        // Sort routes by name for easier lookup
        std::sort(DRoutes.begin(), DRoutes.end(), 
                 [](const std::shared_ptr<SRoute>& a, const std::shared_ptr<SRoute>& b) {
                     return a->Name() < b->Name();
                 });
    }
};

CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> bussystem)
    : DImplementation(std::make_unique<SImplementation>(bussystem)) {
}

CBusSystemIndexer::~CBusSystemIndexer() = default;

std::size_t CBusSystemIndexer::StopCount() const noexcept {
    return DImplementation->DStops.size();
}

std::size_t CBusSystemIndexer::RouteCount() const noexcept {
    return DImplementation->DRoutes.size();
}

std::shared_ptr<CBusSystemIndexer::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    if(index < DImplementation->DStops.size()) {
        return DImplementation->DStops[index];
    }
    return nullptr;
}

std::shared_ptr<CBusSystemIndexer::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    if(index < DImplementation->DRoutes.size()) {
        return DImplementation->DRoutes[index];
    }
    return nullptr;
}

std::shared_ptr<CBusSystemIndexer::SStop> CBusSystemIndexer::StopByNodeID(TNodeID id) const noexcept {
    auto it = DImplementation->DNodeIDToStop.find(id);
    if(it != DImplementation->DNodeIDToStop.end()) {
        return it->second;
    }
    return nullptr;
}

bool CBusSystemIndexer::RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>>& routes) const noexcept {
    routes.clear();
    
    auto srcRoutesIt = DImplementation->DNodeIDToRoutes.find(src);
    auto destRoutesIt = DImplementation->DNodeIDToRoutes.find(dest);
    
    if(srcRoutesIt == DImplementation->DNodeIDToRoutes.end() || 
       destRoutesIt == DImplementation->DNodeIDToRoutes.end()) {
        return false;
    }
    
    // Find common routes between source and destination
    for(const auto& route : srcRoutesIt->second) {
        if(destRoutesIt->second.find(route) != destRoutesIt->second.end()) {
            // Check if source comes before destination in the route
            bool srcFound = false;
            for(std::size_t i = 0; i < route->StopCount(); i++) {
                auto stop = route->StopByIndex(i);
                if(stop->NodeID() == src) {
                    srcFound = true;
                } else if(stop->NodeID() == dest && srcFound) {
                    routes.insert(route);
                    break;
                }
            }
        }
    }
    
    return !routes.empty();
}

bool CBusSystemIndexer::RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const noexcept {
    std::unordered_set<std::shared_ptr<SRoute>> routes;
    return RoutesByNodeIDs(src, dest, routes);
}