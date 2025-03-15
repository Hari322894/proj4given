#include "BusSystemIndexer.h" // including header file for CBusSystemIndexer class usage
#include "BusSystem.h" // including header file for CBusSystem class usage
#include <memory>//provides std::shared_ptr and std::make_shared for memory management
#include <string>//enables usage of std::string for the route name and attribute
#include <iostream>//i need to print bus system details using operator <<
#include <vector>//used to store lists of stops and routes
#include <algorithm>//used to sort the stops and routes

struct CBusSystemIndexer::SImplementation {
    // shared pointer to the bus system
    std::shared_ptr<CBusSystem> BusSystem;
    // constructor for the implementation
    SImplementation(std::shared_ptr<CBusSystem> bussystem) {
        BusSystem = bussystem;
    }
    // return the total number of stops
    std::size_t StopCount() const {
        return BusSystem->StopCount();
    }
    // return the total number of routes
    std::size_t RouteCount() const {
        return BusSystem->RouteCount();
    }
    // return a stop by index
    std::shared_ptr<SStop> SortedStopByIndex(std::size_t index) const {
        if (index >= BusSystem->StopCount()) return nullptr;
        // vector to store stop IDs
        std::vector<CBusSystem::TStopID> StopID;
        for (std::size_t i = 0; i < BusSystem->StopCount(); i++) {
            auto stop = BusSystem->StopByIndex(i);
            // if stop is not null, add the stop ID to the vector
            if (stop) {
                StopID.push_back(stop->ID());
            }
        }
        // sort the stop IDs
        std::sort(StopID.begin(), StopID.end());
        if (index < StopID.size()) {
            return BusSystem->StopByID(StopID[index]);
        }
        // return null if index is out of bounds
        return nullptr;
    }
    // return a route by index
    std::shared_ptr<SRoute> SortedRouteByIndex(std::size_t index) const {
        if (index >= BusSystem->RouteCount()) return nullptr;
        // vector to store route IDs
        std::vector<std::string> RouteID;
        for (std::size_t i = 0; i < BusSystem->RouteCount(); i++) {
            auto route = BusSystem->RouteByIndex(i);
            if (route) {
                RouteID.push_back(route->Name());
            }
        }
        // sort the route IDs
        std::sort(RouteID.begin(), RouteID.end());
        if (index < RouteID.size()) {
            // return the route by name
            return BusSystem->RouteByName(RouteID[index]);
        }
        return nullptr;
    }
    // return a stop by its node ID
    std::shared_ptr<SStop> StopByNodeID(TNodeID id) const {
        for (std::size_t i = 0; i < BusSystem->StopCount(); i++) {
            auto stop = BusSystem->StopByIndex(i);
            if (stop && stop->NodeID() == id) {
                // return the stop if the node ID matches
                return stop;
            }
        }
        return nullptr;
    }
    // return the routes between two node IDs
    bool RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>> &routes) const {
        auto srcStop = StopByNodeID(src);
        auto destStop = StopByNodeID(dest);
       //  return false if either stop is null 
        if (!srcStop || !destStop) {
            return false;
        }
        // set the stop IDs
        CBusSystem::TStopID srcStopID = srcStop->ID();
        CBusSystem::TStopID destStopID = destStop->ID();
        // iterate through the routes
        for (std::size_t i = 0; i < BusSystem->RouteCount(); i++) {
            auto route = BusSystem->RouteByIndex(i);
            if (!route) continue;
        //  set flags to check if the source and destination stops are found    
            bool foundSrc = false;
            bool foundDest = false;
        //  iterate through the stops in the route    
            for (std::size_t j = 0; j < route->StopCount(); j++) {
                if (route->GetStopID(j) == srcStopID) {
                    foundSrc = true;
                }
        //  break if the destination stop is found
                if (route->GetStopID(j) == destStopID) {
                    foundDest = true;
                }
                
                // If we found both stops, add the route to the set
                if (foundSrc && foundDest) {
                    routes.insert(route);
                    break;
                }
            }
        }
        // return true if the set is not empty
        return !routes.empty();
    }
    // return true if there is a route between two node IDs
    bool RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const {
        std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> Routes;
        return RoutesByNodeIDs(src, dest, Routes);
    }
};
// constructor for the bus system indexer
CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> bussystem) {
    DImplementation = std::make_unique<SImplementation>(bussystem);
}
// destructor for the bus system indexer
CBusSystemIndexer::~CBusSystemIndexer() {}
// return the total number of stops
std::size_t CBusSystemIndexer::StopCount() const noexcept {
    return DImplementation->StopCount();
}
// return the total number of routes
std::size_t CBusSystemIndexer::RouteCount() const noexcept {
    return DImplementation->RouteCount();
}
// return a stop by index
std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::SortedStopByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedStopByIndex(index);
}
// return a route by index
std::shared_ptr<CBusSystem::SRoute> CBusSystemIndexer::SortedRouteByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedRouteByIndex(index);
}
// return a stop by its node ID
std::shared_ptr<CBusSystem::SStop> CBusSystemIndexer::StopByNodeID(TNodeID id) const noexcept {
    return DImplementation->StopByNodeID(id);
}
// return the routes between two node IDs
bool CBusSystemIndexer::RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> &routes) const noexcept {
    return DImplementation->RoutesByNodeIDs(src, dest, routes);
}
// return true if there is a route between two node IDs
bool CBusSystemIndexer::RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const noexcept {
    return DImplementation->RouteBetweenNodeIDs(src, dest);
}