#include "BusSystemIndexer.h"
#include "BusSystem.h"
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm> // Use this instead of bits/stdc++.h

struct CBusSystemIndexer::SImplementation {
    std::shared_ptr<CBusSystem> BusSystem;
    
    SImplementation(std::shared_ptr<CBusSystem> bussystem) {
        BusSystem = bussystem;
    }
    
    std::size_t StopCount() const {
        return BusSystem->StopCount();
    }
    
    std::size_t RouteCount() const {
        return BusSystem->RouteCount();
    }
    
    std::shared_ptr<SStop> SortedStopByIndex(std::size_t index) const {
        if (index >= BusSystem->StopCount()) return nullptr;
        
        std::vector<CBusSystem::TStopID> StopID;
        for (std::size_t i = 0; i < BusSystem->StopCount(); i++) {
            auto stop = BusSystem->StopByIndex(i);
            if (stop) {
                StopID.push_back(stop->ID());
            }
        }
        
        std::sort(StopID.begin(), StopID.end());
        if (index < StopID.size()) {
            return BusSystem->StopByID(StopID[index]);
        }
        return nullptr;
    }
    
    std::shared_ptr<SRoute> SortedRouteByIndex(std::size_t index) const {
        if (index >= BusSystem->RouteCount()) return nullptr;
        
        std::vector<std::string> RouteID;
        for (std::size_t i = 0; i < BusSystem->RouteCount(); i++) {
            auto route = BusSystem->RouteByIndex(i);
            if (route) {
                RouteID.push_back(route->Name());
            }
        }
        
        std::sort(RouteID.begin(), RouteID.end());
        if (index < RouteID.size()) {
            return BusSystem->RouteByName(RouteID[index]);
        }
        return nullptr;
    }
    
    std::shared_ptr<SStop> StopByNodeID(TNodeID id) const {
        for (std::size_t i = 0; i < BusSystem->StopCount(); i++) {
            auto stop = BusSystem->StopByIndex(i);
            if (stop && stop->NodeID() == id) {
                return stop;
            }
        }
        return nullptr;
    }
    
    bool RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>> &routes) const {
        auto srcStop = StopByNodeID(src);
        auto destStop = StopByNodeID(dest);
        
        if (!srcStop || !destStop) {
            return false;
        }
        
        CBusSystem::TStopID srcStopID = srcStop->ID();
        CBusSystem::TStopID destStopID = destStop->ID();
        
        for (std::size_t i = 0; i < BusSystem->RouteCount(); i++) {
            auto route = BusSystem->RouteByIndex(i);
            if (!route) continue;
            
            bool foundSrc = false;
            bool foundDest = false;
            
            for (std::size_t j = 0; j < route->StopCount(); j++) {
                if (route->GetStopID(j) == srcStopID) {
                    foundSrc = true;
                }
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
        
        return !routes.empty();
    }
    
    bool RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const {
        std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> Routes;
        return RoutesByNodeIDs(src, dest, Routes);
    }
};

CBusSystemIndexer::CBusSystemIndexer(std::shared_ptr<CBusSystem> bussystem) {
    DImplementation = std::make_unique<SImplementation>(bussystem);
}

CBusSystemIndexer::~CBusSystemIndexer() {}

std::size_t CBusSystemIndexer::StopCount() const noexcept {
    return DImplementation->StopCount();
}

std::size_t CBusSystemIndexer::RouteCount() const noexcept {
    return DImplementation->RouteCount();
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
    return DImplementation->RouteBetweenNodeIDs(src, dest);
}