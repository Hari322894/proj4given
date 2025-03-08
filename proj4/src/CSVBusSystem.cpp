#include "CSVBusSystem.h"
#include "DSVReader.h"
#include <unordered_map>
#include <vector>
#include <memory>

// Implementation of CCSVBusSystem
struct CCSVBusSystem::SImplementation {
    std::shared_ptr<CDSVReader> StopsReader;
    std::shared_ptr<CDSVReader> RoutesReader;
    std::unordered_map<TStopID, std::shared_ptr<SStop>> Stops;
    std::unordered_map<std::string, std::shared_ptr<SRoute>> Routes;
};

class CCSVBusSystem::SStop : public CBusSystem::SStop {
private:
    TStopID DStopID;
    CStreetMap::TNodeID DNodeID;

public:
    SStop(TStopID stopid, CStreetMap::TNodeID nodeid) : DStopID(stopid), DNodeID(nodeid) {}

    virtual TStopID ID() const noexcept override {
        return DStopID;
    }

    virtual CStreetMap::TNodeID NodeID() const noexcept override {
        return DNodeID;
    }
};

class CCSVBusSystem::SRoute : public CBusSystem::SRoute {
private:
    std::string DName;
    std::vector<TStopID> DStops;

public:
    SRoute(const std::string &name) : DName(name) {}

    virtual std::string Name() const noexcept override {
        return DName;
    }

    virtual std::size_t StopCount() const noexcept override {
        return DStops.size();
    }

    virtual TStopID GetStopID(std::size_t index) const noexcept override {
        if(index < DStops.size()) {
            return DStops[index];
        }
        return InvalidStopID;
    }

    void AddStop(TStopID stopid) {
        DStops.push_back(stopid);
    }
};

CCSVBusSystem::CCSVBusSystem(std::shared_ptr<CDSVReader> stopsreader, std::shared_ptr<CDSVReader> routesreader) {
    DImplementation = std::make_unique<SImplementation>();
    DImplementation->StopsReader = stopsreader;
    DImplementation->RoutesReader = routesreader;
    
    // Read stops
    std::vector<std::string> row;
    if(DImplementation->StopsReader->ReadRow(row)) {
        // Skip header row, read data rows
        while(DImplementation->StopsReader->ReadRow(row)) {
            if(row.size() >= 2) {
                TStopID stopID = std::stoul(row[0]);
                CStreetMap::TNodeID nodeID = std::stoul(row[1]);
                auto stop = std::make_shared<SStop>(stopID, nodeID);
                DImplementation->Stops[stopID] = stop;
            }
        }
    }
    
    // Read routes
    if(DImplementation->RoutesReader->ReadRow(row)) {
        // Skip header row
        std::string currentRoute;
        std::shared_ptr<SRoute> currentRoutePtr;
        
        while(DImplementation->RoutesReader->ReadRow(row)) {
            if(row.size() >= 2) {
                std::string routeName = row[0];
                TStopID stopID = std::stoul(row[1]);
                
                if(routeName != currentRoute) {
                    currentRoute = routeName;
                    if(DImplementation->Routes.find(routeName) == DImplementation->Routes.end()) {
                        currentRoutePtr = std::make_shared<SRoute>(routeName);
                        DImplementation->Routes[routeName] = currentRoutePtr;
                    } else {
                        currentRoutePtr = DImplementation->Routes[routeName];
                    }
                }
                
                if(currentRoutePtr) {
                    currentRoutePtr->AddStop(stopID);
                }
            }
        }
    }
}

CCSVBusSystem::~CCSVBusSystem() {
    
}

std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByID(TStopID id) const noexcept {
    auto FindStop = DImplementation->Stops.find(id);
    if(FindStop != DImplementation->Stops.end()) {
        return FindStop->second;
    }
    return nullptr;
}

std::size_t CCSVBusSystem::StopCount() const noexcept {
    return DImplementation->Stops.size();
}

std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByIndex(std::size_t index) const noexcept {
    if(index < StopCount()) {
        auto Iterator = DImplementation->Stops.begin();
        std::advance(Iterator, index);
        return Iterator->second;
    }
    return nullptr;
}

std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByName(const std::string &name) const noexcept {
    auto FindRoute = DImplementation->Routes.find(name);
    if(FindRoute != DImplementation->Routes.end()) {
        return FindRoute->second;
    }
    return nullptr;
}

std::size_t CCSVBusSystem::RouteCount() const noexcept {
    return DImplementation->Routes.size();
}

std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByIndex(std::size_t index) const noexcept {
    if(index < RouteCount()) {
        auto Iterator = DImplementation->Routes.begin();
        std::advance(Iterator, index);
        return Iterator->second;
    }
    return nullptr;
}