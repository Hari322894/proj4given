#include "CSVBusSystem.h"  //this includes the class definition for CCSVBusSystem
#include "DSVReader.h"    //reading CSV (or other DSV) formatted input files
#include <memory>           //provides std::shared_ptr and std::make_shared for memory management
#include <vector>           //used to store lists of stops and routes
#include <string>           //enables usage of std::string for the route name and attribute
#include <unordered_map>    //lookup of stops and routes by ID or name using unordered_map
#include <iostream>         //i need to print bus system details using operator <<

//defines the SStop class, representing a bus stop, inheriting from CBusSystem::SStop can be seen in header file
class CCSVBusSystem::SStop : public CBusSystem::SStop {
public:
    TStopID StopID;  // unique identifier for the stop
    CStreetMap::TNodeID NodeIDValue;  // corresponding node ID in the street map

    //returns the stop ID
    TStopID ID() const noexcept override {
        return StopID;
    }

    //returns the node ID associated with this stop
    CStreetMap::TNodeID NodeID() const noexcept override {
        return NodeIDValue;
    }
};

// defines the SRoute class, representing a bus route, inheriting from CBusSystem::SRoute
class CCSVBusSystem::SRoute : public CBusSystem::SRoute {
public:
//name of the route
    std::string RouteName;  
//list of stop IDs forming the route
    std::vector<TStopID> RouteStops;  

    //return the route name
    std::string Name() const noexcept override {
        return RouteName;
    }

    // return the number of stops in the route
    std::size_t StopCount() const noexcept override {
        return RouteStops.size();
    }

    // return the stop ID at the given index
    TStopID GetStopID(std::size_t index) const noexcept override {
        if (index >= RouteStops.size()) {
            // return an invalid ID if index is out of bounds
            return CBusSystem::InvalidStopID;  
        }
        return RouteStops[index];
    }
};

// defines the internl impl structure
struct CCSVBusSystem::SImplementation {
    // stores stops in order of input
    std::vector<std::shared_ptr<SStop>> StopsByIndex;  
    // maps stop IDs to stop objs
    std::unordered_map<TStopID, std::shared_ptr<SStop>> Stops;  
     //stores routs in order of input
    std::vector<std::shared_ptr<SRoute>> RoutesByIndex; 
    //maps routs names to route objs
    std::unordered_map<std::string, std::shared_ptr<SRoute>> Routes;  
};

// constructor for the bus system
CCSVBusSystem::CCSVBusSystem(std::shared_ptr<CDSVReader> stopsrc, std::shared_ptr<CDSVReader> routesrc) {
    // Initialize the implementation
    DImplementation = std::make_unique<SImplementation>();
    
    //read stops data
    if (stopsrc) {
        // tmp storage for row data
        std::vector<std::string> stopRow;
        //read each row of the stop file with a while loop
        while (stopsrc->ReadRow(stopRow)) {
            // ensure sufficient columns exists
            if (stopRow.size() >= 2) {  
                try {
                    auto stop = std::make_shared<SStop>();
                    //convert first column to StopID(0 is first in index)
                    stop->StopID = std::stoul(stopRow[0]);  
                    //convert second column to NodeID(1 is second in index)
                    stop->NodeIDValue = std::stoul(stopRow[1]);  
                    //store in map for quick lookup
                    DImplementation->Stops[stop->StopID] = stop; 
                    //store in list for indexed access 
                    DImplementation->StopsByIndex.push_back(stop);  
                } catch (const std::exception& e) {
                    //handle error by 
                    std::cerr << "Exception in stop parsing: " << e.what() << "\n";
                }
            }
        }
    }

    // read routes data
    if (routesrc) {
        // tmp storage for row data
        std::vector<std::string> routeRow;
        // tmp storage for routes by name
        std::unordered_map<std::string, std::shared_ptr<SRoute>> tempRoutes;
        
        // read each row of the route file
        while (routesrc->ReadRow(routeRow)) {  
            // ensure sufficient columns exist
            if (routeRow.size() >= 2) {  
                try {
                    //first column is route name
                    std::string routeName = routeRow[0];  
                    //second column is StopID
                    TStopID stopID = std::stoul(routeRow[1]);  
                    
                    // retrieve or create route entry
                    auto routeIt = tempRoutes.find(routeName);
                    if (routeIt == tempRoutes.end()) {
                        // Create new route
                        auto newRoute = std::make_shared<SRoute>();
                        newRoute->RouteName = routeName;
                        newRoute->RouteStops.push_back(stopID);
                        tempRoutes[routeName] = newRoute;
                    } else {
                        // Add stop to existing route
                        routeIt->second->RouteStops.push_back(stopID);
                    }
                } catch (const std::exception& e) {
                    //handle error by 
                    std::cerr << "Exception in route parsing: " << e.what() << "\n";
                }
            }
        }

        // Transfer routes from temporary map to implementation
        for (const auto& pair : tempRoutes) {
            // store in route map
            DImplementation->Routes[pair.first] = pair.second; 
            // store in index list 
            DImplementation->RoutesByIndex.push_back(pair.second);  
        }
    }
    
    // Debug output
    std::cerr << "Loaded " << DImplementation->StopsByIndex.size() << " stops and " 
              << DImplementation->RoutesByIndex.size() << " routes" << std::endl;
}

// destructor
CCSVBusSystem::~CCSVBusSystem() = default;

// return the total number of stops
std::size_t CCSVBusSystem::StopCount() const noexcept {
    return DImplementation ? DImplementation->StopsByIndex.size() : 0;
}

// return the total number of routes
std::size_t CCSVBusSystem::RouteCount() const noexcept {
    return DImplementation ? DImplementation->RoutesByIndex.size() : 0;
}

// return a stop by index
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByIndex(std::size_t index) const noexcept {
    if (!DImplementation || index >= DImplementation->StopsByIndex.size()) {
        return nullptr;
    }
    return DImplementation->StopsByIndex[index];
}

// return a stop by its ID
std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByID(TStopID id) const noexcept {
    if (!DImplementation) {
        return nullptr;
    }
    auto it = DImplementation->Stops.find(id);
    if (it != DImplementation->Stops.end()) {
        return it->second;
    }
    return nullptr;
}

// return a route by index
std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByIndex(std::size_t index) const noexcept {
    if (!DImplementation || index >= DImplementation->RoutesByIndex.size()) {
        return nullptr;
    }
    return DImplementation->RoutesByIndex[index];
}

// return a route by name
std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByName(const std::string &name) const noexcept {
    if (!DImplementation) {
        return nullptr;
    }
    auto it = DImplementation->Routes.find(name);
    if (it != DImplementation->Routes.end()) {
        return it->second;
    }
    return nullptr;
}

//overloads operator<< in order to print the bus system details
std::ostream &operator<<(std::ostream &os, const CCSVBusSystem &bussystem) {
    os << "StopCount: " << std::to_string(bussystem.StopCount()) << "\n";
    os << "RouteCount: " << std::to_string(bussystem.RouteCount()) << "\n";
    
    for (size_t i = 0; i < bussystem.StopCount(); i++) {
        auto stop = bussystem.StopByIndex(i);
        if (stop) {
            os << "Index " << std::to_string(i) << " ID: " << std::to_string(stop->ID()) <<
                  " NodeID: " << std::to_string(stop->NodeID()) << "\n";
        }
    }
    
    for (size_t i = 0; i < bussystem.RouteCount(); i++) {
        auto route = bussystem.RouteByIndex(i);
        if (route) {
            os << "Route Index " << std::to_string(i) << " Name: " << route->Name() +
                  " StopCount: " << std::to_string(route->StopCount()) << "\n";
        }
    }
    
    return os;
}