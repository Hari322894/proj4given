#include "CSVBusSystem.h"
#include <iostream>
#include <iomanip>

CCSVBusSystem::CCSVBusSystem(std::shared_ptr<CDSVReader> stopReader, std::shared_ptr<CDSVReader> routeReader) {
    // Initialize stops
    while (stopReader->ReadRow()) {
        auto stop = std::make_shared<SStop>();
        stop->ID = std::stoul(stopReader->GetCell(0));
        stop->Name() = stopReader->GetCell(1);
        stop->Latitude() = std::stod(stopReader->GetCell(2));
        stop->Longitude() = std::stod(stopReader->GetCell(3));
        stop->NodeIDValue = std::stoul(stopReader->GetCell(4));
        DStops.push_back(stop);
        DStopIDMap[stop->ID()] = stop;
    }

    // Initialize routes
    while (routeReader->ReadRow()) {
        auto route = std::make_shared<SRoute>();
        route->Name() = routeReader->GetCell(0);
        for (size_t i = 1; i < routeReader->GetCellCount(); ++i) {
            route->StopIDs.push_back(std::stoul(routeReader->GetCell(i)));
        }
        DRoutes.push_back(route);
        DRouteNameMap[route->Name()] = route;
    }
}

CCSVBusSystem::~CCSVBusSystem() {}

std::size_t CCSVBusSystem::StopCount() const noexcept {
    return DStops.size();
}

std::size_t CCSVBusSystem::RouteCount() const noexcept {
    return DRoutes.size();
}

std::shared_ptr<CCSVBusSystem::SStop> CCSVBusSystem::StopByIndex(std::size_t index) const noexcept {
    if (index >= DStops.size()) {
        return nullptr;
    }
    return DStops[index];
}

std::shared_ptr<CCSVBusSystem::SRoute> CCSVBusSystem::RouteByIndex(std::size_t index) const noexcept {
    if (index >= DRoutes.size()) {
        return nullptr;
    }
    return DRoutes[index];
}

std::shared_ptr<CCSVBusSystem::SStop> CCSVBusSystem::StopByID(TStopID id) const noexcept {
    auto it = DStopIDMap.find(id);
    if (it != DStopIDMap.end()) {
        return it->second;
    }
    return nullptr;
}

std::shared_ptr<CCSVBusSystem::SRoute> CCSVBusSystem::RouteByName(const std::string &name) const noexcept {
    auto it = DRouteNameMap.find(name);
    if (it != DRouteNameMap.end()) {
        return it->second;
    }
    return nullptr;
}

CBusSystem::TStopID CCSVBusSystem::SStop::ID() const noexcept {
    return ID;
}

std::string CCSVBusSystem::SStop::Name() const noexcept {
    return Name;
}

double CCSVBusSystem::SStop::Latitude() const noexcept {
    return Latitude;
}

double CCSVBusSystem::SStop::Longitude() const noexcept {
    return Longitude;
}

CStreetMap::TNodeID CCSVBusSystem::SStop::NodeID() const noexcept {
    return NodeIDValue;
}

std::string CCSVBusSystem::SRoute::Name() const noexcept {
    return Name;
}

std::size_t CCSVBusSystem::SRoute::StopCount() const noexcept {
    return StopIDs.size();
}

CBusSystem::TStopID CCSVBusSystem::SRoute::GetStopID(std::size_t index) const noexcept {
    if (index >= StopIDs.size()) {
        return CBusSystem::InvalidStopID;
    }
    return StopIDs[index];
}

std::ostream &operator<<(std::ostream &os, const CCSVBusSystem &busSystem) {
    os << "Stops: " << std::endl;
    for (size_t i = 0; i < busSystem.DStops.size(); ++i) {
        auto stop = busSystem.DStops[i];
        os << "Index " << i << " ID: " << stop->ID() << " Name: " << stop->Name() << " Latitude: " << stop->Latitude() << " Longitude: " << stop->Longitude() << " NodeID: " << stop->NodeID() << std::endl;
    }

    os << "Routes: " << std::endl;
    for (size_t i = 0; i < busSystem.DRoutes.size(); ++i) {
        auto route = busSystem.DRoutes[i];
        os << "Index " << i << " Name: " << route->Name() << " StopCount: " << route->StopCount() << std::endl;
    }

    return os;
}