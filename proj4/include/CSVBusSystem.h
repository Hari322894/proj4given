#ifndef CSVBUSSYSTEM_H
#define CSVBUSSYSTEM_H

#include <string>
#include <vector>
#include <memory>
#include "BusSystem.h"
#include "DSVReader.h"

class CCSVBusSystem : public CBusSystem {
public:
    class SStop : public CBusSystem::SStop {
    public:
        TStopID ID() const noexcept override;
        std::string Name() const noexcept override;
        double Latitude() const noexcept override;
        double Longitude() const noexcept override;
        CStreetMap::TNodeID NodeID() const noexcept override;

        CStreetMap::TNodeID NodeIDValue; // corresponding node ID in the street map
    };

    class SRoute : public CBusSystem::SRoute {
    public:
        std::string Name() const noexcept override;
        std::size_t StopCount() const noexcept override;
        TStopID GetStopID(std::size_t index) const noexcept override;

        std::vector<TStopID> StopIDs;
    };

    CCSVBusSystem(std::shared_ptr<CDSVReader> stopReader, std::shared_ptr<CDSVReader> routeReader);
    ~CCSVBusSystem();

    std::size_t StopCount() const noexcept override;
    std::size_t RouteCount() const noexcept override;
    std::shared_ptr<SStop> StopByIndex(std::size_t index) const noexcept override;
    std::shared_ptr<SRoute> RouteByIndex(std::size_t index) const noexcept override;
    std::shared_ptr<SStop> StopByID(TStopID id) const noexcept override;
    std::shared_ptr<SRoute> RouteByName(const std::string &name) const noexcept override;

private:
    std::vector<std::shared_ptr<SStop>> DStops;
    std::vector<std::shared_ptr<SRoute>> DRoutes;
    std::unordered_map<TStopID, std::shared_ptr<SStop>> DStopIDMap;
    std::unordered_map<std::string, std::shared_ptr<SRoute>> DRouteNameMap;
};

std::ostream &operator<<(std::ostream &os, const CCSVBusSystem &busSystem);

#endif