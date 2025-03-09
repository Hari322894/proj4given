#ifndef BUSSYSTEM_H
#define BUSSYSTEM_H

#include <string>
#include <vector>
#include <memory>

class CBusSystem {
public:
    using TRouteID = std::string; // Assuming TRouteID is a string based on typical usage
    using TStopID = int; // Assuming TStopID is an integer based on typical usage

    // Define SStop and SRoute
    struct SStop {
        TStopID ID;
        std::string Name;
        TRouteID RouteID;
        // Add more members as necessary
    };

    struct SRoute {
        TRouteID ID;
        std::string Name;
        std::vector<TStopID> Stops;
        // Add more members as necessary
    };

    virtual ~CBusSystem() {}

    virtual std::size_t RouteCount() const noexcept = 0;
    virtual TRouteID GetRouteID(std::size_t index) const noexcept = 0;
    virtual std::size_t StopCount(TRouteID route) const noexcept = 0;
    virtual TStopID GetStopID(TRouteID route, std::size_t stopIndex) const noexcept = 0;
    virtual double StopTime(TRouteID route, std::size_t stopIndex) const noexcept = 0;
    virtual std::string GetRouteName(TRouteID route) const noexcept = 0;

    // Add the missing method declarations
    virtual std::size_t StopCount() const noexcept = 0;
    virtual std::shared_ptr<SStop> StopByIndex(std::size_t index) const noexcept = 0;
    virtual std::shared_ptr<SRoute> RouteByIndex(std::size_t index) const noexcept = 0;
    virtual std::shared_ptr<SStop> StopByID(TStopID stopID) const noexcept = 0;

    static const TRouteID InvalidRouteID; // Assuming InvalidRouteID is a static member
};

#endif