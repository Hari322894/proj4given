#ifndef BUSSYSTEMINDEXER_H
#define BUSSYSTEMINDEXER_H

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "BusSystem.h"
#include "StreetMap.h"

class CBusSystemIndexer {
public:
    using TNodeID = CStreetMap::TNodeID;
    using SStop = CBusSystem::SStop;
    using SRoute = CBusSystem::SRoute;

    CBusSystemIndexer(std::shared_ptr<CBusSystem> busSystem);
    ~CBusSystemIndexer();

    std::size_t StopCount() const noexcept;
    std::size_t RouteCount() const noexcept;
    std::shared_ptr<SStop> SortedStopByIndex(std::size_t index) const noexcept;
    std::shared_ptr<SRoute> SortedRouteByIndex(std::size_t index) const noexcept;
    std::shared_ptr<SStop> StopByNodeID(TNodeID id) const noexcept;
    bool RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>> &routes) const noexcept;
    bool RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const noexcept;

private:
    struct SImplementation;
    std::unique_ptr<SImplementation> DImplementation;
};

#endif