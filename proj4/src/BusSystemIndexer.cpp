#include "BusSystemIndexer.h"
#include "BusSystem.h"
#include <bits/stdc++.h>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

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
        std::vector<int> StopID;
        for (std::size_t i = 0; i < BusSystem->StopCount(); i++) {
            StopID.push_back(BusSystem->StopByIndex(i)->ID());
        }
        sort(StopID.begin(), StopID.end());
        return BusSystem->StopByID(StopID[index]);
    }

    std::shared_ptr<SRoute> SortedRouteByIndex(std::size_t index) const {
        if (index >= BusSystem->RouteCount()) return nullptr;
        std::vector<std::string> RouteID;
        for (std::size_t i = 0; i < BusSystem->RouteCount(); i++) {
            RouteID.push_back(BusSystem->RouteByIndex(i)->Name());
        }
        sort(RouteID.begin(), RouteID.end());
        return BusSystem->RouteByName(RouteID[index]);
    }

    std::shared_ptr<SStop> StopByNodeID(TNodeID id) const {
        for (std::size_t i = 0; i < BusSystem->StopCount(); i++) {
            if (BusSystem->StopByIndex(i)->NodeID() == id) {
                return BusSystem->StopByIndex(i);
            }
        }
        return nullptr;
    }

    bool RoutesByNodeIDs(TNodeID src, TNodeID dest, std::unordered_set<std::shared_ptr<SRoute>> &routes) const {
        CBusSystem::TStopID s = StopByNodeID(src)->ID();
        CBusSystem::TStopID d = StopByNodeID(dest)->ID();
        int count = 0;
        for (std::size_t i = 0; i < BusSystem->RouteCount(); i++) {
            count = 0;
            for (std::size_t j = 0; j < BusSystem->RouteByIndex(i)->StopCount(); j++) {
                if (BusSystem->RouteByIndex(i)->GetStopID(j) == s) count++;
                if (BusSystem->RouteByIndex(i)->GetStopID(j) == d) count++;
            }
            if (count >= 2) {
                routes.insert(BusSystem->RouteByIndex(i));
            }
        }

        return !routes.empty();
    }

    bool RouteBetweenNodeIDs(TNodeID src, TNodeID dest) const {
        std::unordered_set<std::shared_ptr<CBusSystem::SRoute>> Routes;
        RoutesByNodeIDs(src, dest, Routes);
        return !Routes.empty();
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