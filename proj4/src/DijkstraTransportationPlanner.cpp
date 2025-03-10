#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <map>
#include <set>
#include <iostream>

// Define missing types
using TBusID = std::size_t;
using TStopID = std::size_t;

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> DConfig;
    std::shared_ptr<CStreetMap> DStreetMap;
    std::shared_ptr<CBusSystem> DBusSystem;
    std::shared_ptr<CDijkstraPathRouter> DPathRouter;
    std::vector<std::shared_ptr<CStreetMap::SNode>> DNodes;
    std::map<TNodeID, size_t> DNodeIDToIndex;

    SImplementation(std::shared_ptr<SConfiguration> config) {
        DConfig = config;
        DStreetMap = config->StreetMap();
        DBusSystem = config->BusSystem();

        // Initialize path router for walking
        DPathRouter = std::make_shared<CDijkstraPathRouter>();

        // Store all nodes for quick lookup
        for (std::size_t i = 0; i < DStreetMap->NodeCount(); ++i) {
            auto node = DStreetMap->NodeByIndex(i);
            if (node) {
                DNodes.push_back(node);
            }
        }

        // Sort nodes by ID for consistent indexing
        std::sort(DNodes.begin(), DNodes.end(), [](const auto &a, const auto &b) { 
            return a->ID() < b->ID(); 
        });

        // Create node ID to index mapping for quick lookups
        for (std::size_t i = 0; i < DNodes.size(); ++i) {
            DNodeIDToIndex[DNodes[i]->ID()] = i;
        }

        // Add vertices to the path router
        for (const auto &node : DNodes) {
            DPathRouter->AddVertex(node->ID());
        }

        // Add street edges
        for (std::size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (!way || way->NodeCount() < 2)
                continue;

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                // Find the nodes by ID
                auto node1Iter = DNodeIDToIndex.find(node1ID);
                auto node2Iter = DNodeIDToIndex.find(node2ID);
                
                if (node1Iter == DNodeIDToIndex.end() || node2Iter == DNodeIDToIndex.end())
                    continue;
                
                std::shared_ptr<CStreetMap::SNode> node1 = DNodes[node1Iter->second];
                std::shared_ptr<CStreetMap::SNode> node2 = DNodes[node2Iter->second];

                double distance = CalculateDistance(node1, node2);
                DPathRouter->AddEdge(node1->ID(), node2->ID(), distance, false);

                // Add edge in reverse if way is bidirectional
                if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                    DPathRouter->AddEdge(node2->ID(), node1->ID(), distance, false);
                }
            }
        }

        // Add bus routes as edges if bus system is available
        if (DBusSystem) {
            for (std::size_t routeIdx = 0; routeIdx < DBusSystem->RouteCount(); ++routeIdx) {
                auto route = DBusSystem->RouteByIndex(routeIdx);
                if (!route || route->StopCount() < 2)
                    continue;

                for (std::size_t stopIdx = 1; stopIdx < route->StopCount(); ++stopIdx) {
                    auto stop1ID = route->GetStopID(stopIdx - 1);
                    auto stop2ID = route->GetStopID(stopIdx);
                    
                    auto stop1 = DBusSystem->StopByID(stop1ID);
                    auto stop2 = DBusSystem->StopByID(stop2ID);
                    
                    if (!stop1 || !stop2)
                        continue;

                    TNodeID node1ID = stop1->NodeID();
                    TNodeID node2ID = stop2->NodeID();

                    // Get nodes to calculate distance
                    auto node1Iter = DNodeIDToIndex.find(node1ID);
                    auto node2Iter = DNodeIDToIndex.find(node2ID);
                    
                    if (node1Iter == DNodeIDToIndex.end() || node2Iter == DNodeIDToIndex.end())
                        continue;
                    
                    std::shared_ptr<CStreetMap::SNode> node1 = DNodes[node1Iter->second];
                    std::shared_ptr<CStreetMap::SNode> node2 = DNodes[node2Iter->second];

                    double distance = CalculateDistance(node1, node2);
                    double busTime = distance / DConfig->BusSpeed();

                    // We use the router for calculating time, not distance
                    // Set up special bus edges with time as the weight
                    DPathRouter->AddEdge(node1ID, node2ID, busTime, true);
                }
            }
        }
    }

    double CalculateDistance(std::shared_ptr<CStreetMap::SNode> node1, std::shared_ptr<CStreetMap::SNode> node2) const {
        // Haversine formula for calculating distance between two lat/lon points
        const double EarthRadiusKm = 6371.0;
        const double DegreesToRadians = M_PI / 180.0;

        double lat1 = node1->Location().first * DegreesToRadians;
        double lon1 = node1->Location().second * DegreesToRadians;
        double lat2 = node2->Location().first * DegreesToRadians;
        double lon2 = node2->Location().second * DegreesToRadians;

        double dlon = lon2 - lon1;
        double dlat = lat2 - lat1;

        double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        return EarthRadiusKm * c;
    }

    std::size_t NodeCount() const noexcept {
        return DNodes.size();
    }

    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept {
        if (index < DNodes.size()) {
            return DNodes[index];
        }
        return nullptr;
    }

    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        path.clear();

        // Check if nodes exist
        auto srcIter = DNodeIDToIndex.find(src);
        auto destIter = DNodeIDToIndex.find(dest);
        
        if (srcIter == DNodeIDToIndex.end() || destIter == DNodeIDToIndex.end()) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        // Check if source and destination are the same
        if (src == dest) {
            path.push_back(src);
            return 0.0;
        }

        std::vector<CPathRouter::TVertexID> routerPath;
        double distance = DPathRouter->FindShortestPath(src, dest, routerPath);

        if (distance < 0) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        // Convert router path to node IDs
        for (const auto &vertex : routerPath) {
            path.push_back(vertex);
        }

        return distance;
    }

    // Find fastest path considering walking and buses
    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();

        // Check if nodes exist
        auto srcIter = DNodeIDToIndex.find(src);
        auto destIter = DNodeIDToIndex.find(dest);
        
        if (srcIter == DNodeIDToIndex.end() || destIter == DNodeIDToIndex.end()) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        // Check if source and destination are the same
        if (src == dest) {
            TTripStep step;
            step.first = ETransportationMode::Walk;
            step.second = src;
            path.push_back(step);
            return 0.0;
        }

        // Find the fastest walking path (distance / walking speed = time)
        std::vector<TNodeID> walkPath;
        double walkDistance = FindShortestPath(src, dest, walkPath);
        double walkTime = (walkDistance == std::numeric_limits<double>::max()) ? 
                          std::numeric_limits<double>::max() : 
                          walkDistance / DConfig->WalkSpeed();

        // Create a separate path router for fastest route calculation
        auto fastestRouter = std::make_shared<CDijkstraPathRouter>();

        // Add vertices
        for (const auto &node : DNodes) {
            fastestRouter->AddVertex(node->ID());
        }

        // Add walking edges with time as the weight
        for (std::size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (!way || way->NodeCount() < 2)
                continue;

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                auto node1Iter = DNodeIDToIndex.find(node1ID);
                auto node2Iter = DNodeIDToIndex.find(node2ID);
                
                if (node1Iter == DNodeIDToIndex.end() || node2Iter == DNodeIDToIndex.end())
                    continue;
                
                std::shared_ptr<CStreetMap::SNode> node1 = DNodes[node1Iter->second];
                std::shared_ptr<CStreetMap::SNode> node2 = DNodes[node2Iter->second];

                double distance = CalculateDistance(node1, node2);
                double time = distance / DConfig->WalkSpeed();
                
                fastestRouter->AddEdge(node1->ID(), node2->ID(), time, false);

                if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                    fastestRouter->AddEdge(node2->ID(), node1->ID(), time, false);
                }
            }
        }

        // Add bus routes as edges with time as the weight
        std::unordered_map<TNodeID, std::unordered_map<TNodeID, TBusID>> busRoutes;
        
        if (DBusSystem) {
            for (std::size_t routeIdx = 0; routeIdx < DBusSystem->RouteCount(); ++routeIdx) {
                auto route = DBusSystem->RouteByIndex(routeIdx);
                if (!route || route->StopCount() < 2)
                    continue;

                for (std::size_t stopIdx = 1; stopIdx < route->StopCount(); ++stopIdx) {
                    auto stop1ID = route->GetStopID(stopIdx - 1);
                    auto stop2ID = route->GetStopID(stopIdx);
                    
                    auto stop1 = DBusSystem->StopByID(stop1ID);
                    auto stop2 = DBusSystem->StopByID(stop2ID);
                    
                    if (!stop1 || !stop2)
                        continue;

                    TNodeID node1ID = stop1->NodeID();
                    TNodeID node2ID = stop2->NodeID();

                    auto node1Iter = DNodeIDToIndex.find(node1ID);
                    auto node2Iter = DNodeIDToIndex.find(node2ID);
                    
                    if (node1Iter == DNodeIDToIndex.end() || node2Iter == DNodeIDToIndex.end())
                        continue;
                    
                    std::shared_ptr<CStreetMap::SNode> node1 = DNodes[node1Iter->second];
                    std::shared_ptr<CStreetMap::SNode> node2 = DNodes[node2Iter->second];

                    double distance = CalculateDistance(node1, node2);
                    double busTime = distance / DConfig->BusSpeed();

                    fastestRouter->AddEdge(node1ID, node2ID, busTime, false);
                    
                    // Remember which bus route connects these nodes
                    busRoutes[node1ID][node2ID] = route->ID();
                }
            }
        }

        // Find fastest path considering both walking and buses
        std::vector<CPathRouter::TVertexID> fastestVertexPath;
        double fastestTime = fastestRouter->FindShortestPath(src, dest, fastestVertexPath);

        if (fastestTime < 0) {
            // If no path exists, return the walking path if it exists
            if (walkTime != std::numeric_limits<double>::max()) {
                for (const auto &nodeID : walkPath) {
                    TTripStep step;
                    step.first = ETransportationMode::Walk;
                    step.second = nodeID;
                    path.push_back(step);
                }
                return walkTime;
            }
            return std::numeric_limits<double>::max(); // No path exists
        }

        // Determine the transportation mode for each step of the path
        if (fastestVertexPath.size() > 1) {
            TTripStep firstStep;
            firstStep.first = ETransportationMode::Walk; // Start with walking
            firstStep.second = fastestVertexPath[0];
            path.push_back(firstStep);

            for (size_t i = 1; i < fastestVertexPath.size(); ++i) {
                TNodeID fromNode = fastestVertexPath[i-1];
                TNodeID toNode = fastestVertexPath[i];
                
                // Check if this step is on a bus route
                bool isBusStep = false;
                if (busRoutes.count(fromNode) && busRoutes[fromNode].count(toNode)) {
                    isBusStep = true;
                }
                
                TTripStep step;
                step.first = isBusStep ? ETransportationMode::Bus : ETransportationMode::Walk;
                step.second = toNode;
                path.push_back(step);
            }
        }

        return fastestTime;
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();

        if (path.empty()) {
            return false;
        }

        // Build meaningful description of the path
        TNodeID currentNodeID = path[0].second;
        auto nodeIter = DNodeIDToIndex.find(currentNodeID);
        
        if (nodeIter == DNodeIDToIndex.end()) {
            return false;
        }
        
        std::shared_ptr<CStreetMap::SNode> currentNode = DNodes[nodeIter->second];

        std::string startDesc = "Start at ";
        if (currentNode->HasAttribute("name")) {
            startDesc += currentNode->GetAttribute("name");
        } else {
            startDesc += "node " + std::to_string(currentNodeID);
        }
        desc.push_back(startDesc);

        for (std::size_t i = 1; i < path.size(); ++i) {
            const auto &step = path[i];
            std::string stepDesc;

            // Find node information
            auto nodeIter = DNodeIDToIndex.find(step.second);
            if (nodeIter == DNodeIDToIndex.end())
                continue;
                
            std::shared_ptr<CStreetMap::SNode> node = DNodes[nodeIter->second];

            // Create description based on transportation mode
            switch (step.first) {
            case ETransportationMode::Walk:
                stepDesc = "Walk to ";
                break;
            case ETransportationMode::Bike:
                stepDesc = "Bike to ";
                break;
            case ETransportationMode::Bus:
                stepDesc = "Take bus to ";
                break;
            }

            if (node->HasAttribute("name")) {
                stepDesc += node->GetAttribute("name");
            } else {
                stepDesc += "node " + std::to_string(step.second);
            }

            desc.push_back(stepDesc);
        }

        return true;
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
    // Destructor body can be empty when using std::unique_ptr
}

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->NodeCount();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedNodeByIndex(index);
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    return DImplementation->FindShortestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    return DImplementation->FindFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    return DImplementation->GetPathDescription(path, desc);
}