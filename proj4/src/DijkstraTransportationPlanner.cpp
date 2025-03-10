#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <cmath> // Added for math functions

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
    std::unordered_map<TNodeID, std::shared_ptr<CStreetMap::SNode>> DNodeMap;

    SImplementation(std::shared_ptr<SConfiguration> config) {
        DConfig = config;
        DStreetMap = config->StreetMap();
        DBusSystem = config->BusSystem();

        // Initialize path router
        DPathRouter = std::make_shared<CDijkstraPathRouter>();

        // Store all nodes for quick lookup
        for (std::size_t i = 0; i < DStreetMap->NodeCount(); ++i) {
            auto node = DStreetMap->NodeByIndex(i);
            DNodes.push_back(node);
            DNodeMap[node->ID()] = node;
        }

        // Sort nodes by ID for consistent indexing
        std::sort(DNodes.begin(), DNodes.end(),
                  [](const auto &a, const auto &b) { return a->ID() < b->ID(); });

        // Add vertices and edges to the path router
        for (const auto &node : DNodes) {
            DPathRouter->AddVertex(node->ID());
        }

        // Add street edges
        for (std::size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way->NodeCount() < 2)
                continue;

            double speed = config->DefaultSpeedLimit();
            if (way->HasAttribute("maxspeed")) {
                try {
                    speed = std::stod(way->GetAttribute("maxspeed"));
                } catch (...) {
                    // Use default if conversion fails
                }
            }

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                // Get the node objects from their IDs using our map
                auto node1Iter = DNodeMap.find(node1ID);
                auto node2Iter = DNodeMap.find(node2ID);

                if (node1Iter != DNodeMap.end() && node2Iter != DNodeMap.end()) {
                    auto node1 = node1Iter->second;
                    auto node2 = node2Iter->second;
                    
                    double distance = CalculateDistance(node1, node2);
                    // Removed unused timeWeight variable
                    
                    DPathRouter->AddEdge(node1->ID(), node2->ID(), distance, false);

                    // Add edge in reverse if way is bidirectional
                    if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                        DPathRouter->AddEdge(node2->ID(), node1->ID(), distance, false);
                    }
                }
            }
        }

        // Add bus route edges
        if (DBusSystem) {
            for (std::size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
                auto route = DBusSystem->RouteByIndex(i);
                if (route->StopCount() < 2)
                    continue;

                // Using BikeSpeed instead of BusSpeed (Configuration doesn't have BusSpeed)
                double busSpeed = DConfig->BikeSpeed();
                
                for (std::size_t j = 1; j < route->StopCount(); ++j) {
                    // Fixed: Using GetStopID instead of StopByIndex
                    auto stop1ID = route->GetStopID(j - 1);
                    auto stop2ID = route->GetStopID(j);
                    
                    // Get the nodes associated with these stops
                    TNodeID node1ID = 0;
                    TNodeID node2ID = 0;
                    
                    // Find the nodes for these stops
                    for (std::size_t s = 0; s < DBusSystem->StopCount(); ++s) {
                        auto stop = DBusSystem->StopByIndex(s);
                        if (stop->ID() == stop1ID) {
                            node1ID = stop->NodeID();
                        }
                        if (stop->ID() == stop2ID) {
                            node2ID = stop->NodeID();
                        }
                    }
                    
                    auto node1Iter = DNodeMap.find(node1ID);
                    auto node2Iter = DNodeMap.find(node2ID);
                    
                    if (node1Iter != DNodeMap.end() && node2Iter != DNodeMap.end()) {
                        auto node1 = node1Iter->second;
                        auto node2 = node2Iter->second;
                        
                        double distance = CalculateDistance(node1, node2);
                        // Removed unused busTime variable
                        
                        // We'll use separate bus edges in the actual fastest path calculation
                        // This is just to establish connectivity in the basic path router
                        DPathRouter->AddEdge(node1->ID(), node2->ID(), distance, false);
                        DPathRouter->AddEdge(node2->ID(), node1->ID(), distance, false);
                    }
                }
            }
        }
    }

    double CalculateDistance(std::shared_ptr<CStreetMap::SNode> node1,
                             std::shared_ptr<CStreetMap::SNode> node2) const {
        // Haversine formula for calculating distance between two lat/lon points
        const double EarthRadiusKm = 6371.0;
        const double DegreesToRadians = M_PI / 180.0;

        double lat1 = node1->Location().first * DegreesToRadians;
        double lon1 = node1->Location().second * DegreesToRadians;
        double lat2 = node2->Location().first * DegreesToRadians;
        double lon2 = node2->Location().second * DegreesToRadians;

        double dlon = lon2 - lon1;
        double dlat = lat2 - lat1;

        double a = sin(dlat / 2) * sin(dlat / 2) +
                   cos(lat1) * cos(lat2) *
                       sin(dlon / 2) * sin(dlon / 2);
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
        auto srcIter = DNodeMap.find(src);
        auto destIter = DNodeMap.find(dest);
        
        if (srcIter == DNodeMap.end() || destIter == DNodeMap.end()) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
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

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();

        // Check if source and destination are the same
        if (src == dest) {
            TTripStep step;
            step.first = ETransportationMode::Walk;
            step.second = src;
            path.push_back(step);
            return 0.0;
        }

        // Check if nodes exist
        auto srcIter = DNodeMap.find(src);
        auto destIter = DNodeMap.find(dest);
        
        if (srcIter == DNodeMap.end() || destIter == DNodeMap.end()) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        // Create a multi-modal graph for the fastest path calculation
        std::shared_ptr<CDijkstraPathRouter> multiModalRouter = std::make_shared<CDijkstraPathRouter>();
        
        // Add all nodes (vertices)
        for (const auto &node : DNodes) {
            multiModalRouter->AddVertex(node->ID());
        }
        
        // Add walking edges
        for (std::size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way->NodeCount() < 2)
                continue;

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                auto node1Iter = DNodeMap.find(node1ID);
                auto node2Iter = DNodeMap.find(node2ID);

                if (node1Iter != DNodeMap.end() && node2Iter != DNodeMap.end()) {
                    auto node1 = node1Iter->second;
                    auto node2 = node2Iter->second;
                    
                    double distance = CalculateDistance(node1, node2);
                    double walkTime = distance / DConfig->WalkSpeed();
                    
                    multiModalRouter->AddEdge(node1->ID(), node2->ID(), walkTime, false);

                    // Add edge in reverse if way is bidirectional
                    if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                        multiModalRouter->AddEdge(node2->ID(), node1->ID(), walkTime, false);
                    }
                }
            }
        }

        // Add bus route edges with time weights
        if (DBusSystem) {
            for (std::size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
                auto route = DBusSystem->RouteByIndex(i);
                if (route->StopCount() < 2)
                    continue;

                // Using BikeSpeed instead of BusSpeed
                double busSpeed = DConfig->BikeSpeed();
                
                for (std::size_t j = 1; j < route->StopCount(); ++j) {
                    // Fixed: Using GetStopID instead of StopByIndex
                    auto stop1ID = route->GetStopID(j - 1);
                    auto stop2ID = route->GetStopID(j);
                    
                    // Get the nodes associated with these stops
                    TNodeID node1ID = 0;
                    TNodeID node2ID = 0;
                    
                    // Find the nodes for these stops
                    for (std::size_t s = 0; s < DBusSystem->StopCount(); ++s) {
                        auto stop = DBusSystem->StopByIndex(s);
                        if (stop->ID() == stop1ID) {
                            node1ID = stop->NodeID();
                        }
                        if (stop->ID() == stop2ID) {
                            node2ID = stop->NodeID();
                        }
                    }
                    
                    auto node1Iter = DNodeMap.find(node1ID);
                    auto node2Iter = DNodeMap.find(node2ID);
                    
                    if (node1Iter != DNodeMap.end() && node2Iter != DNodeMap.end()) {
                        auto node1 = node1Iter->second;
                        auto node2 = node2Iter->second;
                        
                        double distance = CalculateDistance(node1, node2);
                        double busTime = distance / busSpeed;
                        
                        // Add wait time to the first stop - using a reasonable default since BusWaitTime isn't available
                        if (j == 1) {
                            busTime += 5.0; // 5 minutes wait time as a default
                        }
                        
                        // Bus edges are directed along the route
                        multiModalRouter->AddEdge(node1ID, node2ID, busTime, true);
                    }
                }
            }
        }

        // Find fastest path using the multi-modal router
        std::vector<CPathRouter::TVertexID> routerPath;
        double totalTime = multiModalRouter->FindShortestPath(src, dest, routerPath);

        if (totalTime < 0) {
            return std::numeric_limits<double>::max(); // No path exists
        }

        // Convert router path to trip steps with appropriate transportation modes
        if (!routerPath.empty()) {
            // Create a mapping of bus route connections for quick lookups
            std::unordered_map<TNodeID, std::unordered_map<TNodeID, bool>> busConnections;
            
            if (DBusSystem) {
                for (std::size_t r = 0; r < DBusSystem->RouteCount(); ++r) {
                    auto route = DBusSystem->RouteByIndex(r);
                    
                    std::vector<TNodeID> routeNodes;
                    // Get all node IDs in this route
                    for (std::size_t s = 0; s < route->StopCount(); ++s) {
                        auto stopID = route->GetStopID(s);
                        TNodeID nodeID = 0;
                        
                        // Find the node for this stop
                        for (std::size_t i = 0; i < DBusSystem->StopCount(); ++i) {
                            auto stop = DBusSystem->StopByIndex(i);
                            if (stop->ID() == stopID) {
                                nodeID = stop->NodeID();
                                break;
                            }
                        }
                        
                        if (nodeID != 0) {
                            routeNodes.push_back(nodeID);
                        }
                    }
                    
                    // Create connections between consecutive stops
                    for (std::size_t i = 1; i < routeNodes.size(); ++i) {
                        busConnections[routeNodes[i-1]][routeNodes[i]] = true;
                    }
                }
            }
            
            // Determine transportation mode for each step
            for (size_t i = 0; i < routerPath.size(); ++i) {
                TTripStep step;
                TNodeID currentNodeID = routerPath[i];
                
                // Default to walking mode
                step.first = ETransportationMode::Walk;
                
                // Check if this step is part of a bus route
                if (i > 0) {
                    TNodeID prevNodeID = routerPath[i-1];
                    
                    // Check if there's a bus connection between these nodes
                    auto prevIter = busConnections.find(prevNodeID);
                    if (prevIter != busConnections.end() && 
                        prevIter->second.find(currentNodeID) != prevIter->second.end()) {
                        step.first = ETransportationMode::Bus;
                    }
                }
                
                step.second = currentNodeID;
                path.push_back(step);
            }
        }

        return totalTime;
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();

        if (path.empty()) {
            return false;
        }

        // Build meaningful description of the path
        // Removed unused currentMode variable
        TNodeID currentNodeID = path[0].second;
        
        auto nodeIter = DNodeMap.find(currentNodeID);
        if (nodeIter == DNodeMap.end()) {
            return false;
        }
        
        auto currentNode = nodeIter->second;

        std::string startDesc = "Start at ";
        if (currentNode->HasAttribute("name")) {
            startDesc += currentNode->GetAttribute("name");
        } else {
            startDesc += "node " + std::to_string(currentNodeID);
        }
        desc.push_back(startDesc);

        for (std::size_t i = 1; i < path.size(); ++i) {
            const auto &step = path[i];
            ETransportationMode mode = step.first;
            TNodeID nodeID = step.second;
            
            auto nodeIter = DNodeMap.find(nodeID);
            if (nodeIter == DNodeMap.end()) {
                continue;
            }
            
            auto node = nodeIter->second;

            std::string stepDesc;
            
            // Mode change detection
            if (i > 1 && mode != path[i-1].first) {
                // Mode changed, add a transition description
                switch (mode) {
                case ETransportationMode::Walk:
                    stepDesc = "Switch to walking";
                    desc.push_back(stepDesc);
                    break;
                case ETransportationMode::Bike:
                    stepDesc = "Switch to biking";
                    desc.push_back(stepDesc);
                    break;
                case ETransportationMode::Bus:
                    stepDesc = "Switch to bus";
                    desc.push_back(stepDesc);
                    break;
                }
            }

            // Create description based on transportation mode
            switch (mode) {
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
                stepDesc += "node " + std::to_string(nodeID);
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