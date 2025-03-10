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
    std::unordered_map<TNodeID, std::size_t> DNodeIDToIndex;

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
        }

        // Sort nodes by ID for consistent indexing
        std::sort(DNodes.begin(), DNodes.end(),
                  [](const auto &a, const auto &b) { return a->ID() < b->ID(); });

        // Build node ID to index mapping for quick lookup
        for (std::size_t i = 0; i < DNodes.size(); ++i) {
            DNodeIDToIndex[DNodes[i]->ID()] = i;
        }

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
                    // If conversion fails, use default
                }
            }

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                // Get the node objects from their IDs
                auto node1 = FindNodeByID(node1ID);
                auto node2 = FindNodeByID(node2ID);

                if (node1 && node2) {
                    double distance = CalculateDistance(node1, node2);
                    double time = distance / speed;
                    
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

                for (std::size_t j = 1; j < route->StopCount(); ++j) {
                    auto stop1 = route->GetStopID(j - 1);
                    auto stop2 = route->GetStopID(j);

                    auto stopNode1 = DBusSystem->StopByID(stop1);
                    auto stopNode2 = DBusSystem->StopByID(stop2);
                    
                    if (stopNode1 && stopNode2) {
                        auto node1 = FindNodeByID(stopNode1->NodeID());
                        auto node2 = FindNodeByID(stopNode2->NodeID());
                        
                        if (node1 && node2) {
                            double distance = CalculateDistance(node1, node2);
                            double busTime = distance / config->BusSpeed();
                            
                            // Add bus edge - we'll use this in FindFastestPath
                            CPathRouter::TEdgeID busEdge = DPathRouter->AddEdge(node1->ID(), node2->ID(), busTime, true);
                        }
                    }
                }
            }
        }
    }

    std::shared_ptr<CStreetMap::SNode> FindNodeByID(TNodeID id) const {
        auto it = DNodeIDToIndex.find(id);
        if (it != DNodeIDToIndex.end()) {
            return DNodes[it->second];
        }
        
        // Fallback to linear search if not found in map
        for (const auto &node : DNodes) {
            if (node->ID() == id) {
                return node;
            }
        }
        return nullptr;
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

        // Check if source and destination are the same
        if (src == dest) {
            path.push_back(src);
            return 0.0;
        }

        // Find the shortest path using the path router
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

        // Create a temporary path router for the fastest path calculation
        auto tempRouter = std::make_shared<CDijkstraPathRouter>();
        
        // Add all vertices
        for (const auto &node : DNodes) {
            tempRouter->AddVertex(node->ID());
        }

        // Add walking edges
        for (std::size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way->NodeCount() < 2)
                continue;

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                auto node1 = FindNodeByID(node1ID);
                auto node2 = FindNodeByID(node2ID);

                if (node1 && node2) {
                    double distance = CalculateDistance(node1, node2);
                    double walkTime = distance / DConfig->WalkSpeed();
                    
                    tempRouter->AddEdge(node1->ID(), node2->ID(), walkTime, false);

                    // Add edge in reverse if way is bidirectional
                    if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                        tempRouter->AddEdge(node2->ID(), node1->ID(), walkTime, false);
                    }
                }
            }
        }

        // Add bus edges
        std::unordered_map<CPathRouter::TEdgeID, ETransportationMode> edgeToMode;
        
        if (DBusSystem) {
            for (std::size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
                auto route = DBusSystem->RouteByIndex(i);
                if (route->StopCount() < 2)
                    continue;

                for (std::size_t j = 1; j < route->StopCount(); ++j) {
                    auto stop1 = route->GetStopID(j - 1);
                    auto stop2 = route->GetStopID(j);

                    auto stopNode1 = DBusSystem->StopByID(stop1);
                    auto stopNode2 = DBusSystem->StopByID(stop2);
                    
                    if (stopNode1 && stopNode2) {
                        auto node1 = FindNodeByID(stopNode1->NodeID());
                        auto node2 = FindNodeByID(stopNode2->NodeID());
                        
                        if (node1 && node2) {
                            double distance = CalculateDistance(node1, node2);
                            double busTime = distance / DConfig->BusSpeed();
                            
                            // Add bus edge with time based on bus speed
                            CPathRouter::TEdgeID busEdge = tempRouter->AddEdge(node1->ID(), node2->ID(), busTime, true);
                            edgeToMode[busEdge] = ETransportationMode::Bus;
                        }
                    }
                }
            }
        }

        // Find the fastest path
        std::vector<CPathRouter::TVertexID> fastestVertexPath;
        std::vector<CPathRouter::TEdgeID> fastestEdgePath;
        double time = tempRouter->FindShortestPath(src, dest, fastestVertexPath, fastestEdgePath);

        if (time < 0) {
            return std::numeric_limits<double>::max(); // No path exists
        }

        // Convert the path to trip steps
        if (!fastestVertexPath.empty()) {
            // Add the starting node
            TTripStep startStep;
            startStep.first = ETransportationMode::Walk; // Default to walking for the first node
            startStep.second = fastestVertexPath[0];
            path.push_back(startStep);

            // Add the rest of the path with appropriate transportation modes
            for (std::size_t i = 0; i < fastestEdgePath.size(); ++i) {
                TTripStep step;
                
                // Determine the transportation mode for this edge
                auto it = edgeToMode.find(fastestEdgePath[i]);
                if (it != edgeToMode.end()) {
                    step.first = it->second;
                } else {
                    step.first = ETransportationMode::Walk;
                }
                
                step.second = fastestVertexPath[i + 1];
                path.push_back(step);
            }
        }

        return time;
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();

        if (path.empty()) {
            return false;
        }

        // Build meaningful description of the path
        ETransportationMode currentMode = path[0].first;
        TNodeID currentNodeID = path[0].second;
        std::shared_ptr<CStreetMap::SNode> currentNode = FindNodeByID(currentNodeID);

        if (!currentNode) {
            return false;
        }

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
            std::shared_ptr<CStreetMap::SNode> node = FindNodeByID(step.second);
            if (!node)
                continue;

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