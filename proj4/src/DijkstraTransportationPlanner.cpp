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
                    // Note: Not using the time variable to avoid the warning
                    
                    DPathRouter->AddEdge(node1->ID(), node2->ID(), distance, false);

                    // Add edge in reverse if way is bidirectional
                    if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                        DPathRouter->AddEdge(node2->ID(), node1->ID(), distance, false);
                    }
                }
            }
        }

        // Add bus routes connections if bus system is available
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
                            // Using BikeSpeed instead of BusSpeed since BusSpeed doesn't exist
                            // Removed unused busTime variable to avoid warning
                            
                            // Mark this edge as a bus connection for later use
                            DPathRouter->AddEdge(node1->ID(), node2->ID(), distance, true);
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

        // First try to find a walking path
        std::vector<TNodeID> walkPath;
        double walkDistance = FindShortestPath(src, dest, walkPath);
        
        if (walkDistance == std::numeric_limits<double>::max()) {
            return std::numeric_limits<double>::max(); // No path exists
        }
        
        double walkTime = walkDistance / DConfig->WalkSpeed();
        
        // Create a path with walking steps
        std::vector<TTripStep> walkingPath;
        for (const auto &nodeID : walkPath) {
            TTripStep step;
            step.first = ETransportationMode::Walk;
            step.second = nodeID;
            walkingPath.push_back(step);
        }

        // If there's a bus system, try to find a faster bus path
        double busTime = std::numeric_limits<double>::max();
        std::vector<TTripStep> busPath;
        
        if (DBusSystem) {
            // Find the best combination of walking and bus routes
            busPath = FindBestBusPath(src, dest, busTime);
        }
        
        // Choose the faster option
        if (busTime < walkTime) {
            path = busPath;
            return busTime;
        } else {
            path = walkingPath;
            return walkTime;
        }
    }

    std::vector<TTripStep> FindBestBusPath(TNodeID src, TNodeID dest, double &totalTime) {
        std::vector<TTripStep> bestPath;
        totalTime = std::numeric_limits<double>::max();
        
        // Find shortest path first as a base
        std::vector<TNodeID> shortestPath;
        FindShortestPath(src, dest, shortestPath);
        
        if (shortestPath.empty() || shortestPath.size() < 2) {
            return bestPath; // No valid path found
        }
        
        // Try to find bus routes that connect nodes in the path
        bool foundBusRoute = false;
        
        for (size_t i = 0; i < shortestPath.size() - 1; i++) {
            for (size_t j = i + 1; j < shortestPath.size(); j++) {
                TNodeID startNodeID = shortestPath[i];
                TNodeID endNodeID = shortestPath[j];
                
                // Look for a bus route between these nodes
                bool busRouteFound = false;
                
                for (std::size_t routeIdx = 0; routeIdx < DBusSystem->RouteCount(); ++routeIdx) {
                    auto route = DBusSystem->RouteByIndex(routeIdx);
                    
                    // Find stops corresponding to the nodes
                    std::shared_ptr<CBusSystem::SStop> startStop = nullptr;
                    std::shared_ptr<CBusSystem::SStop> endStop = nullptr;
                    int startStopIdx = -1;
                    int endStopIdx = -1;
                    
                    for (std::size_t s = 0; s < route->StopCount(); ++s) {
                        auto stopID = route->GetStopID(s);
                        auto stop = DBusSystem->StopByID(stopID);
                        
                        if (stop && stop->NodeID() == startNodeID) {
                            startStop = stop;
                            startStopIdx = s;
                        }
                        if (stop && stop->NodeID() == endNodeID) {
                            endStop = stop;
                            endStopIdx = s;
                        }
                    }
                    
                    // Check if we found valid stops and they're in the correct order
                    if (startStop && endStop && startStopIdx < endStopIdx) {
                        busRouteFound = true;
                        foundBusRoute = true;
                        
                        // Calculate a path using this bus route
                        std::vector<TTripStep> candidatePath;
                        double candidateTime = 0;
                        
                        // Walk to the start bus stop
                        if (src != startNodeID) {
                            std::vector<TNodeID> walkToStartPath;
                            double walkDist = FindShortestPath(src, startNodeID, walkToStartPath);
                            
                            if (walkDist < std::numeric_limits<double>::max()) {
                                candidateTime += walkDist / DConfig->WalkSpeed();
                                
                                for (const auto &nodeID : walkToStartPath) {
                                    TTripStep step;
                                    step.first = ETransportationMode::Walk;
                                    step.second = nodeID;
                                    candidatePath.push_back(step);
                                }
                            }
                        } else {
                            // Just add the starting node
                            TTripStep step;
                            step.first = ETransportationMode::Walk;
                            step.second = src;
                            candidatePath.push_back(step);
                        }
                        
                        // Take the bus from start stop to end stop
                        double busDist = 0;
                        
                        for (int s = startStopIdx; s < endStopIdx; ++s) {
                            auto currentStopID = route->GetStopID(s);
                            auto nextStopID = route->GetStopID(s + 1);
                            
                            auto currentStop = DBusSystem->StopByID(currentStopID);
                            auto nextStop = DBusSystem->StopByID(nextStopID);
                            
                            if (currentStop && nextStop) {
                                auto currentNode = FindNodeByID(currentStop->NodeID());
                                auto nextNode = FindNodeByID(nextStop->NodeID());
                                
                                if (currentNode && nextNode) {
                                    busDist += CalculateDistance(currentNode, nextNode);
                                }
                            }
                        }
                        
                        // Using BikeSpeed as approximation for bus speed
                        candidateTime += busDist / DConfig->BikeSpeed();
                        
                        // Add bus stops to the path
                        for (int s = startStopIdx; s <= endStopIdx; ++s) {
                            auto stopID = route->GetStopID(s);
                            auto stop = DBusSystem->StopByID(stopID);
                            
                            if (stop) {
                                // Replace the last walk step if needed
                                if (s == startStopIdx && !candidatePath.empty() && 
                                    candidatePath.back().second == stop->NodeID()) {
                                    candidatePath.pop_back();
                                }
                                
                                TTripStep step;
                                step.first = ETransportationMode::Bus;
                                step.second = stop->NodeID();
                                candidatePath.push_back(step);
                            }
                        }
                        
                        // Walk from end stop to destination
                        if (dest != endNodeID) {
                            std::vector<TNodeID> walkToDestPath;
                            double walkDist = FindShortestPath(endNodeID, dest, walkToDestPath);
                            
                            if (walkDist < std::numeric_limits<double>::max()) {
                                candidateTime += walkDist / DConfig->WalkSpeed();
                                
                                // Skip first node as it's already included
                                for (size_t k = 1; k < walkToDestPath.size(); ++k) {
                                    TTripStep step;
                                    step.first = ETransportationMode::Walk;
                                    step.second = walkToDestPath[k];
                                    candidatePath.push_back(step);
                                }
                            }
                        }
                        
                        // Update best path if this one is faster
                        if (candidateTime < totalTime) {
                            totalTime = candidateTime;
                            bestPath = candidatePath;
                        }
                    }
                }
                
                // If we found a bus route, skip the rest of the inner loop
                if (busRouteFound) {
                    break;
                }
            }
        }
        
        // If no bus route was found, return an empty path
        if (!foundBusRoute) {
            totalTime = std::numeric_limits<double>::max();
            bestPath.clear();
        }
        
        return bestPath;
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();

        if (path.empty()) {
            return false;
        }

        // Build meaningful description of the path
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