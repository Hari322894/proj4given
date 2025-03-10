#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <set>

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
    std::unordered_map<TStopID, TNodeID> DStopToNodeMap;
    std::unordered_map<TNodeID, std::set<TStopID>> DNodeToStopsMap;
    std::unordered_map<TNodeID, std::unordered_map<TNodeID, bool>> DBusConnections;

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

        // Create mapping between bus stops and nodes
        if (DBusSystem) {
            for (std::size_t i = 0; i < DBusSystem->StopCount(); ++i) {
                auto stop = DBusSystem->StopByIndex(i);
                DStopToNodeMap[stop->ID()] = stop->NodeID();
                DNodeToStopsMap[stop->NodeID()].insert(stop->ID());
            }

            // Build bus connections map
            for (std::size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
                auto route = DBusSystem->RouteByIndex(i);
                for (std::size_t j = 1; j < route->StopCount(); ++j) {
                    auto stop1ID = route->GetStopID(j - 1);
                    auto stop2ID = route->GetStopID(j);
                    
                    auto node1ID = DStopToNodeMap[stop1ID];
                    auto node2ID = DStopToNodeMap[stop2ID];
                    
                    DBusConnections[node1ID][node2ID] = true;
                }
            }
        }

        // Add vertices to the path router
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

                auto node1Iter = DNodeMap.find(node1ID);
                auto node2Iter = DNodeMap.find(node2ID);

                if (node1Iter != DNodeMap.end() && node2Iter != DNodeMap.end()) {
                    auto node1 = node1Iter->second;
                    auto node2 = node2Iter->second;
                    
                    double distance = CalculateDistance(node1, node2);
                    
                    DPathRouter->AddEdge(node1->ID(), node2->ID(), distance, false);

                    // Add edge in reverse if way is bidirectional
                    if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
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

        // Handle special case: source and destination are the same
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

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();

        // Check if nodes exist
        auto srcIter = DNodeMap.find(src);
        auto destIter = DNodeMap.find(dest);
        
        if (srcIter == DNodeMap.end() || destIter == DNodeMap.end()) {
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

                double busSpeed = DConfig->BikeSpeed(); // Using bike speed for bus
                
                for (std::size_t j = 1; j < route->StopCount(); ++j) {
                    auto stop1ID = route->GetStopID(j - 1);
                    auto stop2ID = route->GetStopID(j);
                    
                    auto node1IDIter = DStopToNodeMap.find(stop1ID);
                    auto node2IDIter = DStopToNodeMap.find(stop2ID);
                    
                    if (node1IDIter != DStopToNodeMap.end() && node2IDIter != DStopToNodeMap.end()) {
                        TNodeID node1ID = node1IDIter->second;
                        TNodeID node2ID = node2IDIter->second;
                        
                        auto node1Iter = DNodeMap.find(node1ID);
                        auto node2Iter = DNodeMap.find(node2ID);
                        
                        if (node1Iter != DNodeMap.end() && node2Iter != DNodeMap.end()) {
                            auto node1 = node1Iter->second;
                            auto node2 = node2Iter->second;
                            
                            double distance = CalculateDistance(node1, node2);
                            double busTime = distance / busSpeed;
                            
                            // Add wait time for the first stop of each route
                            if (j == 1) {
                                busTime += 2.0; // 2 minutes wait time (adjust as needed)
                            }
                            
                            // Bus edges are directed along the route
                            multiModalRouter->AddEdge(node1ID, node2ID, busTime, true);
                        }
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
            ETransportationMode currentMode = ETransportationMode::Walk;
            
            for (size_t i = 0; i < routerPath.size(); ++i) {
                TTripStep step;
                TNodeID currentNodeID = routerPath[i];
                
                if (i > 0) {
                    TNodeID prevNodeID = routerPath[i-1];
                    
                    // Check if there's a bus connection between these nodes
                    auto prevIter = DBusConnections.find(prevNodeID);
                    if (prevIter != DBusConnections.end() && 
                        prevIter->second.find(currentNodeID) != prevIter->second.end()) {
                        currentMode = ETransportationMode::Bus;
                    } else {
                        currentMode = ETransportationMode::Walk;
                    }
                }
                
                step.first = currentMode;
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

        ETransportationMode prevMode = path[0].first;
        
        for (std::size_t i = 1; i < path.size(); ++i) {
            const auto &step = path[i];
            ETransportationMode mode = step.first;
            TNodeID nodeID = step.second;
            
            auto nodeIter = DNodeMap.find(nodeID);
            if (nodeIter == DNodeMap.end()) {
                continue;
            }
            
            auto node = nodeIter->second;

            // Mode change detection
            if (mode != prevMode) {
                std::string transitionDesc;
                switch (mode) {
                case ETransportationMode::Walk:
                    transitionDesc = "Switch to walking";
                    break;
                case ETransportationMode::Bike:
                    transitionDesc = "Switch to biking";
                    break;
                case ETransportationMode::Bus:
                    transitionDesc = "Switch to bus";
                    break;
                }
                desc.push_back(transitionDesc);
                prevMode = mode;
            }

            // Create description based on transportation mode
            std::string stepDesc;
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