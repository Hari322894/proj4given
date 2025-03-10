#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <cmath>
#include <unordered_set>
#include <chrono>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> DConfig;
    std::shared_ptr<CStreetMap> DStreetMap;
    std::shared_ptr<CBusSystem> DBusSystem;
    std::shared_ptr<CDijkstraPathRouter> DWalkRouter;    // For walking
    std::shared_ptr<CDijkstraPathRouter> DBikeRouter;    // For biking
    std::shared_ptr<CDijkstraPathRouter> DBusRouter;     // For bus + walking
    std::vector<std::shared_ptr<CStreetMap::SNode>> DNodes;
    std::unordered_map<TNodeID, size_t> DNodeIDToIndex;  // For faster lookups

    SImplementation(std::shared_ptr<SConfiguration> config) {
        DConfig = config;
        DStreetMap = config->StreetMap();
        DBusSystem = config->BusSystem();

        // Initialize path routers for different transportation modes
        DWalkRouter = std::make_shared<CDijkstraPathRouter>();
        DBikeRouter = std::make_shared<CDijkstraPathRouter>();
        DBusRouter = std::make_shared<CDijkstraPathRouter>();

        // Store all nodes for quick lookup
        for (std::size_t i = 0; i < DStreetMap->NodeCount(); ++i) {
            auto node = DStreetMap->NodeByIndex(i);
            DNodes.push_back(node);
        }

        // Sort nodes by ID for consistent indexing
        std::sort(DNodes.begin(), DNodes.end(),
                  [](const auto &a, const auto &b) { return a->ID() < b->ID(); });

        // Create node ID to index mapping for faster lookups
        for (size_t i = 0; i < DNodes.size(); ++i) {
            DNodeIDToIndex[DNodes[i]->ID()] = i;
        }

        // Add vertices to all routers
        for (const auto &node : DNodes) {
            DWalkRouter->AddVertex(node->ID());
            DBikeRouter->AddVertex(node->ID());
            DBusRouter->AddVertex(node->ID());
        }

        // Add street edges for walking and biking
        for (std::size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way->NodeCount() < 2)
                continue;

            // Check way attributes for speed limit and biking restrictions
            double speedLimit = config->DefaultSpeedLimit();
            if (way->HasAttribute("maxspeed")) {
                try {
                    speedLimit = std::stod(way->GetAttribute("maxspeed"));
                } catch(...) {
                    // Use default speed if conversion fails
                }
            }
            
            bool allowBike = true;
            if (way->HasAttribute("bicycle") && way->GetAttribute("bicycle") == "no") {
                allowBike = false;
            }
            
            // Check if pedestrians are allowed (default is yes)
            bool allowWalk = true;
            if (way->HasAttribute("foot") && way->GetAttribute("foot") == "no") {
                allowWalk = false;
            }

            // Check if it's a one-way street
            bool isOneWay = way->HasAttribute("oneway") && way->GetAttribute("oneway") == "yes";

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                // Get the node objects from their IDs
                auto node1 = NodeByID(node1ID);
                auto node2 = NodeByID(node2ID);

                if (node1 && node2) {
                    double distance = CalculateDistance(node1, node2);
                    
                    // Walking edges - weighted by distance
                    if (allowWalk) {
                        DWalkRouter->AddEdge(node1ID, node2ID, distance, false);
                        if (!isOneWay) {
                            DWalkRouter->AddEdge(node2ID, node1ID, distance, false);
                        }
                    }
                    
                    // Biking edges - weighted by time (distance/speed)
                    if (allowBike) {
                        // Cap bike speed by the speed limit or bike speed
                        double bikeSpeed = std::min(config->BikeSpeed(), speedLimit);
                        double bikeTime = distance / bikeSpeed;
                        
                        DBikeRouter->AddEdge(node1ID, node2ID, bikeTime, false);
                        if (!isOneWay) {
                            DBikeRouter->AddEdge(node2ID, node1ID, bikeTime, false);
                        }
                    }
                    
                    // Bus router includes walking edges (weighted by time)
                    if (allowWalk) {
                        double walkTime = distance / config->WalkSpeed();
                        DBusRouter->AddEdge(node1ID, node2ID, walkTime, false);
                        if (!isOneWay) {
                            DBusRouter->AddEdge(node2ID, node1ID, walkTime, false);
                        }
                    }
                }
            }
        }
        
        // Add bus route edges to the bus router
        AddBusRouteEdges();
        
        // Precompute paths
        auto deadline = std::chrono::steady_clock::now() + 
                        std::chrono::seconds(config->PrecomputeTime());
        DWalkRouter->Precompute(deadline);
        DBikeRouter->Precompute(deadline);
        DBusRouter->Precompute(deadline);
    }
    
    void AddBusRouteEdges() {
        // Create map of node ID to bus stops for quick lookup
        std::unordered_map<TNodeID, std::vector<std::shared_ptr<CBusSystem::SStop>>> nodeStops;
        
        // First, index all stops by node ID
        for (size_t i = 0; i < DBusSystem->StopCount(); ++i) {
            auto stop = DBusSystem->StopByIndex(i);
            if (stop) {
                nodeStops[stop->NodeID()].push_back(stop);
            }
        }
        
        // For each route, add edges between consecutive stops
        for (size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
            auto route = DBusSystem->RouteByIndex(i);
            if (!route || route->StopCount() < 2) continue;
            
            for (size_t j = 1; j < route->StopCount(); ++j) {
                auto prevStopID = route->GetStopID(j-1);
                auto currStopID = route->GetStopID(j);
                
                auto prevStop = DBusSystem->StopByID(prevStopID);
                auto currStop = DBusSystem->StopByID(currStopID);
                
                if (!prevStop || !currStop) continue;
                
                auto prevNode = NodeByID(prevStop->NodeID());
                auto currNode = NodeByID(currStop->NodeID());
                
                if (!prevNode || !currNode) continue;
                
                // Calculate direct distance between stops
                double distance = CalculateDistance(prevNode, currNode);
                
                // Use default speed limit for bus routes
                double busSpeed = DConfig->DefaultSpeedLimit();
                
                // Time = distance/speed + stop time
                double travelTime = distance / busSpeed + DConfig->BusStopTime();
                
                // Add edge to bus router (one-way, as routes are directional)
                DBusRouter->AddEdge(prevStop->NodeID(), currStop->NodeID(), travelTime, false);
            }
        }
    }

    std::shared_ptr<CStreetMap::SNode> NodeByID(TNodeID id) const {
        auto it = DNodeIDToIndex.find(id);
        if (it != DNodeIDToIndex.end()) {
            return DNodes[it->second];
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

        // Check if nodes exist
        auto srcNode = NodeByID(src);
        auto destNode = NodeByID(dest);
        
        if (!srcNode || !destNode) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        // If source and destination are the same
        if (src == dest) {
            path.push_back(src);
            return 0.0;
        }

        std::vector<CPathRouter::TVertexID> routerPath;
        double distance = DWalkRouter->FindShortestPath(src, dest, routerPath);

        if (distance == CPathRouter::NoPathExists) {
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
        auto srcNode = NodeByID(src);
        auto destNode = NodeByID(dest);
        
        if (!srcNode || !destNode) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        // If source and destination are the same
        if (src == dest) {
            TTripStep step(ETransportationMode::Walk, src);
            path.push_back(step);
            return 0.0;
        }
        
        // Find fastest path across all transportation modes
        double walkTime = std::numeric_limits<double>::max();
        double bikeTime = std::numeric_limits<double>::max();
        double busTime = std::numeric_limits<double>::max();
        
        std::vector<CPathRouter::TVertexID> walkPath;
        std::vector<CPathRouter::TVertexID> bikePath;
        std::vector<CPathRouter::TVertexID> busPath;
        
        // Find walking path
        walkTime = DWalkRouter->FindShortestPath(src, dest, walkPath);
        if (walkTime != CPathRouter::NoPathExists) {
            walkTime = PathDistance(walkPath) / DConfig->WalkSpeed();
        } else {
            walkTime = std::numeric_limits<double>::max();
        }
        
        // Find biking path
        bikeTime = DBikeRouter->FindShortestPath(src, dest, bikePath);
        
        // Find bus path
        busTime = DBusRouter->FindShortestPath(src, dest, busPath);
        
        // Determine which mode is fastest
        if (walkTime <= bikeTime && walkTime <= busTime && walkTime != std::numeric_limits<double>::max()) {
            // Walking is fastest
            for (const auto &nodeID : walkPath) {
                path.push_back(TTripStep(ETransportationMode::Walk, nodeID));
            }
            return walkTime;
        } else if (bikeTime <= walkTime && bikeTime <= busTime && bikeTime != std::numeric_limits<double>::max()) {
            // Biking is fastest
            for (const auto &nodeID : bikePath) {
                path.push_back(TTripStep(ETransportationMode::Bike, nodeID));
            }
            return bikeTime;
        } else if (busTime != std::numeric_limits<double>::max()) {
            // Bus is fastest
            // We need to determine which segments are bus vs. walking
            DetermineBusSegments(busPath, path);
            return busTime;
        }
        
        return std::numeric_limits<double>::max(); // No path exists
    }
    
    double PathDistance(const std::vector<CPathRouter::TVertexID> &path) const {
        double totalDistance = 0.0;
        
        for (size_t i = 1; i < path.size(); ++i) {
            auto node1 = NodeByID(path[i-1]);
            auto node2 = NodeByID(path[i]);
            
            if (node1 && node2) {
                totalDistance += CalculateDistance(node1, node2);
            }
        }
        
        return totalDistance;
    }
    
    void DetermineBusSegments(const std::vector<CPathRouter::TVertexID> &busPath, 
                              std::vector<TTripStep> &tripPath) {
        if (busPath.empty()) return;
        
        // Create a set of node pairs that are connected by bus routes
        std::unordered_set<std::pair<TNodeID, TNodeID>, PairHash> busConnections;
        
        // Helper for hashing pairs
        struct PairHash {
            std::size_t operator()(const std::pair<TNodeID, TNodeID>& p) const {
                return std::hash<TNodeID>()(p.first) ^ (std::hash<TNodeID>()(p.second) << 1);
            }
        };
        
        // For each route, add edges between consecutive stops
        for (size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
            auto route = DBusSystem->RouteByIndex(i);
            if (!route || route->StopCount() < 2) continue;
            
            for (size_t j = 1; j < route->StopCount(); ++j) {
                auto prevStopID = route->GetStopID(j-1);
                auto currStopID = route->GetStopID(j);
                
                auto prevStop = DBusSystem->StopByID(prevStopID);
                auto currStop = DBusSystem->StopByID(currStopID);
                
                if (!prevStop || !currStop) continue;
                
                // Add to bus connections set
                busConnections.insert(std::make_pair(prevStop->NodeID(), currStop->NodeID()));
            }
        }
        
        // First node is always part of the path
        ETransportationMode currentMode = ETransportationMode::Walk; // Start with walking
        tripPath.push_back(TTripStep(currentMode, busPath[0]));
        
        // Go through the path and determine transportation mode for each segment
        for (size_t i = 1; i < busPath.size(); ++i) {
            TNodeID prevNodeID = busPath[i-1];
            TNodeID currNodeID = busPath[i];
            
            // Check if this segment is a bus connection
            if (busConnections.find(std::make_pair(prevNodeID, currNodeID)) != busConnections.end()) {
                // This is a bus segment
                if (currentMode != ETransportationMode::Bus) {
                    currentMode = ETransportationMode::Bus;
                }
            } else {
                // This is a walking segment
                if (currentMode != ETransportationMode::Walk) {
                    currentMode = ETransportationMode::Walk;
                }
            }
            
            tripPath.push_back(TTripStep(currentMode, currNodeID));
        }
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();

        if (path.empty()) {
            return false;
        }

        // Build meaningful description of the path
        ETransportationMode currentMode = path[0].first;
        TNodeID currentNodeID = path[0].second;
        std::shared_ptr<CStreetMap::SNode> currentNode = NodeByID(currentNodeID);

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

        // Track the current bus route if we're on a bus
        std::string currentBusRoute = "";
        
        for (std::size_t i = 1; i < path.size(); ++i) {
            const auto &step = path[i];
            ETransportationMode mode = step.first;
            TNodeID nodeID = step.second;
            
            // Find node information
            std::shared_ptr<CStreetMap::SNode> node = NodeByID(nodeID);
            if (!node) continue;
            
            // Handle mode changes
            if (i > 1 && mode != path[i-1].first) {
                std::string modeChangeDesc;
                
                switch (mode) {
                case ETransportationMode::Walk:
                    modeChangeDesc = "Get off the bus and walk";
                    break;
                case ETransportationMode::Bike:
                    modeChangeDesc = "Start biking";
                    break;
                case ETransportationMode::Bus:
                    {
                        // Determine which bus route we're taking
                        TNodeID prevNodeID = path[i-1].second;
                        std::string routeName = FindBusRouteBetween(prevNodeID, nodeID);
                        currentBusRoute = routeName;
                        
                        if (!routeName.empty()) {
                            modeChangeDesc = "Take bus " + routeName;
                        } else {
                            modeChangeDesc = "Take the bus";
                        }
                    }
                    break;
                }
                
                desc.push_back(modeChangeDesc);
            }
            
            // Add destination description if this is the last node or mode is changing
            if (i == path.size() - 1 || mode != path[i+1].first) {
                std::string destDesc = "Arrive at ";
                if (node->HasAttribute("name")) {
                    destDesc += node->GetAttribute("name");
                } else {
                    destDesc += "node " + std::to_string(nodeID);
                }
                
                desc.push_back(destDesc);
            }
        }

        return true;
    }
    
    std::string FindBusRouteBetween(TNodeID src, TNodeID dest) const {
        // Find a bus route that connects these two nodes directly
        for (size_t i = 0; i < DBusSystem->RouteCount(); ++i) {
            auto route = DBusSystem->RouteByIndex(i);
            if (!route || route->StopCount() < 2) continue;
            
            // Find stop IDs for these nodes
            std::vector<TNodeID> routeNodes;
            for (size_t j = 0; j < route->StopCount(); ++j) {
                auto stopID = route->GetStopID(j);
                auto stop = DBusSystem->StopByID(stopID);
                if (stop) {
                    routeNodes.push_back(stop->NodeID());
                }
            }
            
            // Check if source and destination are consecutive on this route
            for (size_t j = 1; j < routeNodes.size(); ++j) {
                if (routeNodes[j-1] == src && routeNodes[j] == dest) {
                    return route->Name();
                }
            }
        }
        
        return ""; // No direct bus route found
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