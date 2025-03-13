#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include "GeographicUtils.h"
#include <queue>
#include <unordered_map>
#include <set>
#include <cmath>
#include <algorithm>
#include <sstream>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> Config;
    std::vector<std::shared_ptr<CStreetMap::SNode>> SortedNodes;
    std::shared_ptr<CDijkstraPathRouter> DistanceRouter;
    std::shared_ptr<CDijkstraPathRouter> TimeRouter;
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeIDToDistanceVertexID;
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeIDToTimeVertexID;
    std::unordered_map<CStreetMap::TNodeID, size_t> NodeIDToIndex;
    std::unordered_map<CBusSystem::TStopID, CStreetMap::TNodeID> StopIDToNodeID;
    std::unordered_map<CStreetMap::TNodeID, std::set<std::pair<std::string, CStreetMap::TNodeID>>> BusRouteInfo;
    
    SImplementation(std::shared_ptr<SConfiguration> config)
        : Config(config) {
        
        auto StreetMap = Config->StreetMap();
        auto BusSystem = Config->BusSystem();
        
        // Create path routers
        DistanceRouter = std::make_shared<CDijkstraPathRouter>();
        TimeRouter = std::make_shared<CDijkstraPathRouter>();
        
        // Sort nodes by ID
        for (size_t i = 0; i < StreetMap->NodeCount(); ++i) {
            auto Node = StreetMap->NodeByIndex(i);
            SortedNodes.push_back(Node);
        }
        
        std::sort(SortedNodes.begin(), SortedNodes.end(), 
                 [](const std::shared_ptr<CStreetMap::SNode>& a, const std::shared_ptr<CStreetMap::SNode>& b) {
                     return a->ID() < b->ID();
                 });
        
        // Build node ID to index map
        for (size_t i = 0; i < SortedNodes.size(); ++i) {
            NodeIDToIndex[SortedNodes[i]->ID()] = i;
        }
        
        // Build the node to vertex mappings and add vertices to the routers
        for (size_t i = 0; i < SortedNodes.size(); ++i) {
            auto node = SortedNodes[i];
            NodeIDToDistanceVertexID[node->ID()] = DistanceRouter->AddVertex(node->ID());
            NodeIDToTimeVertexID[node->ID()] = TimeRouter->AddVertex(node->ID());
        }
        
        // Map bus stops to nodes
        for (size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto stop = BusSystem->StopByIndex(i);
            StopIDToNodeID[stop->ID()] = stop->NodeID();
        }
        
        // Build bus route information
        for (size_t r = 0; r < BusSystem->RouteCount(); ++r) {
            auto route = BusSystem->RouteByIndex(r);
            auto routeName = route->Name();
            
            for (size_t i = 0; i < route->StopCount(); ++i) {
                auto currentStopID = route->GetStopID(i);
                auto currentStop = BusSystem->StopByID(currentStopID);
                auto currentNodeID = currentStop->NodeID();
                
                if (i + 1 < route->StopCount()) {
                    auto nextStopID = route->GetStopID(i + 1);
                    auto nextStop = BusSystem->StopByID(nextStopID);
                    auto nextNodeID = nextStop->NodeID();
                    
                    BusRouteInfo[currentNodeID].insert({routeName, nextNodeID});
                }
            }
        }
        
        // Add edges to routers based on ways
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto way = StreetMap->WayByIndex(i);
            
            // Check if way is valid for routing
            if (!IsWayRoutable(way)) {
                continue;
            }
            
            // Get speed limit for the way
            double speedLimit = GetSpeedLimit(way);
            
            // Check if bicycles are disallowed
            bool bicycleAllowed = IsBicycleAllowed(way);
            
            // Process each segment of the way
            for (size_t j = 1; j < way->NodeCount(); ++j) {
                auto src_id = way->GetNodeID(j-1);
                auto dest_id = way->GetNodeID(j);
                
                // Skip if any node is invalid
                if (src_id == CStreetMap::InvalidNodeID || dest_id == CStreetMap::InvalidNodeID) {
                    continue;
                }
                
                auto src_node = StreetMap->NodeByID(src_id);
                auto dest_node = StreetMap->NodeByID(dest_id);
                
                if (!src_node || !dest_node) {
                    continue;
                }
                
                // Calculate distance between nodes
                double distance = CalculateDistance(src_node, dest_node);
                
                // Get vertex IDs
                auto src_dist_vertex = NodeIDToDistanceVertexID[src_id];
                auto dest_dist_vertex = NodeIDToDistanceVertexID[dest_id];
                auto src_time_vertex = NodeIDToTimeVertexID[src_id];
                auto dest_time_vertex = NodeIDToTimeVertexID[dest_id];
                
                // Check for one-way street
                bool isOneWay = IsOneWay(way);
                
                // Add edges to distance router (respect one-way restrictions)
                DistanceRouter->AddEdge(src_dist_vertex, dest_dist_vertex, distance, !isOneWay);
                
                // Calculate time for different transportation modes
                double walkTime = distance / Config->WalkSpeed();
                double bikeTime = bicycleAllowed ? distance / Config->BikeSpeed() : std::numeric_limits<double>::infinity();
                double carTime = distance / speedLimit;
                
                // For time router, walking ignores one-way restrictions
                TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, walkTime, true);
                
                // For biking and buses, respect one-way restrictions
                if (bicycleAllowed) {
                    if (!isOneWay) {
                        // Can bike both ways
                        TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, bikeTime, false);  // Already added above for walking
                        TimeRouter->AddEdge(dest_time_vertex, src_time_vertex, bikeTime, false);
                    } else {
                        // Can bike only in the forward direction
                        TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, bikeTime, false);  // Already added above for walking
                    }
                }
            }
        }
        
        // Add bus route edges to time router
        for (const auto& [nodeID, routes] : BusRouteInfo) {
            for (const auto& [routeName, nextNodeID] : routes) {
                auto src_node = StreetMap->NodeByID(nodeID);
                auto dest_node = StreetMap->NodeByID(nextNodeID);
                
                if (!src_node || !dest_node) {
                    continue;
                }
                
                double distance = CalculateDistance(src_node, dest_node);
                double busTime = distance / Config->DefaultSpeedLimit() + Config->BusStopTime();
                
                auto src_time_vertex = NodeIDToTimeVertexID[nodeID];
                auto dest_time_vertex = NodeIDToTimeVertexID[nextNodeID];
                
                // Buses follow the shortest path, respect one-way restrictions
                TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, busTime, false);
            }
        }
        
        // Precompute paths
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(Config->PrecomputeTime());
        DistanceRouter->Precompute(deadline);
        TimeRouter->Precompute(deadline);
    }
    
    bool IsWayRoutable(const std::shared_ptr<CStreetMap::SWay>& way) const {
        if (way->HasAttribute("highway")) {
            std::string highway = way->GetAttribute("highway");
            // Exclude non-routable highway types
            if (highway == "footway" || highway == "cycleway" || highway == "path" || 
                highway == "service" || highway == "pedestrian" || highway == "track") {
                return false;
            }
            return true;
        }
        return false;
    }
    
    bool IsOneWay(const std::shared_ptr<CStreetMap::SWay>& way) const {
        if (way->HasAttribute("oneway")) {
            std::string oneway = way->GetAttribute("oneway");
            return oneway == "yes" || oneway == "true" || oneway == "1";
        }
        return false;
    }
    
    bool IsBicycleAllowed(const std::shared_ptr<CStreetMap::SWay>& way) const {
        if (way->HasAttribute("bicycle")) {
            std::string bicycle = way->GetAttribute("bicycle");
            return bicycle != "no" && bicycle != "false" && bicycle != "0";
        }
        return true; // By default, bicycles are allowed
    }
    
    double GetSpeedLimit(const std::shared_ptr<CStreetMap::SWay>& way) const {
        if (way->HasAttribute("maxspeed")) {
            try {
                return std::stod(way->GetAttribute("maxspeed"));
            } catch (...) {
                // If conversion fails, use default speed limit
            }
        }
        return Config->DefaultSpeedLimit();
    }
    
    double CalculateDistance(const std::shared_ptr<CStreetMap::SNode>& src, 
                             const std::shared_ptr<CStreetMap::SNode>& dest) const {
        // Convert miles to meters for consistency
        return SGeographicUtils::HaversineDistanceInMiles(src->Location(), dest->Location()) * 1609.34; // 1 mile = 1609.34 meters
    }
    
    std::string GetDirectionString(double angle) const {
        // Map abbreviated directions to full names
        std::string abbr = SGeographicUtils::BearingToDirection(angle);
        if (abbr == "N") return "north";
        if (abbr == "NE") return "northeast";
        if (abbr == "E") return "east";
        if (abbr == "SE") return "southeast";
        if (abbr == "S") return "south";
        if (abbr == "SW") return "southwest";
        if (abbr == "W") return "west";
        return "northwest"; // NW
    }
    
    double CalculateBearing(const std::shared_ptr<CStreetMap::SNode>& src, 
                            const std::shared_ptr<CStreetMap::SNode>& dest) const {
        return SGeographicUtils::CalculateBearing(src->Location(), dest->Location());
    }
    
    std::string GetStreetName(const std::shared_ptr<CStreetMap::SNode>& node1, 
                              const std::shared_ptr<CStreetMap::SNode>& node2) const {
        auto StreetMap = Config->StreetMap();
        
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto way = StreetMap->WayByIndex(i);
            
            bool containsNode1 = false;
            bool containsNode2 = false;
            
            for (size_t j = 0; j < way->NodeCount(); ++j) {
                auto nodeID = way->GetNodeID(j);
                if (nodeID == node1->ID()) containsNode1 = true;
                if (nodeID == node2->ID()) containsNode2 = true;
                
                // If both nodes are found in the way
                if (containsNode1 && containsNode2) {
                    if (way->HasAttribute("name")) {
                        return way->GetAttribute("name");
                    }
                    return "unnamed street";
                }
            }
        }
        
        return "unnamed street";
    }
    
    std::pair<std::string, int> FindBestBusRouteBetweenNodes(const CStreetMap::TNodeID& src, 
                                                            const CStreetMap::TNodeID& dest) const {
        std::string bestRoute = "";
        int maxDistance = -1;
        
        if (BusRouteInfo.count(src) > 0) {
            // First check for direct connections
            std::vector<std::string> possibleRoutes;
            
            for (const auto& [routeName, nextNodeID] : BusRouteInfo.at(src)) {
                if (nextNodeID == dest) {
                    possibleRoutes.push_back(routeName);
                }
            }
            
            // If multiple options available, choose the earliest sorted name
            if (!possibleRoutes.empty()) {
                std::sort(possibleRoutes.begin(), possibleRoutes.end());
                return {possibleRoutes[0], 1}; // Direct connection, distance = 1
            }
            
            // If no direct connection, find route that goes furthest
            auto BusSystem = Config->BusSystem();
            
            for (size_t r = 0; r < BusSystem->RouteCount(); ++r) {
                auto route = BusSystem->RouteByIndex(r);
                auto routeName = route->Name();
                
                // Find src and dest in this route
                int srcIndex = -1;
                int destIndex = -1;
                
                for (size_t i = 0; i < route->StopCount(); ++i) {
                    auto stopID = route->GetStopID(i);
                    auto stop = BusSystem->StopByID(stopID);
                    auto nodeID = stop->NodeID();
                    
                    if (nodeID == src) {
                        srcIndex = i;
                    }
                    if (nodeID == dest) {
                        destIndex = i;
                    }
                }
                
                // If both src and dest are on this route and src comes before dest
                if (srcIndex >= 0 && destIndex > srcIndex) {
                    int distance = destIndex - srcIndex;
                    
                    // If this is the longest route or has the same distance but earlier name
                    if (distance > maxDistance || (distance == maxDistance && routeName < bestRoute)) {
                        maxDistance = distance;
                        bestRoute = routeName;
                    }
                }
            }
        }
        
        return {bestRoute, maxDistance};
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config)
    : DImplementation(std::make_unique<SImplementation>(config)) {
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
}

std::shared_ptr<CStreetMap> CDijkstraTransportationPlanner::StreetMap() const {
    return DImplementation->Config->StreetMap();
}

std::shared_ptr<CBusSystem> CDijkstraTransportationPlanner::BusSystem() const {
    return DImplementation->Config->BusSystem();
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TTripStep>& path) {
    path.clear();
    
    // Verify source and destination are valid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return std::numeric_limits<double>::infinity();
    }
    
    // Get vertex IDs for source and destination
    auto SrcVertexID = DImplementation->NodeIDToDistanceVertexID[src];
    auto DestVertexID = DImplementation->NodeIDToDistanceVertexID[dest];
    
    // Find the shortest path
    std::vector<CPathRouter::TVertexID> RouterPath;
    double distance = DImplementation->DistanceRouter->FindShortestPath(SrcVertexID, DestVertexID, RouterPath);
    
    // Convert router path to node IDs
    for (auto VertexID : RouterPath) {
        path.push_back(DImplementation->DistanceRouter->GetVertexTag(VertexID));
    }
    
    return distance;
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep>& path) {
    path.clear();
    
    // Verify source and destination are valid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return std::numeric_limits<double>::infinity();
    }
    
    // Get vertex IDs for source and destination
    auto SrcVertexID = DImplementation->NodeIDToTimeVertexID[src];
    auto DestVertexID = DImplementation->NodeIDToTimeVertexID[dest];
    
    // Find the fastest path
    std::vector<CPathRouter::TVertexID> RouterPath;
    double time = DImplementation->TimeRouter->FindShortestPath(SrcVertexID, DestVertexID, RouterPath);
    
    // Convert router path to node IDs
    for (auto VertexID : RouterPath) {
        path.push_back(DImplementation->TimeRouter->GetVertexTag(VertexID));
    }
    
    return time;
}

std::size_t CDijkstraTransportationPlanner::NodeCount() const {
    return DImplementation->SortedNodes.size();
}

CStreetMap::TNodeID CDijkstraTransportationPlanner::GetNodeIDByIndex(std::size_t index) const {
    if (index < DImplementation->SortedNodes.size()) {
        return DImplementation->SortedNodes[index]->ID();
    }
    return CStreetMap::InvalidNodeID;
}

std::string CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep>& path) const {
    std::stringstream Description;
    auto StreetMap = DImplementation->Config->StreetMap();
    
    if (path.size() <= 1) {
        return "No path found.";
    }
    
    // First node is the starting point
    Description << "Start at node " << path[0] << "." << std::endl;
    
    // Variables to track current street and direction
    std::string currentStreet = "";
    std::string currentDirection = "";
    std::string currentTransportMode = "walking";
    std::string currentBusRoute = "";
    TNodeID startNode = path[0];
    TNodeID currentNode = path[0];
    double currentDistance = 0.0;
    
    // Process each node in the path
    for (size_t i = 1; i < path.size(); ++i) {
        auto prevNode = StreetMap->NodeByID(path[i-1]);
        auto currNode = StreetMap->NodeByID(path[i]);
        
        // Check if this is a bus route segment
        bool isBusSegment = false;
        std::string busRouteName = "";
        
        if (DImplementation->BusRouteInfo.count(path[i-1]) > 0) {
            for (const auto& [routeName, nextNodeID] : DImplementation->BusRouteInfo.at(path[i-1])) {
                if (nextNodeID == path[i]) {
                    isBusSegment = true;
                    busRouteName = routeName;
                    break;
                }
            }
        }
        
        // Get the street name for this segment
        std::string streetName = DImplementation->GetStreetName(prevNode, currNode);
        
        // Calculate bearing for this segment
        double bearing = DImplementation->CalculateBearing(prevNode, currNode);
        std::string direction = DImplementation->GetDirectionString(bearing);
        
        // Calculate distance for this segment
        double distance = DImplementation->CalculateDistance(prevNode, currNode);
        
        // Determine transportation mode changes
        if (isBusSegment && currentTransportMode != "bus") {
            // Changing from walking/biking to bus
            if (currentTransportMode != "walking" && currentTransportMode != "starting") {
                // Finish the previous segment description
                Description << "Continue " << currentDirection << " along " << currentStreet 
                          << " for " << currentDistance << " meters." << std::endl;
                currentDistance = 0.0;
            }
            
            // If we're not at the start, we need to transition to bus
            if (currentNode != path[0]) {
                Description << "Take bus " << busRouteName << " at node " << path[i-1] << "." << std::endl;
            }
            
            currentTransportMode = "bus";
            currentBusRoute = busRouteName;
            startNode = path[i-1];
            currentNode = path[i];
            currentStreet = streetName;
            currentDirection = direction;
            currentDistance = distance;
        }
        else if (!isBusSegment && currentTransportMode == "bus") {
            // Changing from bus to walking
            if (currentDistance > 0.0) {
                // Finish the bus segment description
                Description << "Take bus " << currentBusRoute << " from node " << startNode 
                          << " to node " << path[i-1] << "." << std::endl;
                currentDistance = 0.0;
            }
            
            currentTransportMode = "walking";
            startNode = path[i-1];
            currentNode = path[i];
            currentStreet = streetName;
            currentDirection = direction;
            currentDistance = distance;
        }
        else if (streetName != currentStreet || direction != currentDirection) {
            // Street or direction change, finish the previous segment
            if (currentDistance > 0.0) {
                if (currentTransportMode == "bus") {
                    Description << "Take bus " << currentBusRoute << " from node " << startNode 
                              << " to node " << path[i-1] << "." << std::endl;
                } else {
                    std::string directionWord = (currentStreet == "unnamed street") ? "toward" : "along";
                    Description << "Continue " << currentDirection << " " << directionWord << " " 
                              << currentStreet << " for " << currentDistance << " meters." << std::endl;
                }
                currentDistance = 0.0;
            }
            
            // Start a new segment
            startNode = path[i-1];
            currentNode = path[i];
            currentStreet = streetName;
            currentDirection = direction;
            currentDistance = distance;
        }
        else {
            // Continue on the same street and direction
            currentDistance += distance;
            currentNode = path[i];
        }
    }
    
    // Add the final segment
    if (currentDistance > 0.0) {
        if (currentTransportMode == "bus") {
            Description << "Take bus " << currentBusRoute << " from node " << startNode 
                      << " to node " << path.back() << "." << std::endl;
        } else {
            std::string directionWord = (currentStreet == "unnamed street") ? "toward" : "along";
            Description << "Continue " << currentDirection << " " << directionWord << " " 
                      << currentStreet << " for " << currentDistance << " meters." << std::endl;
        }
    }
    
    Description << "End at node " << path.back() << ".";
    return Description.str();
}

std::vector<std::tuple<std::string, double, double>> CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest) const {
    std::vector<std::tuple<std::string, double, double>> pathDetails;
    
    // Verify source and destination are valid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return pathDetails;
    }
    
    // Get full path
    std::vector<TTripStep> path;
    double totalTime = FindFastestPath(src, dest, path);
    
    if (path.size() <= 1) {
        return pathDetails;
    }
    
    auto StreetMap = DImplementation->Config->StreetMap();
    
    // Variables to track current mode and segment
    std::string currentMode = "walking";
    std::string currentBusRoute = "";
    TNodeID segmentStart = path[0];
    double segmentDistance = 0.0;
    double segmentTime = 0.0;
    
    // Process each node in the path
    for (size_t i = 1; i < path.size(); ++i) {
        auto prevNode = StreetMap->NodeByID(path[i-1]);
        auto currNode = StreetMap->NodeByID(path[i]);
        
        // Calculate distance for this segment
        double distance = DImplementation->CalculateDistance(prevNode, currNode);
        
        // Check if this is a bus route segment
        bool isBusSegment = false;
        std::string busRouteName = "";
        std::pair<std::string, int> bestBusRoute = {"", 0};
        
        if (DImplementation->BusRouteInfo.count(path[i-1]) > 0) {
            for (const auto& [routeName, nextNodeID] : DImplementation->BusRouteInfo.at(path[i-1])) {
                if (nextNodeID == path[i]) {
                    isBusSegment = true;
                    busRouteName = routeName;
                    break;
                }
            }
        }
        
        // Determine transportation mode changes
        if (isBusSegment && currentMode != "bus") {
            // Changing from walking to bus
            if (segmentDistance > 0.0) {
                // Finish the walking segment
                pathDetails.push_back(std::make_tuple("walking", segmentDistance, segmentTime));
                segmentDistance = 0.0;
                segmentTime = 0.0;
            }
            
            // Check if we have a best bus route that takes us further
            bestBusRoute = DImplementation->FindBestBusRouteBetweenNodes(path[i-1], path[i]);
            if (bestBusRoute.second > 0) {
                currentBusRoute = bestBusRoute.first;
            } else {
                currentBusRoute = busRouteName;
            }
            
            currentMode = "bus";
            segmentStart = path[i-1];
            
            // Calculate time for this bus segment
            double busTime = distance / DImplementation->Config->DefaultSpeedLimit() + DImplementation->Config->BusStopTime();
            segmentDistance += distance;
            segmentTime += busTime;
        }
        else if (!isBusSegment && currentMode == "bus") {
            // Changing from bus to walking
            if (segmentDistance > 0.0) {
                // Finish the bus segment
                pathDetails.push_back(std::make_tuple("bus " + currentBusRoute, segmentDistance, segmentTime));
                segmentDistance = 0.0;
                segmentTime = 0.0;
            }
            
            currentMode = "walking";
            segmentStart = path[i-1];
            
            // Calculate time for this walking segment
            double walkTime = distance / DImplementation->Config->WalkSpeed();
            segmentDistance += distance;
            segmentTime += walkTime;
        }
        else {
            // Continue in the same mode
            if (currentMode == "walking") {
                double walkTime = distance / DImplementation->Config->WalkSpeed();
                segmentDistance += distance;
                segmentTime += walkTime;
            }
            else if (currentMode == "bus") {
                double busTime = distance / DImplementation->Config->DefaultSpeedLimit() + DImplementation->Config->BusStopTime();
                segmentDistance += distance;
                segmentTime += busTime;
            }
        }
    }
    
    // Add the final segment
    if (segmentDistance > 0.0) {
        if (currentMode == "bus") {
            pathDetails.push_back(std::make_tuple("bus " + currentBusRoute, segmentDistance, segmentTime));
        } else {
            pathDetails.push_back(std::make_tuple("walking", segmentDistance, segmentTime));
        }
    }
    
    return pathDetails;
}

std::vector<std::tuple<std::string, double, double>> CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest) const {
    std::vector<std::tuple<std::string, double, double>> pathDetails;
    
    // Verify source and destination are valid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return pathDetails;
    }
    
    // Get full path
    std::vector<TTripStep> path;
    double totalDistance = FindShortestPath(src, dest, path);
    
    if (path.size() <= 1) {
        return pathDetails;
    }
    
    auto StreetMap = DImplementation->Config->StreetMap();
    
    // Variables to track current mode and segment
    std::string currentMode = "walking";
    TNodeID segmentStart = path[0];
    double segmentDistance = 0.0;
    double segmentTime = 0.0;
    
    // Process each node in the path
    for (size_t i = 1; i < path.size(); ++i) {
        auto prevNode = StreetMap->NodeByID(path[i-1]);
        auto currNode = StreetMap->NodeByID(path[i]);
        
        // Calculate distance and time for this segment
        double distance = DImplementation->CalculateDistance(prevNode, currNode);
        double walkTime = distance / DImplementation->Config->WalkSpeed();
        
        segmentDistance += distance;
        segmentTime += walkTime;
    }
    
    // Add the walking segment
    pathDetails.push_back(std::make_tuple("walking", segmentDistance, segmentTime));
    
    return pathDetails;
}