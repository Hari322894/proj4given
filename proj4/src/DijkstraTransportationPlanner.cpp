#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include "GeographicUtils.h"
#include <queue>
#include <unordered_map>
#include <set>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> Config;
    std::vector<std::shared_ptr<CStreetMap::SNode>> SortedNodes;
    std::shared_ptr<CDijkstraPathRouter> DistanceRouter;
    std::shared_ptr<CDijkstraPathRouter> TimeRouter;
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeIDToDistanceVertexID;
    std::unordered_map<CStreetMap::TNodeID, CPathRouter::TVertexID> NodeIDToTimeVertexID;
    std::unordered_map<CPathRouter::TVertexID, CStreetMap::TNodeID> DistanceVertexIDToNodeID;
    std::unordered_map<CPathRouter::TVertexID, CStreetMap::TNodeID> TimeVertexIDToNodeID;
    std::unordered_map<CStreetMap::TNodeID, size_t> NodeIDToIndex;
    std::unordered_map<CBusSystem::TStopID, CStreetMap::TNodeID> StopIDToNodeID;
    std::unordered_map<CBusSystem::TStopID, std::string> StopIDToStopName;
    std::unordered_map<CStreetMap::TNodeID, CBusSystem::TStopID> NodeIDToStopID;
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
            auto distVertexID = DistanceRouter->AddVertex(node->ID());
            auto timeVertexID = TimeRouter->AddVertex(node->ID());
            
            NodeIDToDistanceVertexID[node->ID()] = distVertexID;
            NodeIDToTimeVertexID[node->ID()] = timeVertexID;
            
            // Store reverse mappings for lookup
            DistanceVertexIDToNodeID[distVertexID] = node->ID();
            TimeVertexIDToNodeID[timeVertexID] = node->ID();
        }
        
        // Map bus stops to nodes
        for (size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto stop = BusSystem->StopByIndex(i);
            StopIDToNodeID[stop->ID()] = stop->NodeID();
            NodeIDToStopID[stop->NodeID()] = stop->ID();
        }
        
        // Build bus route information
        for (size_t r = 0; r < BusSystem->RouteCount(); ++r) {
            auto route = BusSystem->RouteByIndex(r);
            auto routeName = route->Name();
            
            for (size_t i = 0; i < route->StopCount() - 1; ++i) {
                auto currentStopID = route->GetStopID(i);
                auto nextStopID = route->GetStopID(i + 1);
                
                auto currentStop = BusSystem->StopByID(currentStopID);
                auto nextStop = BusSystem->StopByID(nextStopID);
                
                auto currentNodeID = currentStop->NodeID();
                auto nextNodeID = nextStop->NodeID();
                
                BusRouteInfo[currentNodeID].insert({routeName, nextNodeID});
            }
        }
        
        // Add edges for walking, biking and driving
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto way = StreetMap->WayByIndex(i);
            
            // Skip non-routable ways
            if (way->NodeCount() < 2 || !way->HasAttribute("highway")) {
                continue;
            }
            
            std::string highway = way->GetAttribute("highway");
            bool is_oneway = false;
            
            if (way->HasAttribute("oneway")) {
                std::string oneway = way->GetAttribute("oneway");
                is_oneway = (oneway == "yes" || oneway == "true" || oneway == "1");
            }
            
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
                double distance = SGeographicUtils::HaversineDistanceInMiles(src_node->Location(), dest_node->Location());
                
                // Skip if distance is zero or negative
                if (distance <= 0.0) {
                    continue;
                }
                
                // Get vertex IDs
                auto src_dist_vertex = NodeIDToDistanceVertexID[src_id];
                auto dest_dist_vertex = NodeIDToDistanceVertexID[dest_id];
                
                // Add edge to distance router (follow oneway directions)
                DistanceRouter->AddEdge(src_dist_vertex, dest_dist_vertex, distance, true);
                if (!is_oneway) {
                    DistanceRouter->AddEdge(dest_dist_vertex, src_dist_vertex, distance, false);
                }
                
                // Get time router vertex IDs
                auto src_time_vertex = NodeIDToTimeVertexID[src_id];
                auto dest_time_vertex = NodeIDToTimeVertexID[dest_id];
                
                // Walking (both directions regardless of oneway)
                double walk_time = distance / Config->WalkSpeed();
                TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, walk_time, false);
                TimeRouter->AddEdge(dest_time_vertex, src_time_vertex, walk_time, false);
                
                // Biking (respect oneway)
                double bike_time = distance / Config->BikeSpeed();
                TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, bike_time, false);
                if (!is_oneway) {
                    TimeRouter->AddEdge(dest_time_vertex, src_time_vertex, bike_time, false);
                }
                
                // Driving/Bus (respect oneway and use speed limit)
                double speed_limit = Config->DefaultSpeedLimit();
                if (way->HasAttribute("maxspeed")) {
                    try {
                        std::string maxspeed = way->GetAttribute("maxspeed");
                        // Handle "20 mph" format by extracting the number
                        size_t spacePos = maxspeed.find(' ');
                        if (spacePos != std::string::npos) {
                            maxspeed = maxspeed.substr(0, spacePos);
                        }
                        speed_limit = std::stod(maxspeed);
                    } catch (...) {
                        // If conversion fails, use default speed limit
                    }
                }
                
                double drive_time = distance / speed_limit;
                TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, drive_time, false);
                if (!is_oneway) {
                    TimeRouter->AddEdge(dest_time_vertex, src_time_vertex, drive_time, false);
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
                
                double distance = SGeographicUtils::HaversineDistanceInMiles(src_node->Location(), dest_node->Location());
                double bus_time = distance / Config->DefaultSpeedLimit() + Config->BusStopTime();
                
                auto src_time_vertex = NodeIDToTimeVertexID[nodeID];
                auto dest_time_vertex = NodeIDToTimeVertexID[nextNodeID];
                
                TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, bus_time, false);
            }
        }
        
        // Precompute paths
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(Config->PrecomputeTime());
        DistanceRouter->Precompute(deadline);
        TimeRouter->Precompute(deadline);
    }

    std::string FindBusRouteBetweenNodes(const CStreetMap::TNodeID& src, 
        const CStreetMap::TNodeID& dest) const {
        if (BusRouteInfo.count(src) == 0) {
            return "";
        }

        // Check if there's a direct bus route between the nodes
        std::vector<std::string> directRoutes;

        for (const auto& [routeName, nextNodeID] : BusRouteInfo.at(src)) {
            if (nextNodeID == dest) {
                directRoutes.push_back(routeName);
            }
        }

        // If there are direct routes, return the earliest sorted name
        if (!directRoutes.empty()) {
            std::sort(directRoutes.begin(), directRoutes.end());
            return directRoutes[0];
        }

        return "";
    }
    
    std::string FormatLocation(const std::shared_ptr<CStreetMap::SNode>& node) const {
        double lat = node->Location().first;
        double lon = node->Location().second;
        
        // Separate degrees, minutes, seconds
        int lat_deg = static_cast<int>(lat);
        double lat_min_full = std::abs(lat - lat_deg) * 60.0;
        int lat_min = static_cast<int>(lat_min_full);
        int lat_sec = static_cast<int>((lat_min_full - lat_min) * 60.0);
        
        int lon_deg = static_cast<int>(std::abs(lon));
        double lon_min_full = (std::abs(lon) - lon_deg) * 60.0;
        int lon_min = static_cast<int>(lon_min_full);
        int lon_sec = static_cast<int>((lon_min_full - lon_min) * 60.0);
        
        std::stringstream ss;
        ss << lat_deg << "d " << lat_min << "' " << lat_sec << "\" " 
           << (lat >= 0 ? "N" : "S") << ", "
           << lon_deg << "d " << lon_min << "' " << lon_sec << "\" " 
           << (lon >= 0 ? "E" : "W");
        return ss.str();
    }
    
    std::string GetDirectionString(double angle) const {
        // Map abbreviated directions to full names
        std::string abbr = SGeographicUtils::BearingToDirection(angle);
        if (abbr == "N") return "N";
        if (abbr == "NE") return "NE";
        if (abbr == "E") return "E";
        if (abbr == "SE") return "SE";
        if (abbr == "S") return "S";
        if (abbr == "SW") return "SW";
        if (abbr == "W") return "W";
        return "NW"; // NW
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
            
            bool foundNode1 = false;
            bool foundNode2 = false;
            
            for (size_t j = 0; j < way->NodeCount(); ++j) {
                auto nodeID = way->GetNodeID(j);
                if (nodeID == node1->ID()) foundNode1 = true;
                if (nodeID == node2->ID()) foundNode2 = true;
                
                // If both nodes are found in the way
                if (foundNode1 && foundNode2) {
                    if (way->HasAttribute("name")) {
                        return way->GetAttribute("name");
                    }
                    return "unnamed street";
                }
            }
        }
        
        return "unnamed street";
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config)
    : DImplementation(std::make_unique<SImplementation>(config)) {
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() = default;

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->SortedNodes.size();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->SortedNodes.size()) {
        return DImplementation->SortedNodes[index];
    }
    return nullptr;
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    // Clear the output path
    path.clear();
    
    // Make sure source and destination are valid
    if (DImplementation->NodeIDToDistanceVertexID.find(src) == DImplementation->NodeIDToDistanceVertexID.end() ||
        DImplementation->NodeIDToDistanceVertexID.find(dest) == DImplementation->NodeIDToDistanceVertexID.end()) {
        return CPathRouter::NoPathExists; // Invalid nodes
    }
    
    // Get vertex IDs for the source and destination
    auto srcVertex = DImplementation->NodeIDToDistanceVertexID[src];
    auto destVertex = DImplementation->NodeIDToDistanceVertexID[dest];
    
    // Find shortest path using the distance router
    std::vector<CPathRouter::TVertexID> routerPath;
    double distance = DImplementation->DistanceRouter->FindShortestPath(srcVertex, destVertex, routerPath);
    
    // If no path found or distance is infinite
    if (distance < 0.0) {
        return CPathRouter::NoPathExists;
    }
    
    // Convert router path (vertex IDs) back to node IDs
    for (const auto& vertexID : routerPath) {
        path.push_back(DImplementation->DistanceVertexIDToNodeID[vertexID]);
    }
    
    return distance;
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    // Clear the output path
    path.clear();
    
    // Make sure source and destination are valid
    if (DImplementation->NodeIDToTimeVertexID.find(src) == DImplementation->NodeIDToTimeVertexID.end() ||
        DImplementation->NodeIDToTimeVertexID.find(dest) == DImplementation->NodeIDToTimeVertexID.end()) {
        return CPathRouter::NoPathExists; // Invalid nodes
    }
    
    // Get vertex IDs for the source and destination
    auto srcVertex = DImplementation->NodeIDToTimeVertexID[src];
    auto destVertex = DImplementation->NodeIDToTimeVertexID[dest];
    
    // Find fastest path using the time router
    std::vector<CPathRouter::TVertexID> routerPath;
    double time = DImplementation->TimeRouter->FindShortestPath(srcVertex, destVertex, routerPath);
    
    // If no path found or time is infinite
    if (time < 0.0) {
        return CPathRouter::NoPathExists;
    }
    
    // Convert router path (vertex IDs) to trip steps
    auto StreetMap = DImplementation->Config->StreetMap();
    
    // Process each node in the path
    for (size_t i = 0; i < routerPath.size(); ++i) {
        auto currentNodeID = DImplementation->TimeVertexIDToNodeID[routerPath[i]];
        
        // Determine transportation mode (default to Walk)
        ETransportationMode mode = ETransportationMode::Walk;
        
        // If not at the first node, determine the mode based on the previous node
        if (i > 0) {
            auto prevNodeID = DImplementation->TimeVertexIDToNodeID[routerPath[i-1]];
            auto prevNode = StreetMap->NodeByID(prevNodeID);
            auto currentNode = StreetMap->NodeByID(currentNodeID);
            
            // Check if there's a bus route between these nodes
            std::string busRoute = DImplementation->FindBusRouteBetweenNodes(prevNodeID, currentNodeID);
            if (!busRoute.empty()) {
                mode = ETransportationMode::Bus;
            } 
            // Check if biking is faster than walking
            else {
                double distance = SGeographicUtils::HaversineDistanceInMiles(
                    prevNode->Location(), currentNode->Location());
                
                double walkTime = distance / DImplementation->Config->WalkSpeed();
                double bikeTime = distance / DImplementation->Config->BikeSpeed();
                
                if (bikeTime < walkTime) {
                    mode = ETransportationMode::Bike;
                }
            }
        }
        
        // Add this step to the path
        path.push_back({mode, currentNodeID});
    }
    
    // Optimize the path by combining consecutive steps with the same transportation mode
    // This needs to remain for each node to keep the nodes in the correct order
    if (path.size() > 1) {
        for (size_t i = 1; i < path.size(); ++i) {
            if (i < path.size() && path[i].first == path[i-1].first) {
                path[i-1].second = path[i].second;
                path.erase(path.begin() + i);
                --i; // Adjust index after erasure
            }
        }
    }
    
    return time;
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    desc.clear();
    
    if (path.empty()) {
        return false;
    }
    
    auto StreetMap = DImplementation->Config->StreetMap();
    auto BusSystem = DImplementation->Config->BusSystem();
    
    // Add starting point description
    auto startNode = StreetMap->NodeByID(path[0].second);
    if (!startNode) {
        return false;
    }
    
    desc.push_back("Start at " + DImplementation->FormatLocation(startNode));
    
    // Process each step in the path
    for (size_t i = 1; i < path.size(); ++i) {
        auto prevNode = StreetMap->NodeByID(path[i-1].second);
        auto currentNode = StreetMap->NodeByID(path[i].second);
        
        if (!prevNode || !currentNode) {
            return false;
        }
        
        // Get the transportation mode for this step
        auto mode = path[i].first;
        
        // Calculate distance between nodes
        double distance = SGeographicUtils::HaversineDistanceInMiles(
            prevNode->Location(), currentNode->Location());
        
        // Calculate bearing/direction
        double bearing = DImplementation->CalculateBearing(prevNode, currentNode);
        std::string direction = DImplementation->GetDirectionString(bearing);
        
        // Get street name
        std::string streetName = DImplementation->GetStreetName(prevNode, currentNode);
        
        std::stringstream ss;
        
        if (mode == ETransportationMode::Walk) {
            ss << "Walk " << direction;
            if (streetName != "unnamed street") {
                ss << " along " << streetName;
            } else {
                ss << " toward End";
            }
            ss << " for " << std::fixed << std::setprecision(1) << distance << " mi";
            desc.push_back(ss.str());
        } 
        else if (mode == ETransportationMode::Bike) {
            ss << "Bike " << direction;
            if (streetName != "unnamed street") {
                ss << " along " << streetName;
            } else {
                ss << " toward End";
            }
            ss << " for " << std::fixed << std::setprecision(1) << distance << " mi";
            desc.push_back(ss.str());
        } 
        else if (mode == ETransportationMode::Bus) {
            // Check if nodes are bus stops
            if (DImplementation->NodeIDToStopID.find(prevNode->ID()) != DImplementation->NodeIDToStopID.end() &&
                DImplementation->NodeIDToStopID.find(currentNode->ID()) != DImplementation->NodeIDToStopID.end()) {
                
                auto srcStopID = DImplementation->NodeIDToStopID.at(prevNode->ID());
                auto destStopID = DImplementation->NodeIDToStopID.at(currentNode->ID());
                
                // Find the bus route for this trip
                std::string busRoute = DImplementation->FindBusRouteBetweenNodes(prevNode->ID(), currentNode->ID());
                
                ss << "Take Bus " << busRoute << " from stop " << srcStopID << " to stop " << destStopID;
                desc.push_back(ss.str());
            } else {
                // If nodes aren't both bus stops, fall back to walking description
                ss << "Walk " << direction;
                if (streetName != "unnamed street") {
                    ss << " along " << streetName;
                } else {
                    ss << " toward End";
                }
                ss << " for " << std::fixed << std::setprecision(1) << distance << " mi";
                desc.push_back(ss.str());
            }
        }
    }
    
    // Add ending point description
    auto endNode = StreetMap->NodeByID(path.back().second);
    if (!endNode) {
        return false;
    }
    desc.push_back("End at " + DImplementation->FormatLocation(endNode));
    
    return true;
}