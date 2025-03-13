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
                
                // Add edges to distance router (bidirectional if the way is not oneway)
                bool is_oneway = IsOneWay(way);
                DistanceRouter->AddEdge(src_dist_vertex, dest_dist_vertex, distance, !is_oneway);
                
                // Add edges to time router
                // Calculate time for different transportation modes
                double walk_time = distance / Config->WalkSpeed();
                double bike_time = distance / Config->BikeSpeed();
                double car_time = distance / speedLimit;
                
                // Use the minimum time for the edge
                double min_time = std::min(walk_time, std::min(bike_time, car_time));
                TimeRouter->AddEdge(src_time_vertex, dest_time_vertex, min_time, !is_oneway);
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
                double bus_time = distance / Config->BusSpeed() + Config->BusStopTime();
                
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
// Convert miles to meters for consistency with the existing implementation
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
    
    std::string FindBusRouteBetweenNodes(const CStreetMap::TNodeID& src, 
                                         const CStreetMap::TNodeID& dest) const {
        if (BusRouteInfo.count(src) > 0) {
            for (const auto& [routeName, nextNodeID] : BusRouteInfo.at(src)) {
                if (nextNodeID == dest) {
                    return routeName;
                }
            }
        }
        return "";
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

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID>& path) {
    path.clear();
    
    // Check if source or destination is invalid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return CPathRouter::NoPathExists; // Use the proper constant for no path
    }
    
    // Check if nodes exist in our map
    if (DImplementation->NodeIDToDistanceVertexID.count(src) == 0 || 
        DImplementation->NodeIDToDistanceVertexID.count(dest) == 0) {
        return CPathRouter::NoPathExists; // Use the proper constant for no path
    }
    
    // Special case: src and dest are the same
    if (src == dest) {
        path.push_back(src);
        return 0.0; // Success indicator
    }
    
    // Get vertex IDs
    auto src_vertex = DImplementation->NodeIDToDistanceVertexID[src];
    auto dest_vertex = DImplementation->NodeIDToDistanceVertexID[dest];
    
    // Find path using distance router
    std::vector<CPathRouter::TVertexID> router_path;
    double distance = DImplementation->DistanceRouter->FindShortestPath(src_vertex, dest_vertex, router_path);
    
    if (distance == CPathRouter::NoPathExists || router_path.empty()) {
        return CPathRouter::NoPathExists; // Use the proper constant for no path
    }
    
    // Convert vertex IDs back to node IDs
    for (auto vertex_id : router_path) {
        // The vertex tag is the same as the node ID in this implementation
        path.push_back(std::any_cast<TNodeID>(DImplementation->DistanceRouter->GetVertexTag(vertex_id)));
    }
    
    return distance; // Success indicator
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep>& path) {
    path.clear();
    
    // Check if source or destination is invalid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return CPathRouter::NoPathExists; // Use the proper constant for no path
    }
    
    // Check if nodes exist in our map
    if (DImplementation->NodeIDToTimeVertexID.count(src) == 0 || 
        DImplementation->NodeIDToTimeVertexID.count(dest) == 0) {
        return CPathRouter::NoPathExists; // Use the proper constant for no path
    }
    
    // Special case: src and dest are the same
    if (src == dest) {
        path.push_back({ETransportationMode::Walk, src});
        return 0.0; // Success indicator
    }
    
    // Get vertex IDs
    auto src_vertex = DImplementation->NodeIDToTimeVertexID[src];
    auto dest_vertex = DImplementation->NodeIDToTimeVertexID[dest];
    
    // Find path using time router
    std::vector<CPathRouter::TVertexID> router_path;
    double time = DImplementation->TimeRouter->FindShortestPath(src_vertex, dest_vertex, router_path);
    
    if (time == CPathRouter::NoPathExists || router_path.empty()) {
        return CPathRouter::NoPathExists; // Use the proper constant for no path
    }
    
    // Convert vertex IDs to trip steps
    for (size_t i = 0; i < router_path.size(); ++i) {
        auto node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(router_path[i]));
        
        // Determine transportation mode
        ETransportationMode mode = ETransportationMode::Walk;
        
        if (i > 0) {
            auto prev_node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(router_path[i-1]));
            
            // Check if this segment is on a bus route
            std::string busRoute = DImplementation->FindBusRouteBetweenNodes(prev_node_id, node_id);
            if (!busRoute.empty()) {
                mode = ETransportationMode::Bus;
            } else {
                // Check if we should continue the same transportation mode
                if (i > 1 && path[i-1].first == ETransportationMode::Bus) {
                    auto prev_prev_node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(router_path[i-2]));
                    std::string prevBusRoute = DImplementation->FindBusRouteBetweenNodes(prev_prev_node_id, prev_node_id);
                    
                    // Continue bus mode if we're still on the same bus route
                    if (!prevBusRoute.empty()) {
                        mode = ETransportationMode::Bus;
                    }
                }
            }
        }
        
        path.push_back({mode, node_id});
    }
    
    return time; // Success indicator
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
    desc.clear();
    
    if (path.empty()) {
        return false;
    }
    
    auto StreetMap = DImplementation->Config->StreetMap();
    auto BusSystem = DImplementation->Config->BusSystem();
    
    // Add starting instruction
    auto start_node = StreetMap->NodeByID(path[0].second);
    if (!start_node) {
        return false;
    }
    
    desc.push_back("Start at node " + std::to_string(path[0].second));
    
    std::string current_bus_route = "";
    std::string last_street_name = "";
    ETransportationMode last_mode = path[0].first;
    
    for (size_t i = 1; i < path.size(); ++i) {
        auto prev_step = path[i-1];
        auto current_step = path[i];
        auto prev_node = StreetMap->NodeByID(prev_step.second);
        auto current_node = StreetMap->NodeByID(current_step.second);
        
        if (!prev_node || !current_node) {
            return false;
        }
        
        // Handle transportation mode changes
        if (prev_step.first != current_step.first) {
            if (current_step.first == ETransportationMode::Bus) {
                // Find the bus route
                current_bus_route = DImplementation->FindBusRouteBetweenNodes(prev_step.second, current_step.second);
                if (!current_bus_route.empty()) {
                    desc.push_back("Take bus " + current_bus_route);
                }
                last_mode = ETransportationMode::Bus;
            } else if (prev_step.first == ETransportationMode::Bus) {
                desc.push_back("Get off bus " + current_bus_route);
                current_bus_route = "";
                last_mode = ETransportationMode::Walk;
            }
        }
        
        // Skip adding direction instructions if we're on a bus
        if (last_mode == ETransportationMode::Bus) {
            continue;
        }
        
        // Get direction and street name
        double bearing = DImplementation->CalculateBearing(prev_node, current_node);
        std::string direction = DImplementation->GetDirectionString(bearing);
        std::string street_name = DImplementation->GetStreetName(prev_node, current_node);
        
        // Check if we're continuing on the same street
        if (street_name == last_street_name && !last_street_name.empty()) {
            continue; // Skip redundant instructions
        }
        
        last_street_name = street_name;
        
        // Add instruction
        std::stringstream ss;
        ss << "Go " << direction << " on " << street_name << " to node " << current_step.second;
        desc.push_back(ss.str());
    }
    
    // Add ending instruction
    desc.push_back("Arrive at node " + std::to_string(path.back().second));
    
    return true;
}