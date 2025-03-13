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
            NodeIDToDistanceVertexID[node->ID()] = DistanceRouter->AddVertex(node->ID());
            NodeIDToTimeVertexID[node->ID()] = TimeRouter->AddVertex(node->ID());
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
                
                // Get vertex IDs
                auto src_dist_vertex = NodeIDToDistanceVertexID[src_id];
                auto dest_dist_vertex = NodeIDToDistanceVertexID[dest_id];
                
                // Add edge to distance router (follow oneway directions)
                DistanceRouter->AddEdge(src_dist_vertex, dest_dist_vertex, distance, true);
                if (!is_oneway) {
                    DistanceRouter->AddEdge(dest_dist_vertex, src_dist_vertex, distance, true);
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
        double lat_min_full = (lat - lat_deg) * 60.0;
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

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID>& path) {
    path.clear();
    
    // Check if source or destination is invalid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return CPathRouter::NoPathExists;
    }
    
    // Check if nodes exist in our map
    if (DImplementation->NodeIDToDistanceVertexID.count(src) == 0 || 
        DImplementation->NodeIDToDistanceVertexID.count(dest) == 0) {
        return CPathRouter::NoPathExists;
    }
    
    // Special case: src and dest are the same
    if (src == dest) {
        path.push_back(src);
        return 0.0;
    }
    
    // Get vertex IDs
    auto src_vertex = DImplementation->NodeIDToDistanceVertexID[src];
    auto dest_vertex = DImplementation->NodeIDToDistanceVertexID[dest];
    
    // Find path using distance router
    std::vector<CPathRouter::TVertexID> router_path;
    double distance = DImplementation->DistanceRouter->FindShortestPath(src_vertex, dest_vertex, router_path);
    
    if (distance == CPathRouter::NoPathExists || router_path.empty()) {
        return CPathRouter::NoPathExists;
    }
    
    // Convert vertex IDs back to node IDs
    for (auto vertex_id : router_path) {
        // The vertex tag is the same as the node ID in this implementation
        path.push_back(std::any_cast<TNodeID>(DImplementation->DistanceRouter->GetVertexTag(vertex_id)));
    }
    
    return distance;
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep>& path) {
    path.clear();
    
    // Check if source or destination is invalid
    if (src == CStreetMap::InvalidNodeID || dest == CStreetMap::InvalidNodeID) {
        return CPathRouter::NoPathExists;
    }
    
    // Check if nodes exist in our map
    if (DImplementation->NodeIDToTimeVertexID.count(src) == 0 || 
        DImplementation->NodeIDToTimeVertexID.count(dest) == 0) {
        return CPathRouter::NoPathExists;
    }
    
    // Special case: src and dest are the same
    if (src == dest) {
        path.push_back({ETransportationMode::Walk, src});
        return 0.0;
    }
    
    // Get vertex IDs
    auto src_vertex = DImplementation->NodeIDToTimeVertexID[src];
    auto dest_vertex = DImplementation->NodeIDToTimeVertexID[dest];
    
    // Find path using time router
    std::vector<CPathRouter::TVertexID> router_path;
    double time = DImplementation->TimeRouter->FindShortestPath(src_vertex, dest_vertex, router_path);
    
    if (time == CPathRouter::NoPathExists || router_path.empty()) {
        return CPathRouter::NoPathExists;
    }
    
    // Convert vertex IDs to trip steps
    auto StreetMap = DImplementation->Config->StreetMap();
    std::string currentBusRoute = "";
    ETransportationMode previousMode = ETransportationMode::Walk; // Start with walking
    
    // Find the best mode of transport for this path
    bool canUseBike = true;
    double bikeDistance = 0.0;
    double walkDistance = 0.0;
    double totalDistance = 0.0;
    
    for (size_t i = 1; i < router_path.size(); ++i) {
        auto prev_node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(router_path[i-1]));
        auto curr_node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(router_path[i]));
        
        auto prev_node = StreetMap->NodeByID(prev_node_id);
        auto curr_node = StreetMap->NodeByID(curr_node_id);
        
        double segmentDistance = SGeographicUtils::HaversineDistanceInMiles(
            prev_node->Location(), curr_node->Location());
            
        totalDistance += segmentDistance;
        
        // Check if bus can be used for this segment
        std::string busRoute = DImplementation->FindBusRouteBetweenNodes(prev_node_id, curr_node_id);
        if (!busRoute.empty()) {
            path.clear();
            
            // Start with walking to the first bus stop
            path.push_back({ETransportationMode::Walk, std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(router_path[0]))});
            
            // Take the bus for the rest of the journey
            for (size_t j = 1; j < router_path.size(); ++j) {
                auto node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(router_path[j]));
                path.push_back({ETransportationMode::Bus, node_id});
            }
            
            return time;
        }
    }
    
    // If we can't use bus, check if biking is faster than walking
    double bikeTime = totalDistance / DImplementation->Config->BikeSpeed();
    double walkTime = totalDistance / DImplementation->Config->WalkSpeed();
    
    if (bikeTime < walkTime) {
        // Use bike for the entire journey
        for (auto vertex_id : router_path) {
            auto node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(vertex_id));
            path.push_back({ETransportationMode::Bike, node_id});
        }
    } else {
        // Use walking for the entire journey
        for (auto vertex_id : router_path) {
            auto node_id = std::any_cast<TNodeID>(DImplementation->TimeRouter->GetVertexTag(vertex_id));
            path.push_back({ETransportationMode::Walk, node_id});
        }
    }
    
    return time;
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep>& path, std::vector<std::string>& desc) const {
    desc.clear();
    
    if (path.empty()) {
        return false;
    }
    
    auto StreetMap = DImplementation->Config->StreetMap();
    auto BusSystem = DImplementation->Config->BusSystem();
    
    // Check first node
    auto start_node = StreetMap->NodeByID(path[0].second);
    if (!start_node) {
        return false;
    }
    
    // Format start location
    std::string startLocation = DImplementation->FormatLocation(start_node);
    desc.push_back("Start at " + startLocation);
    
    std::string current_bus_route = "";
    std::string current_street_name = "";
    std::string prev_street_name = "";
    ETransportationMode current_mode = path[0].first;
    
    double segment_distance = 0.0;
    TNodeID segment_start = path[0].second;
    std::string segment_direction = "";
    
    for (size_t i = 1; i < path.size(); ++i) {
        auto prev_step = path[i-1];
        auto current_step = path[i];
        auto prev_node = StreetMap->NodeByID(prev_step.second);
        auto current_node = StreetMap->NodeByID(current_step.second);
        
        if (!prev_node || !current_node) {
            return false;
        }
        
        // Calculate direction and street name
        double bearing = DImplementation->CalculateBearing(prev_node, current_node);
        std::string direction = DImplementation->GetDirectionString(bearing);
        std::string street_name = DImplementation->GetStreetName(prev_node, current_node);
        
        // Calculate segment distance
        double distance = SGeographicUtils::HaversineDistanceInMiles(
            prev_node->Location(), current_node->Location());
        
        // Check for transportation mode changes
        if (prev_step.first != current_step.first) {
            // Add description for previous segment if needed
            if (segment_distance > 0.0) {
                std::stringstream ss;
                if (current_mode == ETransportationMode::Walk) {
                    ss << "Walk ";
                } else if (current_mode == ETransportationMode::Bike) {
                    ss << "Bike ";
                }
                
                ss << segment_direction << " along " << prev_street_name << " for " 
                   << std::fixed << std::setprecision(1) << segment_distance << " mi";
                desc.push_back(ss.str());
                
                segment_distance = 0.0;
            }
            
            // Update transportation mode
            if (current_step.first == ETransportationMode::Bus) {
                current_bus_route = DImplementation->FindBusRouteBetweenNodes(prev_step.second, current_step.second);
                
                // Find stop IDs
                auto startStopID = DImplementation->NodeIDToStopID.count(prev_step.second) ? 
                                 DImplementation->NodeIDToStopID.at(prev_step.second) : "";
                auto endStopID = DImplementation->NodeIDToStopID.count(path.back().second) ? 
                               DImplementation->NodeIDToStopID.at(path.back().second) : "";
                
                // Create bus instruction
                std::stringstream ss;
                ss << "Take Bus " << current_bus_route << " from stop " << startStopID << " to stop " << endStopID;
                desc.push_back(ss.str());
                
                current_mode = ETransportationMode::Bus;
                segment_start = current_step.second;
            } 
            else if (prev_step.first == ETransportationMode::Bus) {
                current_bus_route = "";
                current_mode = current_step.first;
                segment_start = current_step.second;
                segment_direction = direction;
                prev_street_name = street_name;
            }
            else if (current_step.first == ETransportationMode::Bike) {
                current_mode = ETransportationMode::Bike;
                segment_start = prev_step.second;
                segment_direction = direction;
                prev_street_name = street_name;
                segment_distance = distance;
            }
            else if (prev_step.first == ETransportationMode::Bike) {
                current_mode = ETransportationMode::Walk;
                segment_start = prev_step.second;
                segment_direction = direction;
                prev_street_name = street_name;
                segment_distance = distance;
            }
        }
        else if (current_mode != ETransportationMode::Bus) {
            // If continuing on the same mode but street name changes
            if (street_name != prev_street_name && i > 1) {
                // Add description for previous segment
                if (segment_distance > 0.0) {
                    std::stringstream ss;
                    if (current_mode == ETransportationMode::Walk) {
                        ss << "Walk ";
                    } else if (current_mode == ETransportationMode::Bike) {
                        ss << "Bike ";
                    }
                    
                    ss << segment_direction << " along " << prev_street_name << " for " 
                       << std::fixed << std::setprecision(1) << segment_distance << " mi";
                    desc.push_back(ss.str());
                    
                    segment_distance = 0.0;
                    segment_start = prev_step.second;
                    segment_direction = direction;
                }
                
                prev_street_name = street_name;
                segment_distance = distance;
            }
            else {
                // Continue on the same street
                segment_distance += distance;
            }
        }
        
        // If this is the last node and we have a pending segment description
        if (i == path.size() - 1 && current_mode != ETransportationMode::Bus && segment_distance > 0.0) {
            std::stringstream ss;
            if (current_mode == ETransportationMode::Walk) {
                ss << "Walk ";
            } else if (current_mode == ETransportationMode::Bike) {
                ss << "Bike ";
            }
            
            if (street_name == "unnamed street") {
                ss << segment_direction << " toward End for " 
                   << std::fixed << std::setprecision(1) << segment_distance << " mi";
            } else {
                ss << segment_direction << " along " << street_name << " for " 
                   << std::fixed << std::setprecision(1) << segment_distance << " mi";
            }
            desc.push_back(ss.str());
        }
    }
    
    // Add ending instruction
    auto end_node = StreetMap->NodeByID(path.back().second);
    if (!end_node) {
        return false;
    }
    
    std::string endLocation = DImplementation->FormatLocation(end_node);
    desc.push_back("End at " + endLocation);
    
    return true;
}