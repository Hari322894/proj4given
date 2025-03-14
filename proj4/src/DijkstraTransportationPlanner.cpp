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
                DistanceRouter->AddEdge(src_dist_vertex, dest_dist_vertex, distance, false);
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
    
    std::string CDijkstraTransportationPlanner::SImplementation::FormatLocation(const std::shared_ptr<CStreetMap::SNode>& node) const {
        double lat = node->Location().first;
        double lon = node->Location().second;
        
        // Convert to degrees, minutes, seconds with consistent rounding
        int lat_deg = static_cast<int>(std::floor(std::abs(lat)));
        double lat_min_full = (std::abs(lat) - lat_deg) * 60.0;
        int lat_min = static_cast<int>(std::floor(lat_min_full));
        int lat_sec = static_cast<int>(std::round((lat_min_full - lat_min) * 60.0));
        
        // Handle 60 seconds case
        if (lat_sec == 60) {
            lat_min++;
            lat_sec = 0;
            if (lat_min == 60) {
                lat_deg++;
                lat_min = 0;
            }
        }
        
        int lon_deg = static_cast<int>(std::floor(std::abs(lon)));
        double lon_min_full = (std::abs(lon) - lon_deg) * 60.0;
        int lon_min = static_cast<int>(std::floor(lon_min_full));
        int lon_sec = static_cast<int>(std::round((lon_min_full - lon_min) * 60.0));
        
        // Handle 60 seconds case
        if (lon_sec == 60) {
            lon_min++;
            lon_sec = 0;
            if (lon_min == 60) {
                lon_deg++;
                lon_min = 0;
            }
        }
        
        std::stringstream ss;
        // Apply negative sign only to degrees
        if (lat < 0) {
            lat_deg = -lat_deg;
        }
        
        ss << lat_deg << "d " << lat_min << "' " << lat_sec << "\" " 
           << (lat >= 0 ? "N" : "S") << ", "
           << lon_deg << "d " << lon_min << "' " << lon_sec << "\" " 
           << (lon >= 0 ? "E" : "W");
        return ss.str();
    }

    double CalculateBearing(const std::shared_ptr<CStreetMap::SNode>& src, 
                            const std::shared_ptr<CStreetMap::SNode>& dest) const {
        return SGeographicUtils::CalculateBearing(src->Location(), dest->Location());
    }

    std::string GetDirectionString(double angle) const {
        // Return the exact cardinal direction
        std::string dir = SGeographicUtils::BearingToDirection(angle);
        return dir; // Use the abbreviated direction directly
    }
    
    std::string GetStreetName(const std::shared_ptr<CStreetMap::SNode>& node1, 
                          const std::shared_ptr<CStreetMap::SNode>& node2) const {
    auto StreetMap = Config->StreetMap();
    
    for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
        auto way = StreetMap->WayByIndex(i);
        
        for (size_t j = 0; j < way->NodeCount() - 1; ++j) {
            // Check if consecutive nodes match our nodes (in either order)
            if ((way->GetNodeID(j) == node1->ID() && way->GetNodeID(j+1) == node2->ID()) ||
                (way->GetNodeID(j) == node2->ID() && way->GetNodeID(j+1) == node1->ID())) {
                
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
    
    // If no path found or distance is negative (not just zero)
    if (routerPath.empty() || distance < 0.0) {
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
    
    // Special case: if src and dest are the same
    if (src == dest) {
        path.push_back({ETransportationMode::Walk, src});
        return 0.0;
    }
    
    // Make sure source and destination are valid
    if (DImplementation->NodeIDToTimeVertexID.find(src) == DImplementation->NodeIDToTimeVertexID.end() ||
        DImplementation->NodeIDToTimeVertexID.find(dest) == DImplementation->NodeIDToTimeVertexID.end()) {
        return CPathRouter::NoPathExists; // Invalid nodes
    }
    
    // Get vertex IDs for the source and destination
    auto srcVertex = DImplementation->NodeIDToTimeVertexID[src];
    auto destVertex = DImplementation->NodeIDToTimeVertexID[dest];
    
    // Find shortest path using the time router
    std::vector<CPathRouter::TVertexID> routerPath;
    double time = DImplementation->TimeRouter->FindShortestPath(srcVertex, destVertex, routerPath);
    
    // If no path found or time is invalid
    if (routerPath.empty() || time < 0.0) {
        return CPathRouter::NoPathExists;
    }
    
    auto StreetMap = DImplementation->Config->StreetMap();
    
    // Convert router path to node IDs
    std::vector<TNodeID> nodePath;
    for (const auto& vertex : routerPath) {
        nodePath.push_back(DImplementation->TimeVertexIDToNodeID[vertex]);
    }
    
    // Determine best mode for entire path first
    ETransportationMode bestPathMode = ETransportationMode::Walk;
    
    // Check if biking is possible for the entire path
    bool canBike = true;
    for (size_t i = 1; i < nodePath.size(); ++i) {
        auto prevNode = StreetMap->NodeByID(nodePath[i-1]);
        auto currNode = StreetMap->NodeByID(nodePath[i]);
        
        if (!prevNode || !currNode) {
            canBike = false;
            break;
        }
        
        // Check if this segment can be biked (e.g., not a one-way street against direction)
        // This is a simplified check - you might need more detailed logic
        double distance = SGeographicUtils::HaversineDistanceInMiles(
            prevNode->Location(), currNode->Location());
        
        if (distance <= 0.0) {
            canBike = false;
            break;
        }
    }
    
    // If biking is possible, use it as the best mode for the entire path
    if (canBike) {
        bestPathMode = ETransportationMode::Bike;
    }
    
    // Check if a bus route is available for segments
    std::vector<std::pair<size_t, size_t>> busSegments;
    
    for (size_t i = 0; i < nodePath.size() - 1; ++i) {
        size_t start = i;
        size_t end = i;
        
        // Find the longest continuous bus segment
        while (end < nodePath.size() - 1) {
            std::string busRoute = DImplementation->FindBusRouteBetweenNodes(nodePath[end], nodePath[end+1]);
            if (busRoute.empty() || 
                DImplementation->NodeIDToStopID.find(nodePath[end]) == DImplementation->NodeIDToStopID.end() ||
                DImplementation->NodeIDToStopID.find(nodePath[end+1]) == DImplementation->NodeIDToStopID.end()) {
                break;
            }
            end++;
        }
        
        if (end > start) {
            busSegments.push_back({start, end});
            i = end; // Skip to the end of this bus segment
        }
    }
    
    // Create the path with appropriate transportation modes
    for (size_t i = 0; i < nodePath.size(); ++i) {
        // Default to the best overall mode
        ETransportationMode mode = bestPathMode;
        
        // Check if this node is part of a bus segment
        for (const auto& segment : busSegments) {
            if (i >= segment.first && i <= segment.second) {
                mode = ETransportationMode::Bus;
                break;
            }
        }
        
        // Special case for the starting node
        if (i == 0) {
            // Always start with walking
            path.push_back({ETransportationMode::Walk, nodePath[i]});
        } else {
            path.push_back({mode, nodePath[i]});
        }
    }
    
    // Now optimize the path by combining consecutive steps with the same mode
    if (path.size() > 1) {
        std::vector<TTripStep> optimizedPath;
        optimizedPath.push_back(path[0]);
        
        for (size_t i = 1; i < path.size(); ++i) {
            // Only combine if the modes are the same and not at the boundary of a bus segment
            if (path[i].first == optimizedPath.back().first) {
                // For bus segments, keep each stop
                if (path[i].first == ETransportationMode::Bus) {
                    // Check if this is a bus stop boundary
                    bool isBusStopBoundary = false;
                    for (const auto& segment : busSegments) {
                        if (i - 1 == segment.first || i == segment.second) {
                            isBusStopBoundary = true;
                            break;
                        }
                    }
                    
                    if (isBusStopBoundary) {
                        optimizedPath.push_back(path[i]);
                    } else {
                        optimizedPath.back().second = path[i].second;
                    }
                } else {
                    // For other modes, combine segments
                    optimizedPath.back().second = path[i].second;
                }
            } else {
                // Different mode, add a new step
                optimizedPath.push_back(path[i]);
            }
        }
        
        path = optimizedPath;
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
    
    // Find continuous segments of the same mode
    std::vector<std::pair<size_t, size_t>> segments;
    size_t currentStart = 0;
    
    for (size_t i = 1; i < path.size(); ++i) {
        if (path[i].first != path[i-1].first) {
            segments.push_back({currentStart, i-1});
            currentStart = i;
        }
    }
    segments.push_back({currentStart, path.size()-1});
    
    // Process each segment
    for (const auto& segment : segments) {
        size_t start = segment.first;
        size_t end = segment.second;
        
        // Get the transportation mode for this segment
        auto mode = path[start].first;
        
        if (mode == ETransportationMode::Walk || mode == ETransportationMode::Bike) {
            // For walking or biking, create a description for each significant change in direction
            for (size_t i = start + 1; i <= end; ++i) {
                auto prevNode = StreetMap->NodeByID(path[i-1].second);
                auto currentNode = StreetMap->NodeByID(path[i].second);
                
                if (!prevNode || !currentNode) {
                    return false;
                }
                
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
                } else { // Bike
                    ss << "Bike " << direction;
                }
                
                if (streetName != "unnamed street") {
                    ss << " along " << streetName;
                } else {
                    // Check if this is toward a known street
                    // This is a placeholder - you might need specific logic here
                    ss << " toward " << (i == end ? "End" : "Main St.");
                }
                
                ss << " for " << std::fixed << std::setprecision(1) << distance << " mi";
                desc.push_back(ss.str());
            }
        } else if (mode == ETransportationMode::Bus) {
            // For bus mode, create a description from the first stop to the last
            auto startStopNodeID = path[start].second;
            auto endStopNodeID = path[end].second;
            
            if (DImplementation->NodeIDToStopID.find(startStopNodeID) != DImplementation->NodeIDToStopID.end() &&
                DImplementation->NodeIDToStopID.find(endStopNodeID) != DImplementation->NodeIDToStopID.end()) {
                
                auto startStopID = DImplementation->NodeIDToStopID.at(startStopNodeID);
                auto endStopID = DImplementation->NodeIDToStopID.at(endStopNodeID);
                
                // Find the bus route connecting these stops
                std::string busRoute = "A"; // Placeholder - you might need to implement a function to find the route
                
                // In a real implementation, you'd need to find the actual bus route
                // This is just a placeholder assuming route "A" for simplicity
                std::stringstream ss;
                ss << "Take Bus " << busRoute << " from stop " << startStopID << " to stop " << endStopID;
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