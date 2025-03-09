#include "DijkstraTransportationPlanner.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>
#include <cmath>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> Config;
    std::vector<std::shared_ptr<CStreetMap::SNode>> SortedNodes;
    std::unordered_map<TNodeID, size_t> NodeIDToIndex;
    std::shared_ptr<CPathRouter> ShortestPathRouter;
    std::shared_ptr<CPathRouter> FastestPathRouter;
    std::unordered_map<TNodeID, std::vector<std::pair<TNodeID, std::string>>> StreetSegments;
    std::unordered_map<TNodeID, std::vector<std::pair<TNodeID, CBusSystem::TRouteID>>> BusRoutes;

    SImplementation(std::shared_ptr<SConfiguration> config) 
        : Config(config),
          ShortestPathRouter(CPathRouter::CreatePathRouter()), 
          FastestPathRouter(CPathRouter::CreatePathRouter()) {
        
        // Build sorted nodes and mapping
        auto StreetMapObj = Config->StreetMap();
        for (size_t i = 0; i < StreetMapObj->NodeCount(); ++i) {
            auto Node = StreetMapObj->GetNodeByIndex(i);
            if (Node) {
                SortedNodes.push_back(Node);
            }
        }
        std::sort(SortedNodes.begin(), SortedNodes.end(), 
            [](const std::shared_ptr<CStreetMap::SNode>& lhs, const std::shared_ptr<CStreetMap::SNode>& rhs) {
                return lhs->ID() < rhs->ID();
            });
        
        // Build ID to index mapping
        for (size_t i = 0; i < SortedNodes.size(); ++i) {
            NodeIDToIndex[SortedNodes[i]->ID()] = i;
        }

        // Add nodes to routers
        for (auto& Node : SortedNodes) {
            ShortestPathRouter->AddNode(Node->ID());
            FastestPathRouter->AddNode(Node->ID());
        }

        // Process street segments for routing
        for (size_t i = 0; i < StreetMapObj->WayCount(); ++i) {
            auto Way = StreetMapObj->GetWayByIndex(i);
            if (!Way) continue;

            // Check if the way is bidirectional for bikes and pedestrians
            bool IsBidirectional = true;
            std::string WayName = Way->GetAttribute("name").empty() ? "unnamed street" : Way->GetAttribute("name");

            for (size_t j = 1; j < Way->NodeCount(); ++j) {
                TNodeID src = Way->GetNodeID(j-1);
                TNodeID dest = Way->GetNodeID(j);
                
                // Get source and destination nodes
                auto SrcNode = StreetMapObj->GetNodeByID(src);
                auto DestNode = StreetMapObj->GetNodeByID(dest);
                if (!SrcNode || !DestNode) continue;

                // Calculate distance
                double LatDiff = SrcNode->Latitude() - DestNode->Latitude();
                double LonDiff = SrcNode->Longitude() - DestNode->Longitude();
                double Distance = std::sqrt(LatDiff * LatDiff + LonDiff * LonDiff);

                // Store street segment info for path description
                StreetSegments[src].push_back({dest, WayName});
                if (IsBidirectional) {
                    StreetSegments[dest].push_back({src, WayName});
                }

                // Add edges to shortest path router (distance-based)
                ShortestPathRouter->AddEdge(src, dest, Distance);
                if (IsBidirectional) {
                    ShortestPathRouter->AddEdge(dest, src, Distance);
                }

                // Get speed limit or use default
                double SpeedLimit = Config->DefaultSpeedLimit();
                std::string SpeedLimitStr = Way->GetAttribute("maxspeed");
                if (!SpeedLimitStr.empty()) {
                    try {
                        SpeedLimit = std::stod(SpeedLimitStr);
                    } catch (...) {
                        // Use default if parsing fails
                    }
                }

                // Calculate travel times for different modes
                double WalkTime = Distance / Config->WalkSpeed();
                double BikeTime = Distance / std::min(Config->BikeSpeed(), SpeedLimit);

                // Add edges to fastest path router (time-based)
                // Walking edges
                FastestPathRouter->AddEdge(src, dest, WalkTime, static_cast<CPathRouter::TEdgeLabel>(ETransportationMode::Walk));
                if (IsBidirectional) {
                    FastestPathRouter->AddEdge(dest, src, WalkTime, static_cast<CPathRouter::TEdgeLabel>(ETransportationMode::Walk));
                }

                // Biking edges
                FastestPathRouter->AddEdge(src, dest, BikeTime, static_cast<CPathRouter::TEdgeLabel>(ETransportationMode::Bike));
                if (IsBidirectional) {
                    FastestPathRouter->AddEdge(dest, src, BikeTime, static_cast<CPathRouter::TEdgeLabel>(ETransportationMode::Bike));
                }
            }
        }

        // Process bus routes
        auto BusSystemObj = Config->BusSystem();
        for (size_t routeIndex = 0; routeIndex < BusSystemObj->RouteCount(); ++routeIndex) {
            auto RouteID = BusSystemObj->GetRouteID(routeIndex);
            
            for (size_t stopIndex = 1; stopIndex < BusSystemObj->StopCount(RouteID); ++stopIndex) {
                TNodeID prevStop = BusSystemObj->GetStopID(RouteID, stopIndex - 1);
                TNodeID currStop = BusSystemObj->GetStopID(RouteID, stopIndex);
                
                // Store bus route information for path description
                BusRoutes[prevStop].push_back({currStop, RouteID});
                
                // Calculate bus travel time between stops
                double BusTime = BusSystemObj->StopTime(RouteID, stopIndex) - 
                                BusSystemObj->StopTime(RouteID, stopIndex - 1);
                
                // Add bus edges to fastest path router
                FastestPathRouter->AddEdge(prevStop, currStop, BusTime + Config->BusStopTime(), 
                                        static_cast<CPathRouter::TEdgeLabel>(ETransportationMode::Bus));
            }
        }

        // Precompute paths if specified
        if (Config->PrecomputeTime() > 0) {
            ShortestPathRouter->Precompute(Config->PrecomputeTime());
            FastestPathRouter->Precompute(Config->PrecomputeTime());
        }
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config)
    : DImplementation(std::make_unique<SImplementation>(config)) {
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
}

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
    path.clear();
    
    // Check if source and destination nodes exist
    if (DImplementation->NodeIDToIndex.find(src) == DImplementation->NodeIDToIndex.end() ||
        DImplementation->NodeIDToIndex.find(dest) == DImplementation->NodeIDToIndex.end()) {
        return std::numeric_limits<double>::max();
    }
    
    // Use PathRouter to find the shortest path
    double distance = DImplementation->ShortestPathRouter->FindShortestPath(src, dest, path);
    
    return distance;
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    path.clear();
    
    // Check if source and destination nodes exist
    if (DImplementation->NodeIDToIndex.find(src) == DImplementation->NodeIDToIndex.end() ||
        DImplementation->NodeIDToIndex.find(dest) == DImplementation->NodeIDToIndex.end()) {
        return std::numeric_limits<double>::max();
    }
    
    // Use PathRouter to find the fastest path
    std::vector<TNodeID> nodePath;
    std::vector<CPathRouter::TEdgeLabel> edgeLabels;
    
    double time = DImplementation->FastestPathRouter->FindShortestPath(src, dest, nodePath, edgeLabels);
    
    if (std::numeric_limits<double>::max() == time) {
        return time;
    }
    
    // Convert the node path and edge labels to trip steps
    path.push_back({ETransportationMode::Walk, nodePath[0]}); // Start with walking at the first node
    
    for (size_t i = 1; i < nodePath.size(); ++i) {
        ETransportationMode mode = static_cast<ETransportationMode>(edgeLabels[i-1]);
        path.push_back({mode, nodePath[i]});
    }
    
    return time;
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    desc.clear();
    
    if (path.empty()) {
        return false;
    }
    
    // Track current transportation mode and related info
    ETransportationMode currentMode = path[0].first;
    TNodeID currentNodeID = path[0].second;
    std::string currentStreet;
    CBusSystem::TRouteID currentBusRoute;
    double currentDistance = 0.0;
    
    auto StreetMapObj = DImplementation->Config->StreetMap();
    auto BusSystemObj = DImplementation->Config->BusSystem();
    
    for (size_t i = 1; i < path.size(); ++i) {
        ETransportationMode nextMode = path[i].first;
        TNodeID nextNodeID = path[i].second;
        
        // Handle mode transition
        if (nextMode != currentMode || i == path.size() - 1) {
            // Generate description for completed segment
            std::stringstream ss;
            
            // Handle the last node if we're at the end
            if (i == path.size() - 1 && nextMode == currentMode) {
                // Process the final segment with the same transportation mode
                if (currentMode == ETransportationMode::Walk || currentMode == ETransportationMode::Bike) {
                    // Check for street segment
                    std::string nextStreet;
                    for (const auto &segment : DImplementation->StreetSegments.at(currentNodeID)) {
                        if (segment.first == nextNodeID) {
                            nextStreet = segment.second;
                            break;
                        }
                    }
                    
                    if (nextStreet != currentStreet) {
                        // Finish previous street description if we had one
                        if (!currentStreet.empty()) {
                            if (currentMode == ETransportationMode::Walk) {
                                ss << "Walk " << std::fixed << std::setprecision(1) << currentDistance;
                                ss << " miles along " << currentStreet;
                                desc.push_back(ss.str());
                                ss.str("");
                            }
                            else { // Bike
                                ss << "Bike " << std::fixed << std::setprecision(1) << currentDistance;
                                ss << " miles along " << currentStreet;
                                desc.push_back(ss.str());
                                ss.str("");
                            }
                        }
                        
                        // Start new street segment
                        currentStreet = nextStreet;
                        
                        // Calculate distance for last segment
                        auto currNode = StreetMapObj->GetNodeByID(currentNodeID);
                        auto nextNode = StreetMapObj->GetNodeByID(nextNodeID);
                        if (currNode && nextNode) {
                            double latDiff = currNode->Latitude() - nextNode->Latitude();
                            double lonDiff = currNode->Longitude() - nextNode->Longitude();
                            currentDistance = std::sqrt(latDiff * latDiff + lonDiff * lonDiff);
                        }
                    }
                    else {
                        // Continue on same street, accumulate distance
                        auto currNode = StreetMapObj->GetNodeByID(currentNodeID);
                        auto nextNode = StreetMapObj->GetNodeByID(nextNodeID);
                        if (currNode && nextNode) {
                            double latDiff = currNode->Latitude() - nextNode->Latitude();
                            double lonDiff = currNode->Longitude() - nextNode->Longitude();
                            currentDistance += std::sqrt(latDiff * latDiff + lonDiff * lonDiff);
                        }
                    }
                }
                else if (currentMode == ETransportationMode::Bus) {
                    // Check for bus segment
                    CBusSystem::TRouteID nextBusRoute;
                    bool foundBusSegment = false;
                    
                    for (const auto &route : DImplementation->BusRoutes.at(currentNodeID)) {
                        if (route.first == nextNodeID) {
                            nextBusRoute = route.second;
                            foundBusSegment = true;
                            break;
                        }
                    }
                    
                    if (foundBusSegment && nextBusRoute != currentBusRoute) {
                        // Finish previous bus route description if we had one
                        if (currentBusRoute != CBusSystem::InvalidRouteID) {
                            ss << "Take bus route " << BusSystemObj->GetRouteName(currentBusRoute);
                            desc.push_back(ss.str());
                            ss.str("");
                        }
                        
                        // Start new bus route
                        currentBusRoute = nextBusRoute;
                    }
                }
            }
            
            // Generate final description based on current mode
            if (currentMode == ETransportationMode::Walk) {
                if (!currentStreet.empty()) {
                    ss << "Walk " << std::fixed << std::setprecision(1) << currentDistance;
                    ss << " miles along " << currentStreet;
                    desc.push_back(ss.str());
                }
            }
            else if (currentMode == ETransportationMode::Bike) {
                if (!currentStreet.empty()) {
                    ss << "Bike " << std::fixed << std::setprecision(1) << currentDistance;
                    ss << " miles along " << currentStreet;
                    desc.push_back(ss.str());
                }
            }
            else if (currentMode == ETransportationMode::Bus) {
                if (currentBusRoute != CBusSystem::InvalidRouteID) {
                    ss << "Take bus route " << BusSystemObj->GetRouteName(currentBusRoute);
                    desc.push_back(ss.str());
                }
            }
            
            // Reset for next segment
            currentMode = nextMode;
            currentStreet = "";
            currentBusRoute = CBusSystem::InvalidRouteID;
            currentDistance = 0.0;
        }
        
        // Process current segment
        if (currentMode == ETransportationMode::Walk || currentMode == ETransportationMode::Bike) {
            // Look for street segment
            std::string nextStreet;
            for (const auto &segment : DImplementation->StreetSegments.at(currentNodeID)) {
                if (segment.first == nextNodeID) {
                    nextStreet = segment.second;
                    break;
                }
            }
            
            if (nextStreet != currentStreet) {
                // Finish previous street description if we had one
                if (!currentStreet.empty()) {
                    std::stringstream ss;
                    if (currentMode == ETransportationMode::Walk) {
                        ss << "Walk " << std::fixed << std::setprecision(1) << currentDistance;
                        ss << " miles along " << currentStreet;
                        desc.push_back(ss.str());
                    }
                    else { // Bike
                        ss << "Bike " << std::fixed << std::setprecision(1) << currentDistance;
                        ss << " miles along " << currentStreet;
                        desc.push_back(ss.str());
                    }
                }
                
                // Start new street segment
                currentStreet = nextStreet;
                currentDistance = 0.0;
            }
            
            // Calculate segment distance
            auto currNode = StreetMapObj->GetNodeByID(currentNodeID);
            auto nextNode = StreetMapObj->GetNodeByID(nextNodeID);
            if (currNode && nextNode) {
                double latDiff = currNode->Latitude() - nextNode->Latitude();
                double lonDiff = currNode->Longitude() - nextNode->Longitude();
                currentDistance += std::sqrt(latDiff * latDiff + lonDiff * lonDiff);
            }
        }
        else if (currentMode == ETransportationMode::Bus) {
            // Look for bus segment
            for (const auto &route : DImplementation->BusRoutes.at(currentNodeID)) {
                if (route.first == nextNodeID) {
                    CBusSystem::TRouteID nextBusRoute = route.second;
                    
                    if (nextBusRoute != currentBusRoute) {
                        // Finish previous bus route description if we had one
                        if (currentBusRoute != CBusSystem::InvalidRouteID) {
                            std::stringstream ss;
                            ss << "Take bus route " << BusSystemObj->GetRouteName(currentBusRoute);
                            desc.push_back(ss.str());
                        }
                        
                        // Start new bus route
                        currentBusRoute = nextBusRoute;
                    }
                    break;
                }
            }
        }
        
        // Move to next node
        currentNodeID = nextNodeID;
    }
    
    return !desc.empty();
}