// First, verify these include files match your project structure
#include "DijkstraTransportationPlanner.h"
#include "TransportationPlanner.h"
#include "PathRouter.h"
#include "DijkstraPathRouter.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

struct CDijkstraTransportationPlanner::SImplementation {
    using TNodeID = CTransportationPlanner::TNodeID;
    using TTransportationMode = CTransportationPlanner::ETransportationMode;
    using TTripStep = CTransportationPlanner::TTripStep;
    
    // Define TRouteID if it's not defined in CBusSystem
    using TRouteID = std::size_t;
    static const TRouteID InvalidRouteID = std::numeric_limits<TRouteID>::max();
    
    // Use unordered_map for BusRoutes
    std::unordered_map<TNodeID, std::vector<std::pair<TNodeID, TRouteID>>> BusRoutes;
    
    std::shared_ptr<CStreetMap> StreetMap;
    std::shared_ptr<CBusSystem> BusSystem;
    std::unique_ptr<CPathRouter> ShortestPathRouter;
    std::unique_ptr<CPathRouter> FastestPathRouter;
    double WalkSpeed;
    double BikeSpeed;
    double BusSpeed;
    double BusStopTime;
    
    // Store sorted nodes for NodeCount() and SortedNodeByIndex()
    std::vector<std::shared_ptr<CStreetMap::SNode>> SortedNodes;
    
    SImplementation(std::shared_ptr<CTransportationPlanner::SConfiguration> config) {
        StreetMap = config->StreetMap();
        BusSystem = config->BusSystem();
        WalkSpeed = config->WalkSpeed();
        BikeSpeed = config->BikeSpeed();
        BusSpeed = config->DefaultSpeedLimit();
        BusStopTime = config->BusStopTime();
        
        ShortestPathRouter = std::make_unique<CDijkstraPathRouter>();
        FastestPathRouter = std::make_unique<CDijkstraPathRouter>();
        
        // Prepare sorted nodes for SortedNodeByIndex
        for(std::size_t i = 0; i < StreetMap->NodeCount(); i++){
            SortedNodes.push_back(StreetMap->NodeByIndex(i));
        }
        
        // Sort nodes by ID
        std::sort(SortedNodes.begin(), SortedNodes.end(), 
                  [](const std::shared_ptr<CStreetMap::SNode>& a, const std::shared_ptr<CStreetMap::SNode>& b) {
                      return a->ID() < b->ID();
                  });
        
        // Add nodes/vertices to path routers
        for(auto& Node : SortedNodes){
            auto NodeID = Node->ID();
            ShortestPathRouter->AddVertex(NodeID);
            FastestPathRouter->AddVertex(NodeID);
        }
        
        // Add edges for ways/streets
        for(std::size_t i = 0; i < StreetMap->WayCount(); i++){
            auto Way = StreetMap->WayByIndex(i);
            for(std::size_t j = 1; j < Way->NodeCount(); j++){
                TNodeID src = Way->GetNodeID(j-1);
                TNodeID dest = Way->GetNodeID(j);
                
                // Skip if nodes aren't in our map (unlikely)
                auto SrcNode = StreetMap->NodeByID(src);
                auto DestNode = StreetMap->NodeByID(dest);
                if(!SrcNode || !DestNode){
                    continue;
                }
                
                // Calculate distance using Haversine formula or Euclidean
                double LatDiff = SrcNode->GetLatitude() - DestNode->GetLatitude();
                double LonDiff = SrcNode->GetLongitude() - DestNode->GetLongitude();
                double Distance = std::sqrt(LatDiff * LatDiff + LonDiff * LonDiff);
                
                // Add to shortest path router with distance as weight
                ShortestPathRouter->AddEdge(src, dest, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                ShortestPathRouter->AddEdge(dest, src, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                
                // Add to fastest path router with time as weight
                double WalkTime = Distance / WalkSpeed;
                double BikeTime = Distance / BikeSpeed;
                
                FastestPathRouter->AddEdge(src, dest, WalkTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                FastestPathRouter->AddEdge(dest, src, WalkTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                
                if(Way->GetAttribute("bicycle") != "no" && Way->GetAttribute("highway") != "motorway"){
                    FastestPathRouter->AddEdge(src, dest, BikeTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                    FastestPathRouter->AddEdge(dest, src, BikeTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                    
                    ShortestPathRouter->AddEdge(src, dest, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                    ShortestPathRouter->AddEdge(dest, src, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                }
            }
        }
        
        // Add bus routes
        if(BusSystem){
            // Initialize BusRoutes
            for(std::size_t routeIndex = 0; routeIndex < BusSystem->RouteCount(); routeIndex++){
                auto RouteID = routeIndex; // Use index as RouteID if missing GetRouteID method
                
                for(std::size_t stopIndex = 1; stopIndex < BusSystem->StopCount(RouteID); stopIndex++){
                    TNodeID prevStop = BusSystem->StopID(RouteID, stopIndex - 1);
                    TNodeID currStop = BusSystem->StopID(RouteID, stopIndex);
                    
                    // Record the bus route between these stops
                    BusRoutes[prevStop].push_back({currStop, RouteID});
                    
                    // Calculate bus travel time between stops
                    double BusTime = 0;
                    if(BusSystem->IncludesTimeData()){
                        BusTime = BusSystem->StopTime(RouteID, stopIndex) - 
                                 BusSystem->StopTime(RouteID, stopIndex - 1);
                    }
                    else {
                        // If no time data, estimate based on distance and bus speed
                        auto SrcNode = StreetMap->NodeByID(prevStop);
                        auto DestNode = StreetMap->NodeByID(currStop);
                        
                        if(SrcNode && DestNode){
                            double LatDiff = SrcNode->GetLatitude() - DestNode->GetLatitude();
                            double LonDiff = SrcNode->GetLongitude() - DestNode->GetLongitude();
                            double Distance = std::sqrt(LatDiff * LatDiff + LonDiff * LonDiff);
                            BusTime = Distance / BusSpeed;
                        }
                    }
                    
                    // Add bus stop time
                    BusTime += BusStopTime;
                    
                    if(BusTime > 0){
                        FastestPathRouter->AddEdge(prevStop, currStop, BusTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Bus));
                    }
                }
            }
        }
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
}

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->SortedNodes.size();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    if(index < DImplementation->SortedNodes.size()) {
        return DImplementation->SortedNodes[index];
    }
    return nullptr;
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    path.clear();
    
    std::vector<CPathRouter::TVertexID> pathVertices;
    double distance = DImplementation->ShortestPathRouter->FindShortestPath(src, dest, pathVertices);
    
    if(distance != std::numeric_limits<double>::max()) {
        for(auto vertex : pathVertices) {
            path.push_back(static_cast<TNodeID>(vertex));
        }
    }
    
    return distance;
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    path.clear();
    
    std::vector<CPathRouter::TVertexID> pathVertices;
    std::vector<int> edgeLabels;
    
    double time = DImplementation->FastestPathRouter->FindShortestPath(src, dest, pathVertices, edgeLabels);
    
    if(time != std::numeric_limits<double>::max() && !pathVertices.empty()) {
        // Add the first node with default mode (or could be determined by first edge)
        ETransportationMode firstMode = ETransportationMode::Walk;
        if(!edgeLabels.empty()) {
            firstMode = static_cast<ETransportationMode>(edgeLabels[0]);
        }
        path.push_back({firstMode, pathVertices[0]});
        
        // Add remaining nodes with their transportation modes
        for(size_t i = 1; i < pathVertices.size(); i++) {
            ETransportationMode mode = static_cast<ETransportationMode>(edgeLabels[i-1]);
            path.push_back({mode, pathVertices[i]});
        }
    }
    
    return time;
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &description) const {
    if(path.empty()){
        return false;
    }
    
    description.clear();
    SImplementation::TRouteID currentBusRoute = SImplementation::InvalidRouteID;
    ETransportationMode currentMode = ETransportationMode::Walk;
    std::ostringstream ss;
    double totalDistance = 0.0;
    
    auto BusSystemObj = DImplementation->BusSystem;
    auto StreetMapObj = DImplementation->StreetMap;
    
    for(size_t i = 0; i < path.size(); i++){
        auto currentNodeID = path[i].second;
        auto currentTransportMode = path[i].first;
        
        if(i == 0){
            // Starting point
            auto startNode = StreetMapObj->NodeByID(currentNodeID);
            if(startNode){
                ss << "Start at (" << std::fixed << std::setprecision(6) 
                   << startNode->GetLatitude() << ", " << startNode->GetLongitude() << ")";
                description.push_back(ss.str());
                ss.str("");
            }
            currentMode = currentTransportMode;
            continue;
        }
        
        auto prevNodeID = path[i-1].second;
        
        // Mode change or segment description
        if(currentTransportMode != currentMode){
            // Close out previous segment
            if(currentMode == ETransportationMode::Walk){
                if(totalDistance > 0.0){
                    ss << "Walk " << std::fixed << std::setprecision(1) << totalDistance << " miles";
                    description.push_back(ss.str());
                    ss.str("");
                }
            }
            else if(currentMode == ETransportationMode::Bike){
                if(totalDistance > 0.0){
                    ss << "Bike " << std::fixed << std::setprecision(1) << totalDistance << " miles";
                    description.push_back(ss.str());
                    ss.str("");
                }
            }
            else if(currentMode == ETransportationMode::Bus){
                // Bus route description
                if(currentBusRoute != SImplementation::InvalidRouteID){
                    std::string routeName = "Route " + std::to_string(currentBusRoute);
                    if(BusSystemObj->IncludesRouteNames()) {
                        routeName = BusSystemObj->RouteName(currentBusRoute);
                    }
                    ss << "Take bus route " << routeName;
                    description.push_back(ss.str());
                    ss.str("");
                }
            }
            
            // Start new segment
            totalDistance = 0.0;
            currentMode = currentTransportMode;
        }
        
        // Calculate segment distance
        if(currentMode == ETransportationMode::Walk || currentMode == ETransportationMode::Bike){
            auto currNode = StreetMapObj->NodeByID(currentNodeID);
            auto prevNode = StreetMapObj->NodeByID(prevNodeID);
            
            if(currNode && prevNode){
                double latDiff = prevNode->GetLatitude() - currNode->GetLatitude();
                double lonDiff = prevNode->GetLongitude() - currNode->GetLongitude();
                double segmentDistance = std::sqrt(latDiff * latDiff + lonDiff * lonDiff);
                totalDistance += segmentDistance;
            }
        }
        else if(currentMode == ETransportationMode::Bus){
            // Find the bus route between these nodes
            bool foundBusSegment = false;
            
            // Check if there's a bus route from prevNodeID to currentNodeID
            if(DImplementation->BusRoutes.count(prevNodeID) > 0) {
                for(const auto &route : DImplementation->BusRoutes.at(prevNodeID)) {
                    if(route.first == currentNodeID) {
                        SImplementation::TRouteID nextBusRoute = route.second;
                        foundBusSegment = true;
                        
                        if(nextBusRoute != currentBusRoute) {
                            // Bus route change
                            if(currentBusRoute != SImplementation::InvalidRouteID) {
                                std::string routeName = "Route " + std::to_string(currentBusRoute);
                                if(BusSystemObj->IncludesRouteNames()) {
                                    routeName = BusSystemObj->RouteName(currentBusRoute);
                                }
                                ss << "Take bus route " << routeName;
                                description.push_back(ss.str());
                                ss.str("");
                            }
                            currentBusRoute = nextBusRoute;
                        }
                        break;
                    }
                }
            }
        }
        
        // Last node in path
        if(i == path.size() - 1){
            // Close out the last segment
            if(currentMode == ETransportationMode::Walk){
                ss << "Walk " << std::fixed << std::setprecision(1) << totalDistance << " miles";
                description.push_back(ss.str());
            }
            else if(currentMode == ETransportationMode::Bike){
                ss << "Bike " << std::fixed << std::setprecision(1) << totalDistance << " miles";
                description.push_back(ss.str());
            }
            else if(currentMode == ETransportationMode::Bus){
                if(currentBusRoute != SImplementation::InvalidRouteID){
                    std::string routeName = "Route " + std::to_string(currentBusRoute);
                    if(BusSystemObj->IncludesRouteNames()) {
                        routeName = BusSystemObj->RouteName(currentBusRoute);
                    }
                    ss << "Take bus route " << routeName;
                    description.push_back(ss.str());
                }
            }
            
            // Add destination
            auto endNode = StreetMapObj->NodeByID(currentNodeID);
            if(endNode){
                ss.str("");
                ss << "End at (" << std::fixed << std::setprecision(6) 
                   << endNode->GetLatitude() << ", " << endNode->GetLongitude() << ")";
                description.push_back(ss.str());
            }
        }
    }
    
    return !description.empty();
}