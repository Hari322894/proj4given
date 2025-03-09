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
    using TLocation = std::pair<double, double>;
    using TTransportationMode = CTransportationPlanner::ETransportationMode;
    
    // Define TRouteID if it's not defined in CBusSystem
    using TRouteID = std::size_t; // Adjust this type as needed
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
    
    SImplementation(std::shared_ptr<CTransportationPlanner::SConfiguration> config) {
        StreetMap = config->StreetMap;
        BusSystem = config->BusSystem;
        WalkSpeed = config->WalkSpeed;
        BikeSpeed = config->BikeSpeed;
        BusSpeed = config->BusSpeed;
        
        ShortestPathRouter = std::make_unique<CDijkstraPathRouter>();
        FastestPathRouter = std::make_unique<CDijkstraPathRouter>();
        
        // Add nodes/vertices to path routers
        for(std::size_t i = 0; i < StreetMap->NodeCount(); i++){
            auto Node = StreetMap->NodeByIndex(i);
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
                ShortestPathRouter->AddEdge(src, dest, Distance, CTransportationPlanner::ETransportationMode::Walking);
                ShortestPathRouter->AddEdge(dest, src, Distance, CTransportationPlanner::ETransportationMode::Walking);
                
                // Add to fastest path router with time as weight
                double WalkTime = Distance / WalkSpeed;
                double BikeTime = Distance / BikeSpeed;
                
                FastestPathRouter->AddEdge(src, dest, WalkTime, CTransportationPlanner::ETransportationMode::Walking);
                FastestPathRouter->AddEdge(dest, src, WalkTime, CTransportationPlanner::ETransportationMode::Walking);
                
                if(Way->GetAttribute("bicycle") != "no" && Way->GetAttribute("highway") != "motorway"){
                    FastestPathRouter->AddEdge(src, dest, BikeTime, CTransportationPlanner::ETransportationMode::Biking);
                    FastestPathRouter->AddEdge(dest, src, BikeTime, CTransportationPlanner::ETransportationMode::Biking);
                    
                    ShortestPathRouter->AddEdge(src, dest, Distance, CTransportationPlanner::ETransportationMode::Biking);
                    ShortestPathRouter->AddEdge(dest, src, Distance, CTransportationPlanner::ETransportationMode::Biking);
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
                    
                    if(BusTime > 0){
                        FastestPathRouter->AddEdge(prevStop, currStop, BusTime, CTransportationPlanner::ETransportationMode::Bus);
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

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    // Convert vector<TNodeID> to vector<int> for PathRouter if needed
    std::vector<int> pathAsInt;
    
    double distance = DImplementation->ShortestPathRouter->FindShortestPath(src, dest, pathAsInt);
    
    // Convert back if needed
    path.clear();
    for(auto node : pathAsInt) {
        path.push_back(static_cast<TNodeID>(node));
    }
    
    return distance;
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<std::pair<ETransportationMode, TNodeID>> &path) {
    std::vector<TNodeID> nodePath;
    std::vector<int> edgeLabels;
    
    // Convert vectors if needed
    std::vector<int> nodePathInt;
    std::vector<int> edgeLabelsInt;
    
    double time = DImplementation->FastestPathRouter->FindShortestPath(src, dest, nodePathInt, edgeLabelsInt);
    
    // Convert back to TNodeID
    nodePath.clear();
    for(auto node : nodePathInt) {
        nodePath.push_back(static_cast<TNodeID>(node));
    }
    
    // Convert to path with transportation modes
    path.clear();
    for(size_t i = 0; i < nodePath.size(); i++){
        if(i < edgeLabelsInt.size()){ // For all except the last node
            path.push_back({static_cast<ETransportationMode>(edgeLabelsInt[i]), nodePath[i]});
        }
        else {
            // For the last node, use the last mode or Walking as default
            ETransportationMode lastMode = ETransportationMode::Walking;
            if(!edgeLabelsInt.empty()){
                lastMode = static_cast<ETransportationMode>(edgeLabelsInt.back());
            }
            path.push_back({lastMode, nodePath[i]});
        }
    }
    
    return time;
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<std::pair<ETransportationMode, TNodeID>> &path, std::vector<std::string> &description) const {
    if(path.empty()){
        return false;
    }
    
    description.clear();
    SImplementation::TRouteID currentBusRoute = SImplementation::InvalidRouteID;
    ETransportationMode currentMode = ETransportationMode::Walking;
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
            if(currentMode == ETransportationMode::Walking){
                if(totalDistance > 0.0){
                    ss << "Walk " << std::fixed << std::setprecision(1) << totalDistance << " miles";
                    description.push_back(ss.str());
                    ss.str("");
                }
            }
            else if(currentMode == ETransportationMode::Biking){
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
        if(currentMode == ETransportationMode::Walking || currentMode == ETransportationMode::Biking){
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
            if(currentMode == ETransportationMode::Walking){
                ss << "Walk " << std::fixed << std::setprecision(1) << totalDistance << " miles";
                description.push_back(ss.str());
            }
            else if(currentMode == ETransportationMode::Biking){
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