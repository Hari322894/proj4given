// First, verify these include files match your project structure
#include "DijkstraTransportationPlanner.h"
#include "TransportationPlanner.h"
#include "PathRouter.h"
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
    // Changed from unique_ptr to shared_ptr to match what CreatePathRouter() returns
    std::shared_ptr<CPathRouter> ShortestPathRouter;
    std::shared_ptr<CPathRouter> FastestPathRouter;
    double WalkSpeed;
    double BikeSpeed;
    double BusSpeed;
    double BusStopTime;
    
    // Store sorted nodes for NodeCount() and SortedNodeByIndex()
    std::vector<std::shared_ptr<CStreetMap::SNode>> SortedNodes;
    
    // Add a cache for commonly used paths
    std::unordered_map<TNodeID, std::unordered_map<TNodeID, double>> ShortestPathCache;
    std::unordered_map<TNodeID, std::unordered_map<TNodeID, std::vector<TNodeID>>> ShortestPathNodesCache;
    std::unordered_map<TNodeID, std::unordered_map<TNodeID, double>> FastestPathCache;
    std::unordered_map<TNodeID, std::unordered_map<TNodeID, std::vector<TTripStep>>> FastestPathStepsCache;
    
    SImplementation(std::shared_ptr<CTransportationPlanner::SConfiguration> config) {
        StreetMap = config->StreetMap();
        BusSystem = config->BusSystem();
        WalkSpeed = config->WalkSpeed();
        BikeSpeed = config->BikeSpeed();
        BusSpeed = config->DefaultSpeedLimit();
        BusStopTime = config->BusStopTime();
        
        // Fixed: Simply assign the shared_ptr returned by CreatePathRouter()
        ShortestPathRouter = CPathRouter::CreatePathRouter();
        FastestPathRouter = CPathRouter::CreatePathRouter();
        
        // Performance optimization: Reserve space for nodes
        size_t nodeCount = StreetMap->NodeCount();
        SortedNodes.reserve(nodeCount);
        
        // Prepare sorted nodes for SortedNodeByIndex
        for(std::size_t i = 0; i < nodeCount; i++){
            SortedNodes.push_back(StreetMap->NodeByIndex(i));
        }
        
        // Sort nodes by ID
        std::sort(SortedNodes.begin(), SortedNodes.end(), 
                  [](const std::shared_ptr<CStreetMap::SNode>& a, const std::shared_ptr<CStreetMap::SNode>& b) {
                      return a->ID() < b->ID();
                  });
        
        // Optimization: Get max node ID for proper sizing
        TNodeID maxNodeID = 0;
        for(const auto& node : SortedNodes) {
            maxNodeID = std::max(maxNodeID, node->ID());
        }
        
        // Add nodes/vertices to path routers (optimize by doing this once)
        for(auto& Node : SortedNodes){
            auto NodeID = Node->ID();
            ShortestPathRouter->AddNode(NodeID);
            FastestPathRouter->AddNode(NodeID);
        }
        
        // Process streets and add edges
        processStreetMap();
        
        // Process bus routes
        processBusRoutes();
        
        // Precompute the paths based on the configuration
        // Limit precomputation time to avoid timeout
        int precomputeTime = std::min(config->PrecomputeTime(), 10); // Limit to 10 seconds max
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(precomputeTime);
        
        // Run precomputation with the time limit
        ShortestPathRouter->Precompute(deadline);
        FastestPathRouter->Precompute(deadline);
    }
    
    void processStreetMap() {
        // Get way count once for efficiency
        size_t wayCount = StreetMap->WayCount();
        
        // Add edges for ways/streets
        for(std::size_t i = 0; i < wayCount; i++){
            auto Way = StreetMap->WayByIndex(i);
            size_t nodeCount = Way->NodeCount();
            
            // Skip ways with fewer than 2 nodes
            if (nodeCount < 2) continue;
            
            // Check if biking is allowed
            std::string BikeAttr = "yes";
            std::string HighwayAttr = "";
            try {
                BikeAttr = Way->GetAttribute("bicycle");
            } catch(...) {
                // Attribute not found, assume biking is allowed
            }
            
            try {
                HighwayAttr = Way->GetAttribute("highway");
            } catch(...) {
                // Attribute not found
            }
            
            bool bikingAllowed = (BikeAttr != "no" && HighwayAttr != "motorway");
            
            // Process each segment in the way
            for(std::size_t j = 1; j < nodeCount; j++){
                TNodeID src = Way->GetNodeID(j-1);
                TNodeID dest = Way->GetNodeID(j);
                
                // Skip if nodes aren't in our map (unlikely)
                auto SrcNode = StreetMap->NodeByID(src);
                auto DestNode = StreetMap->NodeByID(dest);
                if(!SrcNode || !DestNode){
                    continue;
                }
                
                // Calculate distance more efficiently
                double LatDiff = SrcNode->Location().first - DestNode->Location().first;
                double LonDiff = SrcNode->Location().second - DestNode->Location().second;
                double Distance = std::sqrt(LatDiff * LatDiff + LonDiff * LonDiff);
                
                // Calculate times once
                double WalkTime = Distance / WalkSpeed;
                double BikeTime = Distance / BikeSpeed;
                
                // Add walking edges (always possible)
                ShortestPathRouter->AddEdge(src, dest, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                ShortestPathRouter->AddEdge(dest, src, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                
                FastestPathRouter->AddEdge(src, dest, WalkTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                FastestPathRouter->AddEdge(dest, src, WalkTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Walk));
                
                // Add biking edges if allowed
                if(bikingAllowed){
                    FastestPathRouter->AddEdge(src, dest, BikeTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                    FastestPathRouter->AddEdge(dest, src, BikeTime, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                    
                    ShortestPathRouter->AddEdge(src, dest, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                    ShortestPathRouter->AddEdge(dest, src, Distance, static_cast<int>(CTransportationPlanner::ETransportationMode::Bike));
                }
            }
        }
    }
    
    void processBusRoutes() {
        if(!BusSystem) return;
        
        // Process all bus routes and stops
        size_t routeCount = BusSystem->RouteCount();
        for(std::size_t routeIndex = 0; routeIndex < routeCount; routeIndex++){
            auto Route = BusSystem->RouteByIndex(routeIndex);
            if(!Route) continue;
            
            auto RouteID = routeIndex; // Use index as RouteID
            size_t stopCount = Route->StopCount();
            
            // Skip routes with fewer than 2 stops
            if (stopCount < 2) continue;
            
            for(std::size_t stopIndex = 1; stopIndex < stopCount; stopIndex++){
                CBusSystem::TStopID prevStopID = Route->GetStopID(stopIndex - 1);
                CBusSystem::TStopID currStopID = Route->GetStopID(stopIndex);
                
                auto prevStop = BusSystem->StopByID(prevStopID);
                auto currStop = BusSystem->StopByID(currStopID);
                
                if(!prevStop || !currStop) continue;
                
                TNodeID prevNodeID = prevStop->NodeID();
                TNodeID currNodeID = currStop->NodeID();
                
                // Record the bus route between these stops
                BusRoutes[prevNodeID].push_back({currNodeID, RouteID});
                
                // Calculate bus travel time between stops (estimate based on distance and bus speed)
                auto SrcNode = StreetMap->NodeByID(prevNodeID);
                auto DestNode = StreetMap->NodeByID(currNodeID);
                
                if(SrcNode && DestNode){
                    double LatDiff = SrcNode->Location().first - DestNode->Location().first;
                    double LonDiff = SrcNode->Location().second - DestNode->Location().second;
                    double Distance = std::sqrt(LatDiff * LatDiff + LonDiff * LonDiff);
                    double BusTime = Distance / BusSpeed;
                    
                    // Add bus stop time
                    BusTime += BusStopTime;
                    
                    if(BusTime > 0){
                        FastestPathRouter->AddEdge(prevNodeID, currNodeID, BusTime, 
                                                static_cast<int>(CTransportationPlanner::ETransportationMode::Bus));
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
    
    // Check cache first
    auto srcIt = DImplementation->ShortestPathCache.find(src);
    if (srcIt != DImplementation->ShortestPathCache.end()) {
        auto destIt = srcIt->second.find(dest);
        if (destIt != srcIt->second.end()) {
            // Cache hit - use cached path
            auto pathIt = DImplementation->ShortestPathNodesCache.find(src);
            if (pathIt != DImplementation->ShortestPathNodesCache.end()) {
                auto pathDestIt = pathIt->second.find(dest);
                if (pathDestIt != pathIt->second.end()) {
                    path = pathDestIt->second;
                    return destIt->second;
                }
            }
        }
    }
    
    // Cache miss - compute path
    std::vector<CPathRouter::TVertexID> pathVertices;
    double distance = DImplementation->ShortestPathRouter->FindShortestPath(src, dest, pathVertices);
    
    if(distance != CPathRouter::NoPathExists) {
        for(auto vertex : pathVertices) {
            path.push_back(static_cast<TNodeID>(vertex));
        }
        
        // Store in cache for future use
        DImplementation->ShortestPathCache[src][dest] = distance;
        DImplementation->ShortestPathNodesCache[src][dest] = path;
    }
    
    return distance;
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    path.clear();
    
    // Check cache first
    auto srcIt = DImplementation->FastestPathCache.find(src);
    if (srcIt != DImplementation->FastestPathCache.end()) {
        auto destIt = srcIt->second.find(dest);
        if (destIt != srcIt->second.end()) {
            // Cache hit - use cached path
            auto pathIt = DImplementation->FastestPathStepsCache.find(src);
            if (pathIt != DImplementation->FastestPathStepsCache.end()) {
                auto pathDestIt = pathIt->second.find(dest);
                if (pathDestIt != pathIt->second.end()) {
                    path = pathDestIt->second;
                    return destIt->second;
                }
            }
        }
    }
    
    // Cache miss - compute path
    std::vector<CPathRouter::TVertexID> pathVertices;
    std::vector<CPathRouter::TEdgeLabel> edgeLabels;
    
    double time = DImplementation->FastestPathRouter->FindShortestPath(src, dest, pathVertices, edgeLabels);
    
    if(time != CPathRouter::NoPathExists && !pathVertices.empty()) {
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
        
        // Store in cache for future use
        DImplementation->FastestPathCache[src][dest] = time;
        DImplementation->FastestPathStepsCache[src][dest] = path;
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
                   << startNode->Location().first << ", " << startNode->Location().second << ")";
                description.push_back(ss.str());
                ss.str("");
            }
            currentMode = currentTransportMode;
            continue;
        }
        
        // If transportation mode changes, create a description
        if(currentMode != currentTransportMode || 
           (currentMode == ETransportationMode::Bus && currentTransportMode == ETransportationMode::Bus)){
            // Handle bus route changes
            if(currentMode == ETransportationMode::Bus && currentTransportMode == ETransportationMode::Bus){
                // Find if this is a bus route change
                auto prevNodeID = path[i-1].second;
                SImplementation::TRouteID newRouteID = SImplementation::InvalidRouteID;
                
                // Find the bus route connecting these nodes
                auto busRouteIt = DImplementation->BusRoutes.find(prevNodeID);
                if(busRouteIt != DImplementation->BusRoutes.end()){
                    for(const auto& routeInfo : busRouteIt->second){
                        if(routeInfo.first == currentNodeID){
                            newRouteID = routeInfo.second;
                            break;
                        }
                    }
                }
                
                if(newRouteID != currentBusRoute){
                    // Bus route changed
                    if(currentBusRoute != SImplementation::InvalidRouteID){
                        // End previous bus journey
                        ss << "Take bus route " << currentBusRoute << " to (" 
                           << std::fixed << std::setprecision(6);
                        auto node = StreetMapObj->NodeByID(prevNodeID);
                        if(node){
                            ss << node->Location().first << ", " << node->Location().second << ")";
                        } else {
                            ss << "unknown location)";
                        }
                        description.push_back(ss.str());
                        ss.str("");
                    }
                    
                    // Start new bus journey
                    currentBusRoute = newRouteID;
                    ss << "Board bus route " << currentBusRoute << " at (";
                    auto node = StreetMapObj->NodeByID(prevNodeID);
                    if(node){
                        ss << std::fixed << std::setprecision(6)
                           << node->Location().first << ", " << node->Location().second << ")";
                    } else {
                        ss << "unknown location)";
                    }
                    description.push_back(ss.str());
                    ss.str("");
                }
            } else {
                // Different transportation mode
                if(currentMode == ETransportationMode::Bus){
                    // End a bus journey
                    ss << "Take bus route " << currentBusRoute << " to (";
                    auto node = StreetMapObj->NodeByID(currentNodeID);
                    if(node){
                        ss << std::fixed << std::setprecision(6) 
                           << node->Location().first << ", " << node->Location().second << ")";
                    } else {
                        ss << "unknown location)";
                    }
                    description.push_back(ss.str());
                    ss.str("");
                    currentBusRoute = SImplementation::InvalidRouteID;
                } else {
                    // End a walking/biking journey
                    ss << (currentMode == ETransportationMode::Walk ? "Walk to (" : "Bike to (");
                    auto node = StreetMapObj->NodeByID(currentNodeID);
                    if(node){
                        ss << std::fixed << std::setprecision(6) 
                           << node->Location().first << ", " << node->Location().second << ")";
                    } else {
                        ss << "unknown location)";
                    }
                    description.push_back(ss.str());
                    ss.str("");
                }
                
                // Start new journey with new mode
                if(currentTransportMode == ETransportationMode::Bus){
                    // Find which bus route to take
                    auto prevNodeID = path[i-1].second;
                    SImplementation::TRouteID newRouteID = SImplementation::InvalidRouteID;
                    
                    auto busRouteIt = DImplementation->BusRoutes.find(prevNodeID);
                    if(busRouteIt != DImplementation->BusRoutes.end()){
                        for(const auto& routeInfo : busRouteIt->second){
                            if(routeInfo.first == currentNodeID){
                                newRouteID = routeInfo.second;
                                break;
                            }
                        }
                    }
                    
                    currentBusRoute = newRouteID;
                    ss << "Board bus route " << currentBusRoute << " at (";
                    auto node = StreetMapObj->NodeByID(prevNodeID);
                    if(node){
                        ss << std::fixed << std::setprecision(6) 
                           << node->Location().first << ", " << node->Location().second << ")";
                    } else {
                        ss << "unknown location)";
                    }
                    description.push_back(ss.str());
                    ss.str("");
                }
            }
            
            currentMode = currentTransportMode;
        }
    }
    
    // Handle the last leg of the journey
    if(!path.empty()){
        auto lastNodeID = path.back().second;
        
        if(currentMode == ETransportationMode::Bus){
            // End a bus journey
            ss << "Take bus route " << currentBusRoute << " to (";
            auto node = StreetMapObj->NodeByID(lastNodeID);
            if(node){
                ss << std::fixed << std::setprecision(6) 
                   << node->Location().first << ", " << node->Location().second << ")";
            } else {
                ss << "unknown location)";
            }
            description.push_back(ss.str());
        } else {
            // End a walking/biking journey
            ss << (currentMode == ETransportationMode::Walk ? "Walk to (" : "Bike to (");
            auto node = StreetMapObj->NodeByID(lastNodeID);
            if(node){
                ss << std::fixed << std::setprecision(6) 
                   << node->Location().first << ", " << node->Location().second << ")";
            } else {
                ss << "unknown location)";
            }
            description.push_back(ss.str());
        }
        
        // Add final destination message
        ss.str("");
        ss << "End at (";
        auto node = StreetMapObj->NodeByID(lastNodeID);
        if(node){
            ss << std::fixed << std::setprecision(6) 
               << node->Location().first << ", " << node->Location().second << ")";
        } else {
            ss << "unknown location)";
        }
        description.push_back(ss.str());
    }
    
    return !description.empty();
}