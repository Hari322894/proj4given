#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <chrono>

struct CDijkstraTransportationPlanner::SImplementation{
    std::shared_ptr<CTransportationPlanner::SConfiguration> DConfig;
    std::vector<CTransportationPlanner::SNode> DNodes;
    std::vector<std::shared_ptr<CStreetMap::SNode>> DStreetMapNodes;
    std::vector<std::shared_ptr<CBusSystem::SStop>> DBusStops;
    
    // Use concrete implementation CDijkstraPathRouter instead of abstract CPathRouter
    CDijkstraPathRouter DWalkingRouter;
    CDijkstraPathRouter DBikingRouter;
    CDijkstraPathRouter DBusRouter;
    
    SImplementation(std::shared_ptr<CTransportationPlanner::SConfiguration> config)
        : DConfig(config) {
        
        // Extract all street map nodes
        for(size_t NodeIndex = 0; NodeIndex < DConfig->StreetMap()->NodeCount(); NodeIndex++){
            auto Node = DConfig->StreetMap()->NodeByIndex(NodeIndex);
            DStreetMapNodes.push_back(Node);
            CTransportationPlanner::SNode TransporationNode;
            TransporationNode.Location = Node->Location();
            DNodes.push_back(TransporationNode);
        }
        
        // Extract all bus stop nodes
        for(size_t StopIndex = 0; StopIndex < DConfig->BusSystem()->StopCount(); StopIndex++){
            auto Stop = DConfig->BusSystem()->StopByIndex(StopIndex);
            DBusStops.push_back(Stop);
            CTransportationPlanner::SNode TransporationNode;
            TransporationNode.Location = Stop->Location();
            DNodes.push_back(TransporationNode);
        }
        
        // Add nodes to the routers
        for (size_t i = 0; i < DNodes.size(); i++) {
            DWalkingRouter.AddVertex(i);
            DBikingRouter.AddVertex(i);
            DBusRouter.AddVertex(i);
        }
    }
    
    ~SImplementation(){
    
    }
    
    void PrecomputePaths() {
        // Process Street Map Nodes
        for(size_t WayIndex = 0; WayIndex < DConfig->StreetMap()->WayCount(); WayIndex++){
            auto Way = DConfig->StreetMap()->WayByIndex(WayIndex);
            // Get node IDs from the way
            std::vector<CStreetMap::TNodeID> NodeIDs;
            for (size_t i = 0; i < Way->NodeCount(); i++) {
                NodeIDs.push_back(Way->GetNodeID(i));
            }
            
            // Ensure there are at least 2 nodes to form an edge
            if(NodeIDs.size() < 2){
                continue;
            }
            
            // Map Way nodes to our transportation nodes
            std::vector<CTransportationPlanner::TNodeID> TransNodeIDs;
            for(auto NodeID : NodeIDs){
                auto Node = DConfig->StreetMap()->NodeByID(NodeID);
                bool Found = false;
                
                for(size_t Index = 0; Index < DStreetMapNodes.size(); Index++){
                    if(DStreetMapNodes[Index]->ID() == NodeID){
                        TransNodeIDs.push_back(Index);
                        Found = true;
                        break;
                    }
                }
                
                if(!Found){
                    TransNodeIDs.push_back(std::numeric_limits<CTransportationPlanner::TNodeID>::max());
                }
            }
            
            // Create edges for each sequential pair
            for(size_t Index = 1; Index < TransNodeIDs.size(); Index++){
                auto FromIndex = TransNodeIDs[Index-1];
                auto ToIndex = TransNodeIDs[Index];
                
                if((FromIndex == std::numeric_limits<CTransportationPlanner::TNodeID>::max()) 
                   || (ToIndex == std::numeric_limits<CTransportationPlanner::TNodeID>::max())){
                    continue;
                }
                
                // Calculate Haversine distance
                double Distance = HaversineDistance(
                                    DNodes[FromIndex].Location.first, 
                                    DNodes[FromIndex].Location.second,
                                    DNodes[ToIndex].Location.first, 
                                    DNodes[ToIndex].Location.second);
                
                // Walking is always allowed
                double WalkTime = Distance / DConfig->WalkSpeed();
                DWalkingRouter.AddEdge(FromIndex, ToIndex, WalkTime, true);
                
                // Biking may be restricted
                auto Attributes = Way->GetAttributes();
                bool BikingAllowed = true;
                
                for(auto Attribute : Attributes){
                    if(Attribute.first == "highway"){
                        if((Attribute.second == "motorway") || (Attribute.second == "motorway_link")){
                            BikingAllowed = false;
                        }
                    }
                }
                
                if(BikingAllowed){
                    double BikeTime = Distance / DConfig->BikeSpeed();
                    DBikingRouter.AddEdge(FromIndex, ToIndex, BikeTime, true);
                }
            }
        }
        
        // Process Bus Routes
        for(size_t RouteIndex = 0; RouteIndex < DConfig->BusSystem()->RouteCount(); RouteIndex++){
            auto Route = DConfig->BusSystem()->RouteByIndex(RouteIndex);
            // Get stop IDs from the route
            std::vector<CBusSystem::TStopID> Stops;
            for (size_t i = 0; i < Route->StopCount(); i++) {
                Stops.push_back(Route->GetStopID(i));
            }
            
            // Ensure there are at least 2 stops to form an edge
            if(Stops.size() < 2){
                continue;
            }
            
            // Map Bus stops to our transportation nodes
            std::vector<CTransportationPlanner::TNodeID> TransNodeIDs;
            size_t StopOffset = DStreetMapNodes.size();
            
            for(auto StopID : Stops){
                auto Stop = DConfig->BusSystem()->StopByID(StopID);
                bool Found = false;
                
                for(size_t Index = 0; Index < DBusStops.size(); Index++){
                    if(DBusStops[Index]->ID() == StopID){
                        TransNodeIDs.push_back(Index + StopOffset);
                        Found = true;
                        break;
                    }
                }
                
                if(!Found){
                    TransNodeIDs.push_back(std::numeric_limits<CTransportationPlanner::TNodeID>::max());
                }
            }
            
            // Create edges for each sequential pair
            for(size_t Index = 1; Index < TransNodeIDs.size(); Index++){
                auto FromIndex = TransNodeIDs[Index-1];
                auto ToIndex = TransNodeIDs[Index];
                
                if((FromIndex == std::numeric_limits<CTransportationPlanner::TNodeID>::max()) 
                   || (ToIndex == std::numeric_limits<CTransportationPlanner::TNodeID>::max())){
                    continue;
                }
                
                // Calculate Haversine distance
                double Distance = HaversineDistance(
                                    DNodes[FromIndex].Location.first, 
                                    DNodes[FromIndex].Location.second,
                                    DNodes[ToIndex].Location.first, 
                                    DNodes[ToIndex].Location.second);
                
                double RouteTime = Distance / DConfig->BusSpeed();
                DBusRouter.AddEdge(FromIndex, ToIndex, RouteTime, false);
            }
        }
        
        try{
            // Use a far future deadline for precomputation
            auto Deadline = std::chrono::steady_clock::now() + std::chrono::hours(1);
            DWalkingRouter.Precompute(Deadline);
            DBikingRouter.Precompute(Deadline);
            DBusRouter.Precompute(Deadline);
        }
        catch(std::exception &Exception){
            std::cout << Exception.what() << std::endl;
        }
    }
    
    // Helper function for Haversine distance
    double HaversineDistance(double lat1, double lon1, double lat2, double lon2) {
        // Convert to radians
        const double PI = 3.14159265358979323846;
        lat1 *= PI / 180.0;
        lon1 *= PI / 180.0;
        lat2 *= PI / 180.0;
        lon2 *= PI / 180.0;
        
        // Haversine formula
        double dlon = lon2 - lon1;
        double dlat = lat2 - lat1;
        double a = std::sin(dlat/2) * std::sin(dlat/2) + std::cos(lat1) * std::cos(lat2) * std::sin(dlon/2) * std::sin(dlon/2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
        
        // Earth radius in meters
        const double R = 6371000.0;
        return R * c;
    }
    
    double FindFastestPath(CTransportationPlanner::TNodeID src, CTransportationPlanner::TNodeID dest, std::vector<std::pair<CTransportationPlanner::ETransportationMode, CTransportationPlanner::TNodeID>> &path){
        // Initialize the data structures for Dijkstra's algorithm
        std::priority_queue<std::pair<double, std::tuple<CTransportationPlanner::TNodeID, CTransportationPlanner::ETransportationMode, CTransportationPlanner::TNodeID>>, 
                           std::vector<std::pair<double, std::tuple<CTransportationPlanner::TNodeID, CTransportationPlanner::ETransportationMode, CTransportationPlanner::TNodeID>>>, 
                           std::greater<std::pair<double, std::tuple<CTransportationPlanner::TNodeID, CTransportationPlanner::ETransportationMode, CTransportationPlanner::TNodeID>>>> PriorityQueue;
        
        // Track best time to reach each node with each transportation mode
        std::unordered_map<std::string, double> ShortestTime;
        std::unordered_map<std::string, std::tuple<CTransportationPlanner::TNodeID, CTransportationPlanner::ETransportationMode, CTransportationPlanner::TNodeID>> Previous;
        
        // Helper function to create a unique key for node+mode combinations
        auto CreateStateKey = [](CTransportationPlanner::TNodeID node, CTransportationPlanner::ETransportationMode mode) -> std::string {
            return std::to_string(node) + "-" + std::to_string(static_cast<int>(mode));
        };
        
        // Initialize sources with different modes
        PriorityQueue.push({0.0, {src, CTransportationPlanner::ETransportationMode::Walking, src}});
        PriorityQueue.push({0.0, {src, CTransportationPlanner::ETransportationMode::Biking, src}});
        
        // Initialize all states with infinity
        for(size_t Index = 0; Index < DNodes.size(); Index++){
            ShortestTime[CreateStateKey(Index, CTransportationPlanner::ETransportationMode::Walking)] = std::numeric_limits<double>::max();
            ShortestTime[CreateStateKey(Index, CTransportationPlanner::ETransportationMode::Biking)] = std::numeric_limits<double>::max();
            ShortestTime[CreateStateKey(Index, CTransportationPlanner::ETransportationMode::Bus)] = std::numeric_limits<double>::max();
        }
        
        // Set initial states
        ShortestTime[CreateStateKey(src, CTransportationPlanner::ETransportationMode::Walking)] = 0.0;
        ShortestTime[CreateStateKey(src, CTransportationPlanner::ETransportationMode::Biking)] = 0.0;
        
        // Run Dijkstra's algorithm
        while(!PriorityQueue.empty()){
            auto TimeTuple = PriorityQueue.top();
            PriorityQueue.pop();
            
            auto CurrentTime = TimeTuple.first;
            auto CurrentIndex = std::get<0>(TimeTuple.second);
            auto CurrentMode = std::get<1>(TimeTuple.second);
            auto PreviousIndex = std::get<2>(TimeTuple.second);
            
            // Skip if we have a better path to this node+mode already
            std::string StateKey = CreateStateKey(CurrentIndex, CurrentMode);
            if(CurrentTime > ShortestTime[StateKey]){
                continue;
            }
            
            // If we've reached the destination, we're done
            if(CurrentIndex == dest){
                // Reconstruct the path
                path.clear();
                auto Current = CurrentIndex;
                auto CurrMode = CurrentMode;
                
                while(Current != src){
                    path.push_back({CurrMode, Current});
                    auto PrevState = Previous[CreateStateKey(Current, CurrMode)];
                    Current = std::get<0>(PrevState);
                    CurrMode = std::get<1>(PrevState);
                }
                
                // Reverse the path
                std::reverse(path.begin(), path.end());
                return CurrentTime;
            }
            
            // Consider all neighbors based on the current mode
            if(CurrentMode == CTransportationPlanner::ETransportationMode::Walking){
                // Get direct neighbors only, not full paths
                for(size_t i = 0; i < DNodes.size(); i++) {
                    if(i == CurrentIndex) continue;
                    
                    std::vector<CTransportationPlanner::TNodeID> edgePath;
                    double EdgeTime = DWalkingRouter.FindShortestPath(CurrentIndex, i, edgePath);
                    
                    if(EdgeTime != std::numeric_limits<double>::max()) {
                        double NewTime = CurrentTime + EdgeTime;
                        std::string DestStateKey = CreateStateKey(i, CTransportationPlanner::ETransportationMode::Walking);
                        
                        if(NewTime < ShortestTime[DestStateKey]){
                            ShortestTime[DestStateKey] = NewTime;
                            Previous[DestStateKey] = {CurrentIndex, CTransportationPlanner::ETransportationMode::Walking, CurrentIndex};
                            PriorityQueue.push({NewTime, {i, CTransportationPlanner::ETransportationMode::Walking, CurrentIndex}});
                        }
                    }
                }
                
                // Try switching to biking
                std::string BikingStateKey = CreateStateKey(CurrentIndex, CTransportationPlanner::ETransportationMode::Biking);
                if(CurrentTime < ShortestTime[BikingStateKey]){
                    ShortestTime[BikingStateKey] = CurrentTime;
                    Previous[BikingStateKey] = {CurrentIndex, CTransportationPlanner::ETransportationMode::Walking, PreviousIndex};
                    PriorityQueue.push({CurrentTime, {CurrentIndex, CTransportationPlanner::ETransportationMode::Biking, CurrentIndex}});
                }
                
                // Try switching to bus (if at a bus stop)
                if(CurrentIndex >= DStreetMapNodes.size() && CurrentIndex < DStreetMapNodes.size() + DBusStops.size()){
                    std::string BusStateKey = CreateStateKey(CurrentIndex, CTransportationPlanner::ETransportationMode::Bus);
                    if(CurrentTime < ShortestTime[BusStateKey]){
                        ShortestTime[BusStateKey] = CurrentTime;
                        Previous[BusStateKey] = {CurrentIndex, CTransportationPlanner::ETransportationMode::Walking, PreviousIndex};
                        PriorityQueue.push({CurrentTime, {CurrentIndex, CTransportationPlanner::ETransportationMode::Bus, CurrentIndex}});
                    }
                }
            }
            else if(CurrentMode == CTransportationPlanner::ETransportationMode::Biking){
                // Get direct neighbors only
                for(size_t i = 0; i < DNodes.size(); i++) {
                    if(i == CurrentIndex) continue;
                    
                    std::vector<CTransportationPlanner::TNodeID> edgePath;
                    double EdgeTime = DBikingRouter.FindShortestPath(CurrentIndex, i, edgePath);
                    
                    if(EdgeTime != std::numeric_limits<double>::max()) {
                        double NewTime = CurrentTime + EdgeTime;
                        std::string DestStateKey = CreateStateKey(i, CTransportationPlanner::ETransportationMode::Biking);
                        
                        if(NewTime < ShortestTime[DestStateKey]){
                            ShortestTime[DestStateKey] = NewTime;
                            Previous[DestStateKey] = {CurrentIndex, CTransportationPlanner::ETransportationMode::Biking, CurrentIndex};
                            PriorityQueue.push({NewTime, {i, CTransportationPlanner::ETransportationMode::Biking, CurrentIndex}});
                        }
                    }
                }
                
                // Try switching to walking
                std::string WalkingStateKey = CreateStateKey(CurrentIndex, CTransportationPlanner::ETransportationMode::Walking);
                if(CurrentTime < ShortestTime[WalkingStateKey]){
                    ShortestTime[WalkingStateKey] = CurrentTime;
                    Previous[WalkingStateKey] = {CurrentIndex, CTransportationPlanner::ETransportationMode::Biking, PreviousIndex};
                    PriorityQueue.push({CurrentTime, {CurrentIndex, CTransportationPlanner::ETransportationMode::Walking, CurrentIndex}});
                }
            }
            else if(CurrentMode == CTransportationPlanner::ETransportationMode::Bus){
                // Get direct neighbors only
                for(size_t i = 0; i < DNodes.size(); i++) {
                    if(i == CurrentIndex) continue;
                    
                    std::vector<CTransportationPlanner::TNodeID> edgePath;
                    double EdgeTime = DBusRouter.FindShortestPath(CurrentIndex, i, edgePath);
                    
                    if(EdgeTime != std::numeric_limits<double>::max()) {
                        double NewTime = CurrentTime + EdgeTime;
                        std::string DestStateKey = CreateStateKey(i, CTransportationPlanner::ETransportationMode::Bus);
                        
                        if(NewTime < ShortestTime[DestStateKey]){
                            ShortestTime[DestStateKey] = NewTime;
                            Previous[DestStateKey] = {CurrentIndex, CTransportationPlanner::ETransportationMode::Bus, CurrentIndex};
                            PriorityQueue.push({NewTime, {i, CTransportationPlanner::ETransportationMode::Bus, CurrentIndex}});
                        }
                    }
                }
                
                // Try switching to walking (only possible at bus stops)
                std::string WalkingStateKey = CreateStateKey(CurrentIndex, CTransportationPlanner::ETransportationMode::Walking);
                if(CurrentTime < ShortestTime[WalkingStateKey]){
                    ShortestTime[WalkingStateKey] = CurrentTime;
                    Previous[WalkingStateKey] = {CurrentIndex, CTransportationPlanner::ETransportationMode::Bus, PreviousIndex};
                    PriorityQueue.push({CurrentTime, {CurrentIndex, CTransportationPlanner::ETransportationMode::Walking, CurrentIndex}});
                }
            }
        }
        
        // No path found
        return std::numeric_limits<double>::max();
    }
    
    bool FindPathByFewestNodes(CTransportationPlanner::TNodeID src, CTransportationPlanner::TNodeID dest, std::vector<CTransportationPlanner::TNodeID> &path){
        // Initialize priority queue for BFS
        std::queue<CTransportationPlanner::TNodeID> Queue;
        std::unordered_map<CTransportationPlanner::TNodeID, CTransportationPlanner::TNodeID> Previous;
        std::unordered_set<CTransportationPlanner::TNodeID> Visited;
        
        Queue.push(src);
        Visited.insert(src);
        
        while(!Queue.empty()){
            auto CurrentID = Queue.front();
            Queue.pop();
            
            if(CurrentID == dest){
                // Reconstruct path
                path.clear();
                path.push_back(dest);
                
                auto Current = dest;
                while(Current != src){
                    Current = Previous[Current];
                    path.push_back(Current);
                }
                
                std::reverse(path.begin(), path.end());
                return true;
            }
            
            // Find neighbors via all modes
            auto processWays = [&](CTransportationPlanner::TNodeID nodeID) {
                for(size_t WayIndex = 0; WayIndex < DConfig->StreetMap()->WayCount(); WayIndex++){
                    auto Way = DConfig->StreetMap()->WayByIndex(WayIndex);
                    // Extract node IDs from the way
                    std::vector<CStreetMap::TNodeID> NodeIDs;
                    for (size_t i = 0; i < Way->NodeCount(); i++) {
                        NodeIDs.push_back(Way->GetNodeID(i));
                    }
                    
                    for(size_t Index = 0; Index < NodeIDs.size(); Index++){
                        auto StreetMapNodeID = NodeIDs[Index];
                        
                        for(size_t TransNodeIndex = 0; TransNodeIndex < DStreetMapNodes.size(); TransNodeIndex++){
                            if(DStreetMapNodes[TransNodeIndex]->ID() == StreetMapNodeID){
                                if((TransNodeIndex != nodeID) && (Visited.find(TransNodeIndex) == Visited.end())){
                                    // Check if this way is accessible
                                    bool Accessible = true;
                                    
                                    // Check for highway=motorway tag
                                    auto Tags = Way->GetAttributes();
                                    for(auto Tag : Tags){
                                        if((Tag.first == "highway") && (Tag.second == "motorway")){
                                            Accessible = false;
                                            break;
                                        }
                                    }
                                    
                                    if(Accessible){
                                        Previous[TransNodeIndex] = nodeID;
                                        Visited.insert(TransNodeIndex);
                                        Queue.push(TransNodeIndex);
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            };
            
            auto processRoutes = [&](CTransportationPlanner::TNodeID nodeID) {
                for(size_t RouteIndex = 0; RouteIndex < DConfig->BusSystem()->RouteCount(); RouteIndex++){
                    auto Route = DConfig->BusSystem()->RouteByIndex(RouteIndex);
                    // Extract stop IDs from the route
                    std::vector<CBusSystem::TStopID> StopIDs;
                    for (size_t i = 0; i < Route->StopCount(); i++) {
                        StopIDs.push_back(Route->GetStopID(i));
                    }
                    
                    for(size_t Index = 0; Index < StopIDs.size(); Index++){
                        auto BusStopID = StopIDs[Index];
                        
                        for(size_t TransNodeIndex = 0; TransNodeIndex < DBusStops.size(); TransNodeIndex++){
                            if(DBusStops[TransNodeIndex]->ID() == BusStopID){
                                auto GlobalNodeID = TransNodeIndex + DStreetMapNodes.size();
                                if((GlobalNodeID != nodeID) && (Visited.find(GlobalNodeID) == Visited.end())){
                                    Previous[GlobalNodeID] = nodeID;
                                    Visited.insert(GlobalNodeID);
                                    Queue.push(GlobalNodeID);
                                }
                                break;
                            }
                        }
                    }
                }
            };
            
            // Process all ways for street map nodes
            processWays(CurrentID);
            
            // Process all routes for bus stops
            processRoutes(CurrentID);
        }
        
        return false;
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config){
    DImplementation = std::make_unique<SImplementation>(config);
    DImplementation->PrecomputePaths();
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner(){
    
}

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept{
    return DImplementation->DNodes.size();
}

CTransportationPlanner::TNodeID CDijkstraTransportationPlanner::GetNodeID(const CStreetMap::TNodeID &nodeid) const noexcept{
    for(size_t Index = 0; Index < DImplementation->DStreetMapNodes.size(); Index++){
        if(DImplementation->DStreetMapNodes[Index]->ID() == nodeid){
            return Index;
        }
    }
    return std::numeric_limits<CTransportationPlanner::TNodeID>::max();
}

CTransportationPlanner::TNodeID CDijkstraTransportationPlanner::GetNodeID(const CBusSystem::TStopID &stopid) const noexcept{
    for(size_t Index = 0; Index < DImplementation->DBusStops.size(); Index++){
        if(DImplementation->DBusStops[Index]->ID() == stopid){
            return Index + DImplementation->DStreetMapNodes.size();
        }
    }
    return std::numeric_limits<CTransportationPlanner::TNodeID>::max();
}

CStreetMap::TNodeID CDijkstraTransportationPlanner::GetStreetMapNodeID(TNodeID id) const noexcept{
    if(id < DImplementation->DStreetMapNodes.size()){
        return DImplementation->DStreetMapNodes[id]->ID();
    }
    return std::numeric_limits<CStreetMap::TNodeID>::max();
}

CBusSystem::TStopID CDijkstraTransportationPlanner::GetBusSystemStopID(TNodeID id) const noexcept{
    auto BusIndex = id - DImplementation->DStreetMapNodes.size();
    if(BusIndex < DImplementation->DBusStops.size()){
        return DImplementation->DBusStops[BusIndex]->ID();
    }
    return std::numeric_limits<CBusSystem::TStopID>::max();
}

std::pair<double, double> CDijkstraTransportationPlanner::GetNodeLocation(TNodeID id) const noexcept{
    if(id < DImplementation->DNodes.size()){
        return DImplementation->DNodes[id].Location;
    }
    return std::make_pair(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) noexcept{
    return DImplementation->FindFastestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<std::pair<ETransportationMode, TNodeID>> &path) noexcept{
    return DImplementation->FindFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::FindPathWithFewestNodes(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) noexcept{
    return DImplementation->FindPathByFewestNodes(src, dest, path);
}