#include "DijkstraTransportationPlanner.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <chrono>
#include <thread>
#include <iostream>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> DConfig;
    std::vector<std::shared_ptr<CStreetMap::SNode>> DNodes;
    std::unordered_map<CStreetMap::TNodeID, size_t> DNodeIDToIndex;
    
    // Precomputed paths for walking, biking, and bus routes
    CPathRouter DWalkingRouter;
    CPathRouter DBikingRouter;
    CPathRouter DBusRouter;
    
    SImplementation(std::shared_ptr<SConfiguration> config) 
        : DConfig(config) {
        
        // Initialize nodes and node ID to index map
        auto StreetMap = DConfig->StreetMap();
        for(size_t Index = 0; Index < StreetMap->NodeCount(); Index++) {
            auto Node = StreetMap->NodeByIndex(Index);
            DNodes.push_back(Node);
            DNodeIDToIndex[Node->ID()] = DNodes.size() - 1;
        }
        
        // Sort nodes by ID for binary search
        std::sort(DNodes.begin(), DNodes.end(), 
                 [](const auto &lhs, const auto &rhs) {
                     return lhs->ID() < rhs->ID();
                 });
        
        // Update index map after sorting
        for(size_t Index = 0; Index < DNodes.size(); Index++) {
            DNodeIDToIndex[DNodes[Index]->ID()] = Index;
        }
        
        // Initialize path routers
        DWalkingRouter.SetNodeCount(DNodes.size());
        DBikingRouter.SetNodeCount(DNodes.size());
        DBusRouter.SetNodeCount(DNodes.size());
        
        // Precompute paths
        PrecomputePaths();
    }
    
    void PrecomputePaths() {
        auto start_time = std::chrono::steady_clock::now();
        auto StreetMap = DConfig->StreetMap();
        auto BusSystem = DConfig->BusSystem();
        double DefaultSpeedLimit = DConfig->DefaultSpeedLimit();
        double WalkSpeed = DConfig->WalkSpeed();
        double BikeSpeed = DConfig->BikeSpeed();
        
        // Add walking edges
        for(size_t Index = 0; Index < StreetMap->WayCount(); Index++) {
            auto Way = StreetMap->WayByIndex(Index);
            auto NodeIDs = Way->NodeIDs();
            
            for(size_t IDIndex = 0; IDIndex + 1 < NodeIDs.size(); IDIndex++) {
                auto FromNodeID = NodeIDs[IDIndex];
                auto ToNodeID = NodeIDs[IDIndex + 1];
                
                if(DNodeIDToIndex.find(FromNodeID) == DNodeIDToIndex.end() || 
                   DNodeIDToIndex.find(ToNodeID) == DNodeIDToIndex.end()) {
                    continue;
                }
                
                auto FromIndex = DNodeIDToIndex[FromNodeID];
                auto ToIndex = DNodeIDToIndex[ToNodeID];
                
                auto FromNode = StreetMap->NodeByID(FromNodeID);
                auto ToNode = StreetMap->NodeByID(ToNodeID);
                
                if(!FromNode || !ToNode) {
                    continue;
                }
                
                double Distance = CPathRouter::HaversineDistance(
                    FromNode->Location().first, FromNode->Location().second,
                    ToNode->Location().first, ToNode->Location().second
                );
                
                // Walking is always allowed
                double WalkTime = Distance / WalkSpeed;
                DWalkingRouter.AddDirectedEdge(FromIndex, ToIndex, WalkTime);
                DWalkingRouter.AddDirectedEdge(ToIndex, FromIndex, WalkTime);
                
                // Biking is allowed if not explicitly forbidden
                bool BikeAllowed = true;
                auto Attributes = Way->Tags();
                for(auto &Attribute : Attributes) {
                    if(Attribute.first == "bicycle" && Attribute.second == "no") {
                        BikeAllowed = false;
                        break;
                    }
                }
                
                if(BikeAllowed) {
                    double BikeTime = Distance / BikeSpeed;
                    DBikingRouter.AddDirectedEdge(FromIndex, ToIndex, BikeTime);
                    DBikingRouter.AddDirectedEdge(ToIndex, FromIndex, BikeTime);
                }
            }
        }
        
        // Add bus routes
        for(size_t RouteIndex = 0; RouteIndex < BusSystem->RouteCount(); RouteIndex++) {
            auto Route = BusSystem->RouteByIndex(RouteIndex);
            auto Stops = Route->StopIDs();
            
            for(size_t StopIndex = 0; StopIndex + 1 < Stops.size(); StopIndex++) {
                auto FromStopID = Stops[StopIndex];
                auto ToStopID = Stops[StopIndex + 1];
                
                auto FromStop = BusSystem->StopByID(FromStopID);
                auto ToStop = BusSystem->StopByID(ToStopID);
                
                if(!FromStop || !ToStop) {
                    continue;
                }
                
                auto FromNodeID = FromStop->NodeID();
                auto ToNodeID = ToStop->NodeID();
                
                if(DNodeIDToIndex.find(FromNodeID) == DNodeIDToIndex.end() || 
                   DNodeIDToIndex.find(ToNodeID) == DNodeIDToIndex.end()) {
                    continue;
                }
                
                auto FromIndex = DNodeIDToIndex[FromNodeID];
                auto ToIndex = DNodeIDToIndex[ToNodeID];
                
                auto FromNode = StreetMap->NodeByID(FromNodeID);
                auto ToNode = StreetMap->NodeByID(ToNodeID);
                
                if(!FromNode || !ToNode) {
                    continue;
                }
                
                double Distance = CPathRouter::HaversineDistance(
                    FromNode->Location().first, FromNode->Location().second,
                    ToNode->Location().first, ToNode->Location().second
                );
                
                // Route time includes bus speed and stop time
                double RouteTime = Distance / DefaultSpeedLimit + DConfig->BusStopTime() / 3600.0;
                DBusRouter.AddDirectedEdge(FromIndex, ToIndex, RouteTime);
            }
        }
        
        // Precompute paths for allocated time
        int seconds_to_precompute = DConfig->PrecomputeTime();
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        // Precompute paths up to the allocated time limit
        while(elapsed < seconds_to_precompute) {
            DWalkingRouter.Precompute();
            DBikingRouter.Precompute();
            DBusRouter.Precompute();
            
            current_time = std::chrono::steady_clock::now();
            elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        }
    }
    
    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        path.clear();
        
        if(DNodeIDToIndex.find(src) == DNodeIDToIndex.end() || 
           DNodeIDToIndex.find(dest) == DNodeIDToIndex.end()) {
            return -1.0; // Invalid nodes
        }
        
        auto SrcIndex = DNodeIDToIndex[src];
        auto DestIndex = DNodeIDToIndex[dest];
        
        std::vector<size_t> indices;
        double distance = DWalkingRouter.FindShortestPath(SrcIndex, DestIndex, indices);
        
        if(distance == CPathRouter::NoPathExists) {
            return -1.0; // No path exists
        }
        
        // Convert indices back to node IDs
        for(auto index : indices) {
            path.push_back(DNodes[index]->ID());
        }
        
        return distance;
    }
    
    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();
        
        if(DNodeIDToIndex.find(src) == DNodeIDToIndex.end() || 
           DNodeIDToIndex.find(dest) == DNodeIDToIndex.end()) {
            return -1.0; // Invalid nodes
        }
        
        auto SrcIndex = DNodeIDToIndex[src];
        auto DestIndex = DNodeIDToIndex[dest];
        
        // Create a multimodal graph for the fastest path
        struct SNodeInfo {
            double Time = std::numeric_limits<double>::max();
            bool Visited = false;
            size_t PrevNode = std::numeric_limits<size_t>::max();
            ETransportationMode PrevMode = ETransportationMode::Walk;
            ETransportationMode CurrMode = ETransportationMode::Walk;
        };
        
        // One set of nodes for each transportation mode
        std::vector<SNodeInfo> WalkNodes(DNodes.size());
        std::vector<SNodeInfo> BikeNodes(DNodes.size());
        std::vector<SNodeInfo> BusNodes(DNodes.size());
        
        // Priority queue for Dijkstra's algorithm
        using TNodeTimeIndex = std::tuple<double, ETransportationMode, size_t>;
        std::priority_queue<TNodeTimeIndex, std::vector<TNodeTimeIndex>, std::greater<TNodeTimeIndex>> PriorityQueue;
        
        // Initialize walking source
        WalkNodes[SrcIndex].Time = 0.0;
        WalkNodes[SrcIndex].CurrMode = ETransportationMode::Walk;
        PriorityQueue.push(std::make_tuple(0.0, ETransportationMode::Walk, SrcIndex));
        
        // Initialize biking source
        BikeNodes[SrcIndex].Time = 0.0;
        BikeNodes[SrcIndex].CurrMode = ETransportationMode::Bike;
        PriorityQueue.push(std::make_tuple(0.0, ETransportationMode::Bike, SrcIndex));
        
        // Initialize bus source
        // Check if the source is a bus stop
        bool SrcIsBusStop = false;
        auto BusSystem = DConfig->BusSystem();
        for(size_t i = 0; i < BusSystem->StopCount(); i++) {
            auto Stop = BusSystem->StopByIndex(i);
            if(Stop->NodeID() == src) {
                SrcIsBusStop = true;
                break;
            }
        }
        
        if(SrcIsBusStop) {
            BusNodes[SrcIndex].Time = 0.0;
            BusNodes[SrcIndex].CurrMode = ETransportationMode::Bus;
            PriorityQueue.push(std::make_tuple(0.0, ETransportationMode::Bus, SrcIndex));
        }
        
        while(!PriorityQueue.empty()) {
            auto [Time, Mode, CurrentIndex] = PriorityQueue.top();
            PriorityQueue.pop();
            
            // Skip if already visited with better time
            std::vector<SNodeInfo> *Nodes;
            if(Mode == ETransportationMode::Walk) {
                Nodes = &WalkNodes;
            } else if(Mode == ETransportationMode::Bike) {
                Nodes = &BikeNodes;
            } else { // Bus
                Nodes = &BusNodes;
            }
            
            if((*Nodes)[CurrentIndex].Visited) {
                continue;
            }
            
            (*Nodes)[CurrentIndex].Visited = true;
            
            // If destination reached, trace back the path
            if(CurrentIndex == DestIndex) {
                // Found the fastest path
                path.clear();
                size_t CurrentNode = DestIndex;
                ETransportationMode CurrentMode = Mode;
                
                while(CurrentNode != SrcIndex) {
                    path.push_back(std::make_pair(CurrentMode, DNodes[CurrentNode]->ID()));
                    
                    // Move to previous node
                    if(CurrentMode == ETransportationMode::Walk) {
                        CurrentMode = WalkNodes[CurrentNode].PrevMode;
                        CurrentNode = WalkNodes[CurrentNode].PrevNode;
                    } else if(CurrentMode == ETransportationMode::Bike) {
                        CurrentMode = BikeNodes[CurrentNode].PrevMode;
                        CurrentNode = BikeNodes[CurrentNode].PrevNode;
                    } else { // Bus
                        CurrentMode = BusNodes[CurrentNode].PrevMode;
                        CurrentNode = BusNodes[CurrentNode].PrevNode;
                    }
                }
                
                // Add source node
                path.push_back(std::make_pair(CurrentMode, DNodes[SrcIndex]->ID()));
                
                // Reverse path to get source to destination
                std::reverse(path.begin(), path.end());
                
                return Time;
            }
            
            // Process edges from current node
            if(Mode == ETransportationMode::Walk) {
                // Walking to walking
                for(size_t EdgeIndex = 0; EdgeIndex < DWalkingRouter.EdgeCount(CurrentIndex); EdgeIndex++) {
                    auto [DestIndex, EdgeTime] = DWalkingRouter.GetEdge(CurrentIndex, EdgeIndex);
                    double NewTime = Time + EdgeTime;
                    
                    if(NewTime < WalkNodes[DestIndex].Time) {
                        WalkNodes[DestIndex].Time = NewTime;
                        WalkNodes[DestIndex].PrevNode = CurrentIndex;
                        WalkNodes[DestIndex].PrevMode = ETransportationMode::Walk;
                        WalkNodes[DestIndex].CurrMode = ETransportationMode::Walk;
                        PriorityQueue.push(std::make_tuple(NewTime, ETransportationMode::Walk, DestIndex));
                    }
                }
                
                // Walking to biking (can switch at any node)
                for(size_t EdgeIndex = 0; EdgeIndex < DBikingRouter.EdgeCount(CurrentIndex); EdgeIndex++) {
                    auto [DestIndex, EdgeTime] = DBikingRouter.GetEdge(CurrentIndex, EdgeIndex);
                    double NewTime = Time + EdgeTime;
                    
                    if(NewTime < BikeNodes[DestIndex].Time) {
                        BikeNodes[DestIndex].Time = NewTime;
                        BikeNodes[DestIndex].PrevNode = CurrentIndex;
                        BikeNodes[DestIndex].PrevMode = ETransportationMode::Walk;
                        BikeNodes[DestIndex].CurrMode = ETransportationMode::Bike;
                        PriorityQueue.push(std::make_tuple(NewTime, ETransportationMode::Bike, DestIndex));
                    }
                }
                
                // Walking to bus (only at bus stops)
                bool CurrentIsBusStop = false;
                for(size_t i = 0; i < BusSystem->StopCount(); i++) {
                    auto Stop = BusSystem->StopByIndex(i);
                    if(Stop->NodeID() == DNodes[CurrentIndex]->ID()) {
                        CurrentIsBusStop = true;
                        break;
                    }
                }
                
                if(CurrentIsBusStop) {
                    for(size_t EdgeIndex = 0; EdgeIndex < DBusRouter.EdgeCount(CurrentIndex); EdgeIndex++) {
                        auto [DestIndex, EdgeTime] = DBusRouter.GetEdge(CurrentIndex, EdgeIndex);
                        double NewTime = Time + EdgeTime;
                        
                        if(NewTime < BusNodes[DestIndex].Time) {
                            BusNodes[DestIndex].Time = NewTime;
                            BusNodes[DestIndex].PrevNode = CurrentIndex;
                            BusNodes[DestIndex].PrevMode = ETransportationMode::Walk;
                            BusNodes[DestIndex].CurrMode = ETransportationMode::Bus;
                            PriorityQueue.push(std::make_tuple(NewTime, ETransportationMode::Bus, DestIndex));
                        }
                    }
                }
            } else if(Mode == ETransportationMode::Bike) {
                // Biking to biking
                for(size_t EdgeIndex = 0; EdgeIndex < DBikingRouter.EdgeCount(CurrentIndex); EdgeIndex++) {
                    auto [DestIndex, EdgeTime] = DBikingRouter.GetEdge(CurrentIndex, EdgeIndex);
                    double NewTime = Time + EdgeTime;
                    
                    if(NewTime < BikeNodes[DestIndex].Time) {
                        BikeNodes[DestIndex].Time = NewTime;
                        BikeNodes[DestIndex].PrevNode = CurrentIndex;
                        BikeNodes[DestIndex].PrevMode = ETransportationMode::Bike;
                        BikeNodes[DestIndex].CurrMode = ETransportationMode::Bike;
                        PriorityQueue.push(std::make_tuple(NewTime, ETransportationMode::Bike, DestIndex));
                    }
                }
                
                // Biking to walking (can switch at any node)
                for(size_t EdgeIndex = 0; EdgeIndex < DWalkingRouter.EdgeCount(CurrentIndex); EdgeIndex++) {
                    auto [DestIndex, EdgeTime] = DWalkingRouter.GetEdge(CurrentIndex, EdgeIndex);
                    double NewTime = Time + EdgeTime;
                    
                    if(NewTime < WalkNodes[DestIndex].Time) {
                        WalkNodes[DestIndex].Time = NewTime;
                        WalkNodes[DestIndex].PrevNode = CurrentIndex;
                        WalkNodes[DestIndex].PrevMode = ETransportationMode::Bike;
                        WalkNodes[DestIndex].CurrMode = ETransportationMode::Walk;
                        PriorityQueue.push(std::make_tuple(NewTime, ETransportationMode::Walk, DestIndex));
                    }
                }
            } else { // Bus
                // Bus to bus
                for(size_t EdgeIndex = 0; EdgeIndex < DBusRouter.EdgeCount(CurrentIndex); EdgeIndex++) {
                    auto [DestIndex, EdgeTime] = DBusRouter.GetEdge(CurrentIndex, EdgeIndex);
                    double NewTime = Time + EdgeTime;
                    
                    if(NewTime < BusNodes[DestIndex].Time) {
                        BusNodes[DestIndex].Time = NewTime;
                        BusNodes[DestIndex].PrevNode = CurrentIndex;
                        BusNodes[DestIndex].PrevMode = ETransportationMode::Bus;
                        BusNodes[DestIndex].CurrMode = ETransportationMode::Bus;
                        PriorityQueue.push(std::make_tuple(NewTime, ETransportationMode::Bus, DestIndex));
                    }
                }
                
                // Bus to walking (can switch at any node)
                for(size_t EdgeIndex = 0; EdgeIndex < DWalkingRouter.EdgeCount(CurrentIndex); EdgeIndex++) {
                    auto [DestIndex, EdgeTime] = DWalkingRouter.GetEdge(CurrentIndex, EdgeIndex);
                    double NewTime = Time + EdgeTime;
                    
                    if(NewTime < WalkNodes[DestIndex].Time) {
                        WalkNodes[DestIndex].Time = NewTime;
                        WalkNodes[DestIndex].PrevNode = CurrentIndex;
                        WalkNodes[DestIndex].PrevMode = ETransportationMode::Bus;
                        WalkNodes[DestIndex].CurrMode = ETransportationMode::Walk;
                        PriorityQueue.push(std::make_tuple(NewTime, ETransportationMode::Walk, DestIndex));
                    }
                }
            }
        }
        
        // No path found
        return -1.0;
    }
    
    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();
        
        if(path.empty()) {
            return false;
        }
        
        auto StreetMap = DConfig->StreetMap();
        auto BusSystem = DConfig->BusSystem();
        
        // Get street name for a segment if possible
        auto GetStreetName = [&](TNodeID from, TNodeID to) -> std::string {
            for(size_t WayIndex = 0; WayIndex < StreetMap->WayCount(); WayIndex++) {
                auto Way = StreetMap->WayByIndex(WayIndex);
                auto NodeIDs = Way->NodeIDs();
                
                for(size_t IDIndex = 0; IDIndex + 1 < NodeIDs.size(); IDIndex++) {
                    if((NodeIDs[IDIndex] == from && NodeIDs[IDIndex + 1] == to) ||
                       (NodeIDs[IDIndex] == to && NodeIDs[IDIndex + 1] == from)) {
                        auto Tags = Way->Tags();
                        for(auto &Tag : Tags) {
                            if(Tag.first == "name") {
                                return Tag.second;
                            }
                        }
                        break;
                    }
                }
            }
            return "unnamed street";
        };
        
        // Get bus route for a segment if possible
        auto GetBusRoute = [&](TNodeID from, TNodeID to) -> std::string {
            for(size_t RouteIndex = 0; RouteIndex < BusSystem->RouteCount(); RouteIndex++) {
                auto Route = BusSystem->RouteByIndex(RouteIndex);
                auto StopIDs = Route->StopIDs();
                
                for(size_t StopIndex = 0; StopIndex + 1 < StopIDs.size(); StopIndex++) {
                    auto FromStop = BusSystem->StopByID(StopIDs[StopIndex]);
                    auto ToStop = BusSystem->StopByID(StopIDs[StopIndex + 1]);
                    
                    if(!FromStop || !ToStop) {
                        continue;
                    }
                    
                    if((FromStop->NodeID() == from && ToStop->NodeID() == to) ||
                       (FromStop->NodeID() == to && ToStop->NodeID() == from)) {
                        return Route->Name();
                    }
                }
            }
            return "unknown route";
        };
        
        ETransportationMode CurrentMode = path[0].first;
        std::string CurrentStreet = "";
        std::string CurrentRoute = "";
        TNodeID StartNode = path[0].second;
        
        for(size_t i = 1; i < path.size(); i++) {
            auto PrevMode = CurrentMode;
            auto PrevNode = path[i - 1].second;
            CurrentMode = path[i].first;
            auto CurrentNode = path[i].second;
            
            // Mode change
            if(PrevMode != CurrentMode) {
                if(PrevMode == ETransportationMode::Walk) {
                    desc.push_back("Walk to " + std::to_string(PrevNode));
                } else if(PrevMode == ETransportationMode::Bike) {
                    desc.push_back("Bike to " + std::to_string(PrevNode));
                } else if(PrevMode == ETransportationMode::Bus) {
                    desc.push_back("Take bus to " + std::to_string(PrevNode));
                }
                
                if(CurrentMode == ETransportationMode::Walk) {
                    desc.push_back("Start walking");
                } else if(CurrentMode == ETransportationMode::Bike) {
                    desc.push_back("Start biking");
                } else if(CurrentMode == ETransportationMode::Bus) {
                    desc.push_back("Board bus at stop " + std::to_string(PrevNode));
                }
                
                StartNode = PrevNode;
                
                if(CurrentMode == ETransportationMode::Bus) {
                    CurrentRoute = GetBusRoute(PrevNode, CurrentNode);
                    if(CurrentRoute != "unknown route") {
                        desc.push_back("Take " + CurrentRoute);
                    }
                } else {
                    CurrentStreet = GetStreetName(PrevNode, CurrentNode);
                    if(CurrentStreet != "unnamed street") {
                        desc.push_back("Take " + CurrentStreet);
                    }
                }
            } else if(CurrentMode != ETransportationMode::Bus) {
                // Continue on same mode, check for street change
                std::string NewStreet = GetStreetName(PrevNode, CurrentNode);
                if(NewStreet != CurrentStreet && NewStreet != "unnamed street") {
                    if(CurrentStreet != "") {
                        if(CurrentMode == ETransportationMode::Walk) {
                            desc.push_back("Walk along " + CurrentStreet + " to " + std::to_string(PrevNode));
                        } else if(CurrentMode == ETransportationMode::Bike) {
                            desc.push_back("Bike along " + CurrentStreet + " to " + std::to_string(PrevNode));
                        }
                    }
                    desc.push_back("Take " + NewStreet);
                    CurrentStreet = NewStreet;
                    StartNode = PrevNode;
                }
            } else {
                // Continue on bus, check for route change
                std::string NewRoute = GetBusRoute(PrevNode, CurrentNode);
                if(NewRoute != CurrentRoute && NewRoute != "unknown route") {
                    if(CurrentRoute != "") {
                        desc.push_back("Take bus route " + CurrentRoute + " to " + std::to_string(PrevNode));
                    }
                    desc.push_back("Transfer to bus route " + NewRoute + " at stop " + std::to_string(PrevNode));
                    CurrentRoute = NewRoute;
                    StartNode = PrevNode;
                }
            }
        }
        
        // Add final segment
        TNodeID FinalNode = path.back().second;
        if(CurrentMode == ETransportationMode::Walk) {
            if(CurrentStreet != "") {
                desc.push_back("Walk along " + CurrentStreet + " to " + std::to_string(FinalNode));
            } else {
                desc.push_back("Walk to " + std::to_string(FinalNode));
            }
        } else if(CurrentMode == ETransportationMode::Bike) {
            if(CurrentStreet != "") {
                desc.push_back("Bike along " + CurrentStreet + " to " + std::to_string(FinalNode));
            } else {
                desc.push_back("Bike to " + std::to_string(FinalNode));
            }
        } else if(CurrentMode == ETransportationMode::Bus) {
            if(CurrentRoute != "") {
                desc.push_back("Take bus route " + CurrentRoute + " to " + std::to_string(FinalNode));
            } else {
                desc.push_back("Take bus to " + std::to_string(FinalNode));
            }
            desc.push_back("Exit bus at stop " + std::to_string(FinalNode));
        }
        
        return true;
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
    
}

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->DNodes.size();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    if(index < DImplementation->DNodes.size()) {
        return DImplementation->DNodes[index];
    }
    return nullptr;
}

double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    return DImplementation->FindShortestPath(src, dest, path);
}

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    return DImplementation->FindFastestPath(src, dest, path);
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    return DImplementation->GetPathDescription(path, desc);
}