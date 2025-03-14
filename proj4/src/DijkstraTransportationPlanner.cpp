#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include "GeographicUtils.h"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <sstream>
#include <cmath>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> DConfig;
    CDijkstraPathRouter DShortestPathRouter;
    CDijkstraPathRouter DFastestPathRouter;
    std::vector<std::shared_ptr<CStreetMap::SNode>> DNodes;
    std::unordered_map<CStreetMap::TNodeID, size_t> DNodeIDToIndex;
    std::unordered_map<CStreetMap::TNodeID, std::vector<CStreetMap::TWayID>> DNodeToWays;
    std::unordered_map<size_t, std::vector<std::pair<size_t, std::string>>> DEdgeStreetNames;
    std::unordered_map<CStreetMap::TNodeID, size_t> DBusStopToIndex;
    std::unordered_map<size_t, CStreetMap::TNodeID> DIndexToBusStop;
    std::unordered_map<std::string, std::vector<size_t>> DBusRoutes;
    
    SImplementation(std::shared_ptr<SConfiguration> config) 
        : DConfig(config) {
        ProcessStreetMap();
        ProcessBusSystem();
        BuildShortestPathRouter();
        BuildFastestPathRouter();
        PrecomputePaths();
    }
    
    void ProcessStreetMap() {
        auto StreetMap = DConfig->StreetMap();
        
        // Create the node mapping
        for(size_t i = 0; i < StreetMap->NodeCount(); ++i) {
            auto Node = StreetMap->NodeByIndex(i);
            DNodes.push_back(Node);
            DNodeIDToIndex[Node->ID()] = DNodes.size() - 1;
        }
        
        // Build the node to ways mapping
        for(size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto Way = StreetMap->WayByIndex(i);
            for(size_t j = 0; j < Way->NodeCount(); ++j) {
                auto NodeID = Way->GetNodeID(j);
                DNodeToWays[NodeID].push_back(Way->ID());
            }
        }
    }
    
    void ProcessBusSystem() {
        auto BusSystem = DConfig->BusSystem();
        
        // Process bus stops
        for(size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto Stop = BusSystem->StopByIndex(i);
            size_t index = DNodeIDToIndex.at(Stop->NodeID());
            DBusStopToIndex[Stop->ID()] = index;
            DIndexToBusStop[index] = Stop->ID();
        }
        
        // Process bus routes
        for(size_t i = 0; i < BusSystem->RouteCount(); ++i) {
            auto Route = BusSystem->RouteByIndex(i);
            std::vector<size_t> RouteStops;
            
            for(size_t j = 0; j < Route->StopCount(); ++j) {
                auto StopID = Route->GetStopID(j);
                auto NodeID = BusSystem->StopByID(StopID)->NodeID();
                RouteStops.push_back(DNodeIDToIndex.at(NodeID));
            }
            
            DBusRoutes[Route->Name()] = RouteStops;
        }
    }
    
    void BuildShortestPathRouter() {
        auto StreetMap = DConfig->StreetMap();
        
        // Add vertices for each node
        for(auto& Node : DNodes) {
            DShortestPathRouter.AddVertex(Node->ID());
        }
        
        // Add edges for each way
        for(size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto Way = StreetMap->WayByIndex(i);
            bool IsOneWay = false;
            
            // Check if the way is one-way
            for(size_t j = 0; j < Way->AttributeCount(); ++j) {
                std::string key = Way->GetAttributeKey(j);
                std::string value = Way->GetAttribute(key);
                if(key == "oneway" && value == "yes") {
                    IsOneWay = true;
                    break;
                }
            }
            
            // Get the street name if available
            std::string StreetName;
            for(size_t j = 0; j < Way->AttributeCount(); ++j) {
                std::string key = Way->GetAttributeKey(j);
                std::string value = Way->GetAttribute(key);
                if(key == "name") {
                    StreetName = value;
                    break;
                }
            }
            
            // Add edges between consecutive nodes
            for(size_t j = 1; j < Way->NodeCount(); ++j) {
                auto PrevNodeID = Way->GetNodeID(j-1);
                auto CurrNodeID = Way->GetNodeID(j);
                
                auto PrevNode = StreetMap->NodeByID(PrevNodeID);
                auto CurrNode = StreetMap->NodeByID(CurrNodeID);
                
                auto PrevLoc = PrevNode->Location();
                auto CurrLoc = CurrNode->Location();
                
                double Distance = SGeographicUtils::HaversineDistanceInMiles(PrevLoc, CurrLoc);
                
                size_t PrevIndex = DNodeIDToIndex.at(PrevNodeID);
                size_t CurrIndex = DNodeIDToIndex.at(CurrNodeID);
                
                DShortestPathRouter.AddEdge(PrevIndex, CurrIndex, Distance, !IsOneWay);
                
                // Store street name for the edge
                if(!StreetName.empty()) {
                    DEdgeStreetNames[PrevIndex].push_back({CurrIndex, StreetName});
                    if(!IsOneWay) {
                        DEdgeStreetNames[CurrIndex].push_back({PrevIndex, StreetName});
                    }
                }
            }
        }
    }
    
    void BuildFastestPathRouter() {
        auto StreetMap = DConfig->StreetMap();
        
        // Add vertices for each node (walking, biking, and bus modes)
        for(auto& Node : DNodes) {
            DFastestPathRouter.AddVertex(std::make_pair(Node->ID(), ETransportationMode::Walk));
            DFastestPathRouter.AddVertex(std::make_pair(Node->ID(), ETransportationMode::Bike));
            DFastestPathRouter.AddVertex(std::make_pair(Node->ID(), ETransportationMode::Bus));
        }
        
        // Add walk/bike/drive edges for each way
        for(size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto Way = StreetMap->WayByIndex(i);
            bool IsOneWay = false;
            double SpeedLimit = DConfig->DefaultSpeedLimit();
            
            // Check way attributes
            for(size_t j = 0; j < Way->AttributeCount(); ++j) {
                std::string key = Way->GetAttributeKey(j);
                std::string value = Way->GetAttribute(key);
                if(key == "oneway" && value == "yes") {
                    IsOneWay = true;
                }
                else if(key == "maxspeed" && value.find("mph") != std::string::npos) {
                    try {
                        SpeedLimit = std::stod(value.substr(0, value.find("mph")));
                    }
                    catch(...) {
                        // Use default speed limit if conversion fails
                    }
                }
            }
            
            // Add edges between consecutive nodes
            for(size_t j = 1; j < Way->NodeCount(); ++j) {
                auto PrevNodeID = Way->GetNodeID(j-1);
                auto CurrNodeID = Way->GetNodeID(j);
                
                auto PrevNode = StreetMap->NodeByID(PrevNodeID);
                auto CurrNode = StreetMap->NodeByID(CurrNodeID);
                
                auto PrevLoc = PrevNode->Location();
                auto CurrLoc = CurrNode->Location();
                
                double Distance = SGeographicUtils::HaversineDistanceInMiles(PrevLoc, CurrLoc);
                
                size_t PrevIndex = DNodeIDToIndex.at(PrevNodeID);
                size_t CurrIndex = DNodeIDToIndex.at(CurrNodeID);
                
                // Walking time (in hours) = distance / walk speed
                double WalkTime = Distance / DConfig->WalkSpeed();
                
                // Biking time (in hours) = distance / bike speed
                double BikeTime = Distance / DConfig->BikeSpeed();
                
                // Driving time (in hours) = distance / speed limit
                double DriveTime = Distance / SpeedLimit;
                
                // Add walking edges
                DFastestPathRouter.AddEdge(
                    PrevIndex * 3 + static_cast<int>(ETransportationMode::Walk), 
                    CurrIndex * 3 + static_cast<int>(ETransportationMode::Walk), 
                    WalkTime, !IsOneWay
                );
                
                // Add biking edges
                DFastestPathRouter.AddEdge(
                    PrevIndex * 3 + static_cast<int>(ETransportationMode::Bike), 
                    CurrIndex * 3 + static_cast<int>(ETransportationMode::Bike), 
                    BikeTime, !IsOneWay
                );
                
                // Add bus edges
                DFastestPathRouter.AddEdge(
                    PrevIndex * 3 + static_cast<int>(ETransportationMode::Bus), 
                    CurrIndex * 3 + static_cast<int>(ETransportationMode::Bus), 
                    DriveTime, !IsOneWay
                );
            }
        }
        
        // Add mode switching edges at each node
        for(size_t i = 0; i < DNodes.size(); ++i) {
            // Walk -> Bike
            DFastestPathRouter.AddEdge(
                i * 3 + static_cast<int>(ETransportationMode::Walk), 
                i * 3 + static_cast<int>(ETransportationMode::Bike), 
                0.0, false
            );
            
            // Bike -> Walk
            DFastestPathRouter.AddEdge(
                i * 3 + static_cast<int>(ETransportationMode::Bike), 
                i * 3 + static_cast<int>(ETransportationMode::Walk), 
                0.0, false
            );
            
            // Walk -> Bus (only at bus stops, with wait time)
            if(DIndexToBusStop.count(i)) {
                DFastestPathRouter.AddEdge(
                    i * 3 + static_cast<int>(ETransportationMode::Walk), 
                    i * 3 + static_cast<int>(ETransportationMode::Bus), 
                    DConfig->BusStopTime() / 3600.0, false
                );
                
                // Bus -> Walk
                DFastestPathRouter.AddEdge(
                    i * 3 + static_cast<int>(ETransportationMode::Bus), 
                    i * 3 + static_cast<int>(ETransportationMode::Walk), 
                    0.0, false
                );
            }
        }
        
        // Add direct bus route connections
        auto BusSystem = DConfig->BusSystem();
        
        for(const auto& Route : DBusRoutes) {
            const auto& Stops = Route.second;
            
            for(size_t i = 0; i < Stops.size() - 1; ++i) {
                size_t SrcIndex = Stops[i];
                
                for(size_t j = i + 1; j < Stops.size(); ++j) {
                    size_t DestIndex = Stops[j];
                    
                    // Calculate distance between these stops
                    std::vector<TNodeID> ShortestPathBetweenStops;
                    double Distance = DShortestPathRouter.FindShortestPath(SrcIndex, DestIndex, ShortestPathBetweenStops);
                    
                    if(Distance != CPathRouter::NoPathExists) {
                        // Bus time + waiting time for intermediate stops
                        double BusTime = Distance / DConfig->DefaultSpeedLimit() + 
                                        (j - i - 1) * (DConfig->BusStopTime() / 3600.0);
                        
                        // Add a direct bus edge
                        DFastestPathRouter.AddEdge(
                            SrcIndex * 3 + static_cast<int>(ETransportationMode::Bus), 
                            DestIndex * 3 + static_cast<int>(ETransportationMode::Bus), 
                            BusTime, false
                        );
                    }
                }
            }
        }
    }
    
    void PrecomputePaths() {
        auto DeadlineTime = std::chrono::steady_clock::now() + 
                           std::chrono::seconds(DConfig->PrecomputeTime());
        
        DShortestPathRouter.Precompute(DeadlineTime);
        DFastestPathRouter.Precompute(DeadlineTime);
    }
    
    std::size_t NodeCount() const {
        return DNodes.size();
    }
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const {
        if(index < DNodes.size()) {
            return DNodes[index];
        }
        return nullptr;
    }
    
    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        path.clear();
        
        if(DNodeIDToIndex.count(src) == 0 || DNodeIDToIndex.count(dest) == 0) {
            return CPathRouter::NoPathExists;
        }
        
        size_t SrcIndex = DNodeIDToIndex.at(src);
        size_t DestIndex = DNodeIDToIndex.at(dest);
        
        std::vector<CPathRouter::TVertexID> RouterPath;
        double Distance = DShortestPathRouter.FindShortestPath(SrcIndex, DestIndex, RouterPath);
        
        if(Distance == CPathRouter::NoPathExists) {
            return CPathRouter::NoPathExists;
        }
        
        for(auto VertexID : RouterPath) {
            TNodeID NodeID = DNodes[VertexID]->ID();
            path.push_back(NodeID);
        }
        
        return Distance;
    }
    
    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();
        
        if(DNodeIDToIndex.count(src) == 0 || DNodeIDToIndex.count(dest) == 0) {
            return CPathRouter::NoPathExists;
        }
        
        size_t SrcIndex = DNodeIDToIndex.at(src);
        size_t DestIndex = DNodeIDToIndex.at(dest);
        
        // Try all modes as starting modes
        double BestTime = std::numeric_limits<double>::infinity();
        std::vector<CPathRouter::TVertexID> BestPath;
        
        for(int StartMode = 0; StartMode < 3; ++StartMode) {
            for(int EndMode = 0; EndMode < 3; ++EndMode) {
                std::vector<CPathRouter::TVertexID> CurrentPath;
                double Time = DFastestPathRouter.FindShortestPath(
                    SrcIndex * 3 + StartMode, 
                    DestIndex * 3 + EndMode, 
                    CurrentPath
                );
                
                if(Time != CPathRouter::NoPathExists && Time < BestTime) {
                    BestTime = Time;
                    BestPath = CurrentPath;
                }
            }
        }
        
        if(BestTime == std::numeric_limits<double>::infinity()) {
            return CPathRouter::NoPathExists;
        }
        
        // Convert router path to trip steps, consolidating segments with same mode
        ETransportationMode CurrentMode = static_cast<ETransportationMode>(BestPath[0] % 3);
        TNodeID CurrentNodeID = DNodes[BestPath[0] / 3]->ID();
        path.push_back({CurrentMode, CurrentNodeID});
        
        for(size_t i = 1; i < BestPath.size(); ++i) {
            ETransportationMode NewMode = static_cast<ETransportationMode>(BestPath[i] % 3);
            TNodeID NewNodeID = DNodes[BestPath[i] / 3]->ID();
            
            if(NewMode != CurrentMode) {
                CurrentMode = NewMode;
                path.push_back({CurrentMode, NewNodeID});
            }
            else if(i == BestPath.size() - 1) { // Always include the destination
                path.push_back({CurrentMode, NewNodeID});
            }
        }
        
        return BestTime;
    }
    
    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();
        
        if(path.empty()) {
            return false;
        }
        
        auto StreetMap = DConfig->StreetMap();
        auto BusSystem = DConfig->BusSystem();
        
        // Start location
        TNodeID FirstNodeID = path[0].second;
        auto FirstNode = StreetMap->NodeByID(FirstNodeID);
        auto StartLocation = FirstNode->Location();
        desc.push_back("Start at " + SGeographicUtils::ConvertLLToDMS(StartLocation));
        
        for(size_t i = 0; i < path.size() - 1; ++i) {
            auto CurrentStep = path[i];
            auto NextStep = path[i+1];
            
            TNodeID CurrentNodeID = CurrentStep.second;
            TNodeID NextNodeID = NextStep.second;
            
            auto CurrentNode = StreetMap->NodeByID(CurrentNodeID);
            auto NextNode = StreetMap->NodeByID(NextNodeID);
            
            auto CurrentLocation = CurrentNode->Location();
            auto NextLocation = NextNode->Location();
            
            ETransportationMode CurrentMode = CurrentStep.first;
            ETransportationMode NextMode = NextStep.first;
            
            // Handle bus mode
            if(CurrentMode == ETransportationMode::Bus && NextMode == ETransportationMode::Bus) {
                // Find bus stop IDs
                auto CurrentStopID = DIndexToBusStop.at(DNodeIDToIndex.at(CurrentNodeID));
                auto NextStopID = DIndexToBusStop.at(DNodeIDToIndex.at(NextNodeID));
                
                // Find which bus route connects these stops
                std::string BusRoute;
                for(const auto& Route : DBusRoutes) {
                    const auto& Stops = Route.second;
                    auto CurrentStopIt = std::find(Stops.begin(), Stops.end(), DNodeIDToIndex.at(CurrentNodeID));
                    auto NextStopIt = std::find(Stops.begin(), Stops.end(), DNodeIDToIndex.at(NextNodeID));
                    
                    if(CurrentStopIt != Stops.end() && NextStopIt != Stops.end() && 
                       std::distance(CurrentStopIt, NextStopIt) > 0) {
                        BusRoute = Route.first;
                        break;
                    }
                }
                
                desc.push_back("Take Bus " + BusRoute + " from stop " + std::to_string(CurrentStopID) + 
                               " to stop " + std::to_string(NextStopID));
            }
            // Handle walk/bike mode
            else if(CurrentMode == NextMode && (CurrentMode == ETransportationMode::Walk || CurrentMode == ETransportationMode::Bike)) {
                double Distance = SGeographicUtils::HaversineDistanceInMiles(CurrentLocation, NextLocation);
                double Bearing = SGeographicUtils::CalculateBearing(CurrentLocation, NextLocation);
                std::string Direction = SGeographicUtils::BearingToDirection(Bearing);
                // Find street name
                std::string StreetName;
                size_t CurrentIndex = DNodeIDToIndex.at(CurrentNodeID);
                size_t NextIndex = DNodeIDToIndex.at(NextNodeID);
                
                if(DEdgeStreetNames.count(CurrentIndex)) {
                    for(const auto& Edge : DEdgeStreetNames.at(CurrentIndex)) {
                        if(Edge.first == NextIndex) {
                            StreetName = Edge.second;
                            break;
                        }
                    }
                }
                
                std::string ModeStr = (CurrentMode == ETransportationMode::Walk) ? "Walk" : "Bike";
                
                if(!StreetName.empty()) {
                    desc.push_back(ModeStr + " " + Direction + " along " + StreetName + 
                                   " for " + std::to_string(static_cast<int>(Distance * 10) / 10.0) + " mi");
                }
                else {
                    desc.push_back(ModeStr + " " + Direction + " toward End for " + 
                                   std::to_string(static_cast<int>(Distance * 10) / 10.0) + " mi");
                }
            }
        }
        
        // End location
        TNodeID LastNodeID = path.back().second;
        auto LastNode = StreetMap->NodeByID(LastNodeID);
        auto EndLocation = LastNode->Location();
        desc.push_back("End at " + SGeographicUtils::ConvertLLToDMS(EndLocation));
        
        return true;
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() = default;

std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->NodeCount();
}

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedNodeByIndex(index);
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