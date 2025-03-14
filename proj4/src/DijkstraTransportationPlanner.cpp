#include "DijkstraTransportationPlanner.h"
#include "GeographicUtils.h"
#include <limits>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<STransportationPlannerConfig> DConfig;
    CDijkstraPathRouter DPathRouter;
    std::unordered_map<TNodeID, TVertexID> DNodeToShortestVertex;
    std::unordered_map<TVertexID, TNodeID> DShortestVertexToNode;
    std::unordered_map<TNodeID, TVertexID> DNodeToWalkingVertex;
    std::unordered_map<TVertexID, TNodeID> DWalkingVertexToNode;
    std::unordered_map<TNodeID, TVertexID> DNodeToBikingVertex;
    std::unordered_map<TVertexID, TNodeID> DBikingVertexToNode;
    std::unordered_map<TNodeID, TVertexID> DNodeToBusVertex;
    std::unordered_map<TVertexID, TNodeID> DBusVertexToNode;
    std::unordered_map<std::size_t, TStopID> DVertexToStop;
    std::unordered_map<TStopID, std::unordered_map<TStopID, std::string>> DBusRoutes;

    SImplementation(std::shared_ptr<STransportationPlannerConfig> config) : DConfig(config) {
        BuildGraphs();
    }

    void BuildGraphs() {
        // Build vertices for each transportation mode
        auto StreetMap = DConfig->StreetMap();
        auto BusSystem = DConfig->BusSystem();

        // Create vertices for all nodes in the map
        for (std::size_t i = 0; i < StreetMap->NodeCount(); ++i) {
            auto Node = StreetMap->GetSortedNodeByIndex(i);
            TNodeID NodeID = Node->ID();

            // Create vertices for different transportation modes
            TVertexID ShortestVertex = DPathRouter.AddVertex(NodeID);
            TVertexID WalkingVertex = DPathRouter.AddVertex(NodeID);
            TVertexID BikingVertex = DPathRouter.AddVertex(NodeID);
            TVertexID BusVertex = DPathRouter.AddVertex(NodeID);

            // Map Node IDs to Vertex IDs
            DNodeToShortestVertex[NodeID] = ShortestVertex;
            DShortestVertexToNode[ShortestVertex] = NodeID;
            DNodeToWalkingVertex[NodeID] = WalkingVertex;
            DWalkingVertexToNode[WalkingVertex] = NodeID;
            DNodeToBikingVertex[NodeID] = BikingVertex;
            DBikingVertexToNode[BikingVertex] = NodeID;
            DNodeToBusVertex[NodeID] = BusVertex;
            DBusVertexToNode[BusVertex] = NodeID;
        }

        // Add edges for the street map
        for (std::size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto Way = StreetMap->GetWayByIndex(i);
            bool IsOneWay = false;

            // Check if the way is one-way
            for (std::size_t j = 0; j < Way->AttributeCount(); ++j) {
                if (Way->GetAttributeKey(j) == "oneway" && Way->GetAttributeValue(j) == "yes") {
                    IsOneWay = true;
                    break;
                }
            }

            // Get max speed if available
            double MaxSpeedMPH = 25.0; // Default speed
            for (std::size_t j = 0; j < Way->AttributeCount(); ++j) {
                if (Way->GetAttributeKey(j) == "maxspeed") {
                    std::string MaxSpeedStr = Way->GetAttributeValue(j);
                    // Parse maxspeed value (e.g. "20 mph")
                    std::istringstream iss(MaxSpeedStr);
                    iss >> MaxSpeedMPH;
                    break;
                }
            }

            // Add edges between each pair of nodes in the way
            for (std::size_t j = 0; j < Way->NodeCount() - 1; ++j) {
                TNodeID SrcID = Way->GetNodeID(j);
                TNodeID DestID = Way->GetNodeID(j + 1);

                auto SrcNode = StreetMap->FindNodeByID(SrcID);
                auto DestNode = StreetMap->FindNodeByID(DestID);

                if (SrcNode && DestNode) {
                    CStreetMap::TLocation SrcLoc = std::make_pair(SrcNode->Latitude(), SrcNode->Longitude());
                    CStreetMap::TLocation DestLoc = std::make_pair(DestNode->Latitude(), DestNode->Longitude());
                    double Distance = SGeographicUtils::HaversineDistanceInMiles(SrcLoc, DestLoc);

                    // Add edges for shortest path (just distance)
                    DPathRouter.AddEdge(DNodeToShortestVertex[SrcID], DNodeToShortestVertex[DestID], 
                                        Distance, !IsOneWay);

                    // Add edges for walking (Speed: 3 mph)
                    double WalkingTime = Distance / 3.0;
                    DPathRouter.AddEdge(DNodeToWalkingVertex[SrcID], DNodeToWalkingVertex[DestID], 
                                        WalkingTime, !IsOneWay);

                    // Add edges for biking (Speed: 8 mph)
                    double BikingTime = Distance / 8.0;
                    DPathRouter.AddEdge(DNodeToBikingVertex[SrcID], DNodeToBikingVertex[DestID], 
                                        BikingTime, !IsOneWay);

                    // Add edges for driving/bus (Speed: MaxSpeedMPH)
                    double DrivingTime = Distance / MaxSpeedMPH;
                    DPathRouter.AddEdge(DNodeToBusVertex[SrcID], DNodeToBusVertex[DestID], 
                                        DrivingTime, !IsOneWay);
                }
            }
        }

        // Process bus stops and routes
        for (std::size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto Stop = BusSystem->GetStopByIndex(i);
            TStopID StopID = Stop->ID();
            TNodeID NodeID = Stop->NodeID();
            
            // Keep track of stops in relation to vertices
            DVertexToStop[DNodeToBusVertex[NodeID]] = StopID;
        }

        // Process bus routes
        for (std::size_t i = 0; i < BusSystem->RouteCount(); ++i) {
            auto Route = BusSystem->GetRouteByIndex(i);
            std::string RouteName = Route->Name();
            
            // Add connections between consecutive stops in the route
            for (std::size_t j = 0; j < Route->StopCount() - 1; ++j) {
                TStopID SrcStopID = Route->GetStopID(j);
                TStopID DestStopID = Route->GetStopID(j + 1);
                
                // Store the route information
                DBusRoutes[SrcStopID][DestStopID] = RouteName;
                
                // Add bus transfer edges between transport mode vertices
                auto SrcStop = BusSystem->FindStopByID(SrcStopID);
                auto DestStop = BusSystem->FindStopByID(DestStopID);
                
                if (SrcStop && DestStop) {
                    TNodeID SrcNodeID = SrcStop->NodeID();
                    TNodeID DestNodeID = DestStop->NodeID();
                    
                    // Add an edge with the bus waiting time (1 minute = 1/60 hour)
                    DPathRouter.AddEdge(DNodeToBusVertex[SrcNodeID], DNodeToBusVertex[DestNodeID], 
                                      1.0/60.0, false);
                }
            }
        }

        // Add transfer edges between different transportation modes at the same node
        for (std::size_t i = 0; i < StreetMap->NodeCount(); ++i) {
            auto Node = StreetMap->GetSortedNodeByIndex(i);
            TNodeID NodeID = Node->ID();
            
            // Transfers between walking and biking (no time cost)
            DPathRouter.AddEdge(DNodeToWalkingVertex[NodeID], DNodeToBikingVertex[NodeID], 0.0, true);
            
            // Transfer to/from bus (no time cost)
            DPathRouter.AddEdge(DNodeToWalkingVertex[NodeID], DNodeToBusVertex[NodeID], 0.0, true);
            DPathRouter.AddEdge(DNodeToBikingVertex[NodeID], DNodeToBusVertex[NodeID], 0.0, true);
        }
    }

    std::size_t NodeCount() const {
        return DConfig->StreetMap()->NodeCount();
    }

    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const {
        if (index < NodeCount()) {
            return DConfig->StreetMap()->GetSortedNodeByIndex(index);
        }
        return nullptr;
    }

    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        auto SourceVertex = DNodeToShortestVertex.find(src);
        auto DestVertex = DNodeToShortestVertex.find(dest);
        
        if (SourceVertex == DNodeToShortestVertex.end() || DestVertex == DNodeToShortestVertex.end()) {
            return CPathRouter::NoPathExists;
        }

        std::vector<CPathRouter::TVertexID> RouterPath;
        double Distance = DPathRouter.FindShortestPath(SourceVertex->second, DestVertex->second, RouterPath);
        
        if (Distance == CPathRouter::NoPathExists) {
            return CPathRouter::NoPathExists;
        }

        // Convert vertex IDs back to node IDs
        path.clear();
        for (auto VertexID : RouterPath) {
            path.push_back(DShortestVertexToNode[VertexID]);
        }
        
        return Distance;
    }

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();

        // Find the fastest path considering all transportation modes
        double BestTime = std::numeric_limits<double>::infinity();
        std::vector<CPathRouter::TVertexID> BestPath;
        
        // Try walking path
        std::vector<CPathRouter::TVertexID> WalkingPath;
        auto SrcWalkVertex = DNodeToWalkingVertex.find(src);
        auto DestWalkVertex = DNodeToWalkingVertex.find(dest);
        
        if (SrcWalkVertex != DNodeToWalkingVertex.end() && DestWalkVertex != DNodeToWalkingVertex.end()) {
            double WalkingTime = DPathRouter.FindShortestPath(SrcWalkVertex->second, DestWalkVertex->second, WalkingPath);
            if (WalkingTime != CPathRouter::NoPathExists && WalkingTime < BestTime) {
                BestTime = WalkingTime;
                BestPath = WalkingPath;
            }
        }
        
        // Try biking path
        std::vector<CPathRouter::TVertexID> BikingPath;
        auto SrcBikeVertex = DNodeToBikingVertex.find(src);
        auto DestBikeVertex = DNodeToBikingVertex.find(dest);
        
        if (SrcBikeVertex != DNodeToBikingVertex.end() && DestBikeVertex != DNodeToBikingVertex.end()) {
            double BikingTime = DPathRouter.FindShortestPath(SrcBikeVertex->second, DestBikeVertex->second, BikingPath);
            if (BikingTime != CPathRouter::NoPathExists && BikingTime < BestTime) {
                BestTime = BikingTime;
                BestPath = BikingPath;
            }
        }
        
        // Try bus path
        std::vector<CPathRouter::TVertexID> BusPath;
        auto SrcBusVertex = DNodeToBusVertex.find(src);
        auto DestBusVertex = DNodeToBusVertex.find(dest);
        
        if (SrcBusVertex != DNodeToBusVertex.end() && DestBusVertex != DNodeToBusVertex.end()) {
            double BusTime = DPathRouter.FindShortestPath(SrcBusVertex->second, DestBusVertex->second, BusPath);
            if (BusTime != CPathRouter::NoPathExists && BusTime < BestTime) {
                BestTime = BusTime;
                BestPath = BusPath;
            }
        }
        
        if (BestTime == std::numeric_limits<double>::infinity()) {
            return CPathRouter::NoPathExists;
        }
        
        // Convert the path to TTripStep format
        // Analyze each vertex to determine its transportation mode
        ETransportationMode CurrentMode = ETransportationMode::Walk;
        for (std::size_t i = 0; i < BestPath.size(); ++i) {
            TVertexID CurrentVertex = BestPath[i];
            TNodeID CurrentNode;
            ETransportationMode NewMode;
            
            // Determine the mode of the current vertex
            if (DWalkingVertexToNode.find(CurrentVertex) != DWalkingVertexToNode.end()) {
                CurrentNode = DWalkingVertexToNode[CurrentVertex];
                NewMode = ETransportationMode::Walk;
            } else if (DBikingVertexToNode.find(CurrentVertex) != DBikingVertexToNode.end()) {
                CurrentNode = DBikingVertexToNode[CurrentVertex];
                NewMode = ETransportationMode::Bike;
            } else if (DBusVertexToNode.find(CurrentVertex) != DBusVertexToNode.end()) {
                CurrentNode = DBusVertexToNode[CurrentVertex];
                NewMode = ETransportationMode::Bus;
            } else {
                // This shouldn't happen with proper vertex mapping
                continue;
            }
            
            // Add new step or update mode if changes
            if (path.empty() || CurrentMode != NewMode) {
                path.push_back({NewMode, CurrentNode});
                CurrentMode = NewMode;
            } else if (!path.empty() && path.back().first == NewMode) {
                // Update the node ID for the existing mode
                path.back().second = CurrentNode;
            }
        }
        
        return BestTime;
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();
        
        if (path.empty()) {
            return false;
        }
        
        auto StreetMap = DConfig->StreetMap();
        auto BusSystem = DConfig->BusSystem();
        
        // Start location
        auto FirstNode = StreetMap->FindNodeByID(path[0].second);
        if (!FirstNode) {
            return false;
        }
        
        CStreetMap::TLocation StartLoc = std::make_pair(FirstNode->Latitude(), FirstNode->Longitude());
        desc.push_back("Start at " + SGeographicUtils::ConvertLLToDMS(StartLoc));
        
        // Process each step in the path
        for (std::size_t i = 0; i < path.size() - 1; ++i) {
            auto CurrentStep = path[i];
            auto NextStep = path[i + 1];
            
            auto CurrentNode = StreetMap->FindNodeByID(CurrentStep.second);
            auto NextNode = StreetMap->FindNodeByID(NextStep.second);
            
            if (!CurrentNode || !NextNode) {
                continue;
            }
            
            CStreetMap::TLocation CurrentLoc = std::make_pair(CurrentNode->Latitude(), CurrentNode->Longitude());
            CStreetMap::TLocation NextLoc = std::make_pair(NextNode->Latitude(), NextNode->Longitude());
            
            double Distance = SGeographicUtils::HaversineDistanceInMiles(CurrentLoc, NextLoc);
            double Bearing = SGeographicUtils::CalculateBearing(CurrentLoc, NextLoc);
            std::string Direction = SGeographicUtils::BearingToDirection(Bearing);
            
            // Handle different transportation modes
            if (CurrentStep.first == ETransportationMode::Bus && NextStep.first == ETransportationMode::Bus) {
                // Find the bus stops
                TStopID CurrentStopID = 0;
                TStopID NextStopID = 0;
                
                for (std::size_t j = 0; j < BusSystem->StopCount(); ++j) {
                    auto Stop = BusSystem->GetStopByIndex(j);
                    if (Stop->NodeID() == CurrentStep.second) {
                        CurrentStopID = Stop->ID();
                    }
                    if (Stop->NodeID() == NextStep.second) {
                        NextStopID = Stop->ID();
                    }
                }
                
                // Find the bus route
                std::string RouteName;
                if (DBusRoutes.count(CurrentStopID) && DBusRoutes.at(CurrentStopID).count(NextStopID)) {
                    RouteName = DBusRoutes.at(CurrentStopID).at(NextStopID);
                }
                
                if (!RouteName.empty()) {
                    desc.push_back("Take Bus " + RouteName + " from stop " + 
                                  std::to_string(CurrentStopID) + " to stop " + 
                                  std::to_string(NextStopID));
                }
            } else {
                // Walking or biking directions
                std::string ModeStr;
                if (CurrentStep.first == ETransportationMode::Walk) {
                    ModeStr = "Walk";
                } else if (CurrentStep.first == ETransportationMode::Bike) {
                    ModeStr = "Bike";
                } else {
                    continue;
                }
                
                // Find the street name if available
                std::string StreetName;
                for (std::size_t w = 0; w < StreetMap->WayCount(); ++w) {
                    auto Way = StreetMap->GetWayByIndex(w);
                    bool HasCurrentNode = false;
                    bool HasNextNode = false;
                    
                    for (std::size_t n = 0; n < Way->NodeCount(); ++n) {
                        if (Way->GetNodeID(n) == CurrentStep.second) {
                            HasCurrentNode = true;
                        }
                        if (Way->GetNodeID(n) == NextStep.second) {
                            HasNextNode = true;
                        }
                    }
                    
                    if (HasCurrentNode && HasNextNode) {
                        for (std::size_t a = 0; a < Way->AttributeCount(); ++a) {
                            if (Way->GetAttributeKey(a) == "name") {
                                StreetName = Way->GetAttributeValue(a);
                                break;
                            }
                        }
                        if (!StreetName.empty()) {
                            break;
                        }
                    }
                }
                
                std::string DirectionStr;
                if (i == path.size() - 2) { // Last step
                    DirectionStr = "toward End";
                } else if (!StreetName.empty()) {
                    DirectionStr = Direction + " along " + StreetName;
                } else {
                    DirectionStr = Direction + " toward " + "Next Point";
                }
                
                std::ostringstream oss;
                oss << ModeStr << " " << DirectionStr << " for " << std::fixed << std::setprecision(1) << Distance << " mi";
                desc.push_back(oss.str());
            }
        }
        
        // End location
        auto LastNode = StreetMap->FindNodeByID(path.back().second);
        if (!LastNode) {
            return false;
        }
        
        CStreetMap::TLocation EndLoc = std::make_pair(LastNode->Latitude(), LastNode->Longitude());
        desc.push_back("End at " + SGeographicUtils::ConvertLLToDMS(EndLoc));
        
        return true;
    }
};

// Constructor
CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<STransportationPlannerConfig> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

// Destructor
CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
}

// Returns the number of nodes in the street map
std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->NodeCount();
}

// Returns the street map node specified by index
std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    return DImplementation->SortedNodeByIndex(index);
}

// Find shortest path
double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    return DImplementation->FindShortestPath(src, dest, path);
}

// Find fastest path
double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    return DImplementation->FindFastestPath(src, dest, path);
}

// Get path description
bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    return DImplementation->GetPathDescription(path, desc);
}