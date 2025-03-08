#include "DijkstraTransportationPlanner.h"
#include "GeographicUtils.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <sstream>
#include <iomanip>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<STransportationPlannerConfig> DConfig;
    std::vector<TNodeID> DNodeIDToSortedIndex;
    std::vector<std::shared_ptr<CStreetMap::SNode>> DSortedNodes;
    
    // Path routers for each transportation mode
    std::shared_ptr<CPathRouter> DWalkRouter;
    std::shared_ptr<CPathRouter> DBikeRouter;
    std::shared_ptr<CPathRouter> DBusRouter;
    
    SImplementation(std::shared_ptr<STransportationPlannerConfig> config) 
        : DConfig(config), DWalkRouter(std::make_shared<CPathRouter>()), 
          DBikeRouter(std::make_shared<CPathRouter>()), DBusRouter(std::make_shared<CPathRouter>()) {
        
        // Extract and sort all nodes from street map
        auto StreetMap = DConfig->StreetMap();
        std::vector<TNodeID> NodeIDs;
        
        for (size_t i = 0; i < StreetMap->NodeCount(); ++i) {
            auto Node = StreetMap->NodeByIndex(i);
            if (Node) {
                NodeIDs.push_back(Node->ID());
            }
        }
        
        // Sort node IDs
        std::sort(NodeIDs.begin(), NodeIDs.end());
        
        // Create mapping from TNodeID to index
        DNodeIDToSortedIndex.resize(*std::max_element(NodeIDs.begin(), NodeIDs.end()) + 1, std::numeric_limits<size_t>::max());
        DSortedNodes.resize(NodeIDs.size());
        
        for (size_t i = 0; i < NodeIDs.size(); ++i) {
            auto NodeID = NodeIDs[i];
            DNodeIDToSortedIndex[NodeID] = i;
            DSortedNodes[i] = StreetMap->NodeByID(NodeID);
        }
        
        // Build the routing graphs based on transportation mode
        BuildWalkingGraph();
        BuildBikingGraph();
        BuildBusGraph();
    }
    
    void BuildWalkingGraph() {
        auto StreetMap = DConfig->StreetMap();
        
        // Create vertices for all nodes
        for (size_t i = 0; i < DSortedNodes.size(); ++i) {
            DWalkRouter->AddVertex(i);
        }
        
        // Add edges between nodes on ways
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto Way = StreetMap->WayByIndex(i);
            if (!Way) continue;
            
            auto NodeIDs = Way->NodeIDs();
            for (size_t j = 1; j < NodeIDs.size(); ++j) {
                auto FromNodeID = NodeIDs[j-1];
                auto ToNodeID = NodeIDs[j];
                
                auto FromNode = StreetMap->NodeByID(FromNodeID);
                auto ToNode = StreetMap->NodeByID(ToNodeID);
                if (!FromNode || !ToNode) continue;
                
                double Distance = SGeographicUtils::HaversineDistanceInMiles(
                    std::make_pair(FromNode->Location().first, FromNode->Location().second),
                    std::make_pair(ToNode->Location().first, ToNode->Location().second)
                );
                
                auto FromIndex = DNodeIDToSortedIndex[FromNodeID];
                auto ToIndex = DNodeIDToSortedIndex[ToNodeID];
                
                // For walking, we can walk both ways regardless of oneway tag
                DWalkRouter->AddEdge(FromIndex, ToIndex, Distance);
                DWalkRouter->AddEdge(ToIndex, FromIndex, Distance);
            }
        }
    }
    
    void BuildBikingGraph() {
        auto StreetMap = DConfig->StreetMap();
        
        // Create vertices for all nodes
        for (size_t i = 0; i < DSortedNodes.size(); ++i) {
            DBikeRouter->AddVertex(i);
        }
        
        // Add edges between nodes on ways
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto Way = StreetMap->WayByIndex(i);
            if (!Way) continue;
            
            auto NodeIDs = Way->NodeIDs();
            bool Oneway = false;
            
            // Check if the way is one-way
            for (const auto &Tag : Way->Tags()) {
                if (Tag.first == "oneway" && Tag.second == "yes") {
                    Oneway = true;
                    break;
                }
            }
            
            for (size_t j = 1; j < NodeIDs.size(); ++j) {
                auto FromNodeID = NodeIDs[j-1];
                auto ToNodeID = NodeIDs[j];
                
                auto FromNode = StreetMap->NodeByID(FromNodeID);
                auto ToNode = StreetMap->NodeByID(ToNodeID);
                if (!FromNode || !ToNode) continue;
                
                double Distance = SGeographicUtils::HaversineDistanceInMiles(
                    std::make_pair(FromNode->Location().first, FromNode->Location().second),
                    std::make_pair(ToNode->Location().first, ToNode->Location().second)
                );
                
                auto FromIndex = DNodeIDToSortedIndex[FromNodeID];
                auto ToIndex = DNodeIDToSortedIndex[ToNodeID];
                
                // Add the forward edge
                DBikeRouter->AddEdge(FromIndex, ToIndex, Distance);
                
                // Add the reverse edge only if not a one-way
                if (!Oneway) {
                    DBikeRouter->AddEdge(ToIndex, FromIndex, Distance);
                }
            }
        }
    }
    
    void BuildBusGraph() {
        auto StreetMap = DConfig->StreetMap();
        auto BusSystem = DConfig->BusSystem();
        
        // Create vertices for all nodes
        for (size_t i = 0; i < DSortedNodes.size(); ++i) {
            DBusRouter->AddVertex(i);
        }
        
        // Map from stop ID to node ID
        std::unordered_map<CBusSystem::TStopID, TNodeID> StopToNode;
        for (size_t i = 0; i < BusSystem->StopCount(); ++i) {
            auto Stop = BusSystem->StopByIndex(i);
            if (Stop) {
                StopToNode[Stop->ID()] = Stop->NodeID();
            }
        }
        
        // Create bus edges for each route
        for (size_t i = 0; i < BusSystem->RouteCount(); ++i) {
            auto Route = BusSystem->RouteByIndex(i);
            if (!Route) continue;
            
            auto StopIDs = Route->StopIDs();
            for (size_t j = 1; j < StopIDs.size(); ++j) {
                auto FromStopID = StopIDs[j-1];
                auto ToStopID = StopIDs[j];
                
                // Find corresponding nodes
                auto FromNodeIDIter = StopToNode.find(FromStopID);
                auto ToNodeIDIter = StopToNode.find(ToStopID);
                
                if (FromNodeIDIter == StopToNode.end() || ToNodeIDIter == StopToNode.end()) {
                    continue;
                }
                
                auto FromNodeID = FromNodeIDIter->second;
                auto ToNodeID = ToNodeIDIter->second;
                
                auto FromNode = StreetMap->NodeByID(FromNodeID);
                auto ToNode = StreetMap->NodeByID(ToNodeID);
                if (!FromNode || !ToNode) continue;
                
                // Calculate distance between stops
                double Distance = SGeographicUtils::HaversineDistanceInMiles(
                    std::make_pair(FromNode->Location().first, FromNode->Location().second),
                    std::make_pair(ToNode->Location().first, ToNode->Location().second)
                );
                
                auto FromIndex = DNodeIDToSortedIndex[FromNodeID];
                auto ToIndex = DNodeIDToSortedIndex[ToNodeID];
                
                // Add edge with weight based on distance and bus speed
                double Time = Distance / DConfig->DefaultSpeedLimit() + DConfig->BusStopTime() / 3600.0;
                DBusRouter->AddEdge(FromIndex, ToIndex, Time);
            }
        }
    }
    
    std::string FormatLatitude(double latitude) const {
        int degrees = static_cast<int>(latitude);
        double minutes_decimal = (latitude - degrees) * 60.0;
        int minutes = static_cast<int>(minutes_decimal);
        int seconds = static_cast<int>((minutes_decimal - minutes) * 60.0);
        
        std::ostringstream oss;
        oss << degrees << "d " << minutes << "' " << seconds << "\" ";
        oss << (latitude >= 0.0 ? "N" : "S");
        return oss.str();
    }
    
    std::string FormatLongitude(double longitude) const {
        int degrees = static_cast<int>(std::abs(longitude));
        double minutes_decimal = (std::abs(longitude) - degrees) * 60.0;
        int minutes = static_cast<int>(minutes_decimal);
        int seconds = static_cast<int>((minutes_decimal - minutes) * 60.0);
        
        std::ostringstream oss;
        oss << degrees << "d " << minutes << "' " << seconds << "\" ";
        oss << (longitude >= 0.0 ? "E" : "W");
        return oss.str();
    }
    
    std::string GetDirection(double lat1, double lon1, double lat2, double lon2) const {
        double dx = lon2 - lon1;
        double dy = lat2 - lat1;
        double angle = std::atan2(dy, dx) * 180.0 / M_PI;
        
        if (angle < 0) {
            angle += 360.0;
        }
        
        if (angle >= 337.5 || angle < 22.5) return "E";
        if (angle >= 22.5 && angle < 67.5) return "NE";
        if (angle >= 67.5 && angle < 112.5) return "N";
        if (angle >= 112.5 && angle < 157.5) return "NW";
        if (angle >= 157.5 && angle < 202.5) return "W";
        if (angle >= 202.5 && angle < 247.5) return "SW";
        if (angle >= 247.5 && angle < 292.5) return "S";
        return "SE";
    }
    
    std::string GetRoadName(TNodeID from, TNodeID to) const {
        auto StreetMap = DConfig->StreetMap();
        
        for (size_t i = 0; i < StreetMap->WayCount(); ++i) {
            auto Way = StreetMap->WayByIndex(i);
            if (!Way) continue;
            
            auto NodeIDs = Way->NodeIDs();
            for (size_t j = 1; j < NodeIDs.size(); ++j) {
                if ((NodeIDs[j-1] == from && NodeIDs[j] == to) || 
                    (NodeIDs[j-1] == to && NodeIDs[j] == from)) {
                    
                    // Find name tag
                    for (const auto &Tag : Way->Tags()) {
                        if (Tag.first == "name") {
                            return Tag.second;
                        }
                    }
                    break;
                }
            }
        }
        
        return "";
    }
};

// Constructor
CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<STransportationPlannerConfig> config)
    : DImplementation(std::make_shared<SImplementation>(config)) {
}

// Destructor
CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
}

// Return number of nodes
std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept {
    return DImplementation->DSortedNodes.size();
}

// Get sorted node by index
std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->DSortedNodes.size()) {
        return DImplementation->DSortedNodes[index];
    }
    return nullptr;
}

// Find shortest path
double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
    path.clear();
    
    // Convert node IDs to indices
    if (src >= DImplementation->DNodeIDToSortedIndex.size() || 
        dest >= DImplementation->DNodeIDToSortedIndex.size() ||
        DImplementation->DNodeIDToSortedIndex[src] == std::numeric_limits<size_t>::max() ||
        DImplementation->DNodeIDToSortedIndex[dest] == std::numeric_limits<size_t>::max()) {
        return CPathRouter::NoPathExists;
    }
    
    size_t SrcIndex = DImplementation->DNodeIDToSortedIndex[src];
    size_t DestIndex = DImplementation->DNodeIDToSortedIndex[dest];
    
    // Find shortest path using walking router (distance-based)
    std::vector<CPathRouter::TVertexID> RouterPath;
    double Distance = DImplementation->DWalkRouter->FindShortestPath(SrcIndex, DestIndex, RouterPath);
    
    if (Distance == CPathRouter::NoPathExists) {
        return CPathRouter::NoPathExists;
    }
    
    // Convert router path back to node IDs
    for (auto VertexID : RouterPath) {
        path.push_back(DImplementation->DSortedNodes[VertexID]->ID());
    }
    
    return Distance;
}

// Find fastest path
double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
    path.clear();
    
    // Convert node IDs to indices
    if (src >= DImplementation->DNodeIDToSortedIndex.size() || 
        dest >= DImplementation->DNodeIDToSortedIndex.size() ||
        DImplementation->DNodeIDToSortedIndex[src] == std::numeric_limits<size_t>::max() ||
        DImplementation->DNodeIDToSortedIndex[dest] == std::numeric_limits<size_t>::max()) {
        return CPathRouter::NoPathExists;
    }
    
    size_t SrcIndex = DImplementation->DNodeIDToSortedIndex[src];
    size_t DestIndex = DImplementation->DNodeIDToSortedIndex[dest];
    
    // Try walk path
    std::vector<CPathRouter::TVertexID> WalkPath;
    double WalkDistance = DImplementation->DWalkRouter->FindShortestPath(SrcIndex, DestIndex, WalkPath);
    double WalkTime = (WalkDistance != CPathRouter::NoPathExists) ? 
                       WalkDistance / DImplementation->DConfig->WalkSpeed() : 
                       std::numeric_limits<double>::max();
    
    // Try bike path
    std::vector<CPathRouter::TVertexID> BikePath;
    double BikeDistance = DImplementation->DBikeRouter->FindShortestPath(SrcIndex, DestIndex, BikePath);
    double BikeTime = (BikeDistance != CPathRouter::NoPathExists) ? 
                       BikeDistance / DImplementation->DConfig->BikeSpeed() : 
                       std::numeric_limits<double>::max();
    
    // Try bus path (already time-based weights)
    std::vector<CPathRouter::TVertexID> BusPath;
    double BusTime = DImplementation->DBusRouter->FindShortestPath(SrcIndex, DestIndex, BusPath);
    
    // Find the fastest mode
    double FastestTime = std::min({WalkTime, BikeTime, BusTime});
    
    if (FastestTime == std::numeric_limits<double>::max()) {
        return CPathRouter::NoPathExists;
    }
    
    // Build the path based on the fastest mode
    if (FastestTime == WalkTime) {
        for (auto VertexID : WalkPath) {
            path.push_back({ETransportationMode::Walk, DImplementation->DSortedNodes[VertexID]->ID()});
        }
        return WalkTime;
    } 
    else if (FastestTime == BikeTime) {
        for (auto VertexID : BikePath) {
            path.push_back({ETransportationMode::Bike, DImplementation->DSortedNodes[VertexID]->ID()});
        }
        return BikeTime;
    } 
    else {
        // Bus path
        for (auto VertexID : BusPath) {
            path.push_back({ETransportationMode::Bus, DImplementation->DSortedNodes[VertexID]->ID()});
        }
        return BusTime;
    }
}

// Get path description
bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
    desc.clear();
    
    if (path.empty()) {
        return false;
    }
    
    auto StreetMap = DImplementation->DConfig->StreetMap();
    auto BusSystem = DImplementation->DConfig->BusSystem();
    
    // First step is the starting point
    auto FirstNode = StreetMap->NodeByID(path[0].second);
    if (!FirstNode) {
        return false;
    }
    
    double FirstLat = FirstNode->Location().first;
    double FirstLon = FirstNode->Location().second;
    
    std::ostringstream StartDesc;
    StartDesc << "Start at " << DImplementation->FormatLatitude(FirstLat) << ", " 
              << DImplementation->FormatLongitude(FirstLon);
    desc.push_back(StartDesc.str());
    
    // Process each segment
    ETransportationMode CurrentMode = path[0].first;
    TNodeID CurrentStop = path[0].second;
    TNodeID SegmentStart = path[0].second;
    std::size_t SegmentStartIdx = 0;
    std::string CurrentRoute;
    
    std::unordered_map<TNodeID, CBusSystem::TStopID> NodeToStop;
    for (size_t i = 0; i < BusSystem->StopCount(); ++i) {
        auto Stop = BusSystem->StopByIndex(i);
        if (Stop) {
            NodeToStop[Stop->NodeID()] = Stop->ID();
        }
    }
    
    for (size_t i = 1; i < path.size(); ++i) {
        ETransportationMode NextMode = path[i].first;
        TNodeID NextStop = path[i].second;
        
        // Mode change or last segment
        if (NextMode != CurrentMode || i == path.size() - 1) {
            // Process completed segment
            auto StartNode = StreetMap->NodeByID(SegmentStart);
            auto EndNode = StreetMap->NodeByID(path[i-1].second);
            
            if (!StartNode || !EndNode) {
                continue;
            }
            
            double StartLat = StartNode->Location().first;
            double StartLon = StartNode->Location().second;
            double EndLat = EndNode->Location().first;
            double EndLon = EndNode->Location().second;
            
            std::ostringstream SegmentDesc;
            
            if (CurrentMode == ETransportationMode::Bus) {
                // Find bus route
                for (size_t r = 0; r < BusSystem->RouteCount(); ++r) {
                    auto Route = BusSystem->RouteByIndex(r);
                    if (!Route) continue;
                    
                    auto StopIDs = Route->StopIDs();
                    bool FoundStart = false;
                    bool FoundEnd = false;
                    
                    for (auto StopID : StopIDs) {
                        auto Stop = BusSystem->StopByID(StopID);
                        if (!Stop) continue;
                        
                        if (Stop->NodeID() == SegmentStart) {
                            FoundStart = true;
                        }
                        if (Stop->NodeID() == path[i-1].second && FoundStart) {
                            FoundEnd = true;
                            break;
                        }
                    }
                    
                    if (FoundStart && FoundEnd) {
                        CurrentRoute = Route->Name();
                        break;
                    }
                }
                
                auto StartStopIter = NodeToStop.find(SegmentStart);
                auto EndStopIter = NodeToStop.find(path[i-1].second);
                
                if (StartStopIter != NodeToStop.end() && EndStopIter != NodeToStop.end()) {
                    SegmentDesc << "Take Bus " << CurrentRoute << " from stop " 
                                << StartStopIter->second << " to stop " << EndStopIter->second;
                }
            } else {
                // Walk or Bike segment
                std::string TransportMode = (CurrentMode == ETransportationMode::Walk) ? "Walk" : "Bike";
                std::string Direction = DImplementation->GetDirection(StartLat, StartLon, EndLat, EndLon);
                
                // Calculate total distance for this segment
                double TotalDistance = 0.0;
                for (size_t j = SegmentStartIdx; j < i - 1; ++j) {
                    auto FromNode = StreetMap->NodeByID(path[j].second);
                    auto ToNode = StreetMap->NodeByID(path[j+1].second);
                    
                    if (FromNode && ToNode) {
                        TotalDistance += SGeographicUtils::HaversineDistanceInMiles(
                            std::make_pair(FromNode->Location().first, FromNode->Location().second),
                            std::make_pair(ToNode->Location().first, ToNode->Location().second)
                        );
                    }
                }
                
                // Get road name if available
                std::string RoadName = DImplementation->GetRoadName(SegmentStart, path[i-1].second);
                
                SegmentDesc << TransportMode << " " << Direction;
                if (!RoadName.empty()) {
                    SegmentDesc << " along " << RoadName;
                } else {
                    SegmentDesc << " toward End";
                }
                SegmentDesc << " for " << std::fixed << std::setprecision(1) << TotalDistance << " mi";
            }
            
            desc.push_back(SegmentDesc.str());
            
            // Start new segment
            SegmentStart = path[i-1].second;
            SegmentStartIdx = i-1;
        }
        
        CurrentMode = NextMode;
        CurrentStop = NextStop;
    }
    
    // Add the final node as the end point
    auto LastNode = StreetMap->NodeByID(path.back().second);
    if (LastNode) {
        double LastLat = LastNode->Location().first;
        double LastLon = LastNode->Location().second;
        
        std::ostringstream EndDesc;
        EndDesc << "End at " << DImplementation->FormatLatitude(LastLat) << ", " 
                << DImplementation->FormatLongitude(LastLon);
        desc.push_back(EndDesc.str());
    }
    
    return true;
}