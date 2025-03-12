#include "TransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <sstream>

class CTransportationPlanner::SImplementation {
public:
    struct SVertexInfo {
        TNodeID DNodeID;
        std::shared_ptr<CStreetMap::SNode> DNode;
    };

    std::shared_ptr<CStreetMap> DStreetMap;
    std::shared_ptr<CBusSystem> DBusSystem;
    double DWalkSpeed;
    double DBikeSpeed;
    double DDefaultSpeedLimit;
    double DBusStopTime;
    
    std::vector<std::shared_ptr<CStreetMap::SNode>> DSortedNodes;
    std::unordered_map<TNodeID, size_t> DNodeIDToIndex;
    
    // Path routers for different transportation modes
    std::shared_ptr<CDijkstraPathRouter> DWalkRouter;
    std::shared_ptr<CDijkstraPathRouter> DBikeRouter;
    std::shared_ptr<CDijkstraPathRouter> DBusRouter;
    
    // Maps to store vertex IDs for different routers
    std::unordered_map<TNodeID, CPathRouter::TVertexID> DNodeIDToWalkVertexID;
    std::unordered_map<CPathRouter::TVertexID, TNodeID> DWalkVertexIDToNodeID;
    
    std::unordered_map<TNodeID, CPathRouter::TVertexID> DNodeIDToBikeVertexID;
    std::unordered_map<CPathRouter::TVertexID, TNodeID> DBikeVertexIDToNodeID;
    
    std::unordered_map<TNodeID, CPathRouter::TVertexID> DNodeIDToBusVertexID;
    std::unordered_map<CPathRouter::TVertexID, TNodeID> DBusVertexIDToNodeID;
    
    // Store bus stop relationships
    std::unordered_map<TNodeID, std::vector<std::pair<TNodeID, std::string>>> DBusStopConnections;
    
    SImplementation(const SConfiguration &config) 
        : DStreetMap(config.StreetMap()), 
          DBusSystem(config.BusSystem()),
          DWalkSpeed(config.WalkSpeed()),
          DBikeSpeed(config.BikeSpeed()),
          DDefaultSpeedLimit(config.DefaultSpeedLimit()),
          DBusStopTime(config.BusStopTime()),
          DWalkRouter(std::make_shared<CDijkstraPathRouter>()),
          DBikeRouter(std::make_shared<CDijkstraPathRouter>()),
          DBusRouter(std::make_shared<CDijkstraPathRouter>()) {
        
        // Initialize data structures and build the path routers
        Initialize(config.PrecomputeTime());
    }
    
    void Initialize(int precomputeTime) {
        // Build sorted nodes list
        BuildSortedNodesList();
        
        // Create path routers for each transportation mode
        BuildWalkRouter();
        BuildBikeRouter();
        BuildBusRouter();
        
        // Precompute paths
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(precomputeTime);
        DWalkRouter->Precompute(deadline);
        DBikeRouter->Precompute(deadline);
        DBusRouter->Precompute(deadline);
    }
    
    void BuildSortedNodesList() {
        // Collect all nodes
        DSortedNodes.clear();
        DNodeIDToIndex.clear();
        
        for (size_t i = 0; i < DStreetMap->NodeCount(); ++i) {
            auto node = DStreetMap->NodeByIndex(i);
            if (node) {
                DSortedNodes.push_back(node);
            }
        }
        
        // Sort nodes by ID
        std::sort(DSortedNodes.begin(), DSortedNodes.end(), 
            [](const std::shared_ptr<CStreetMap::SNode> &a, const std::shared_ptr<CStreetMap::SNode> &b) {
                return a->ID() < b->ID();
            });
        
        // Build lookup map
        for (size_t i = 0; i < DSortedNodes.size(); ++i) {
            DNodeIDToIndex[DSortedNodes[i]->ID()] = i;
        }
    }
    
    void BuildWalkRouter() {
        DNodeIDToWalkVertexID.clear();
        DWalkVertexIDToNodeID.clear();
        
        // Add all nodes as vertices
        for (auto &node : DSortedNodes) {
            auto vertexID = DWalkRouter->AddVertex(node->ID());
            DNodeIDToWalkVertexID[node->ID()] = vertexID;
            DWalkVertexIDToNodeID[vertexID] = node->ID();
        }
        
        // Add edges based on ways
        for (size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way && way->NodeCount() >= 2) {
                // Check if way is walkable
                bool isFootway = false;
                
                for (size_t j = 0; j < way->AttributeCount(); ++j) {
                    std::string key = way->GetAttributeKey(j);
                    std::string value = way->GetAttribute(key);
                    
                    if ((key == "highway" && (value == "footway" || value == "path" || value == "pedestrian")) ||
                        (key == "foot" && value == "yes")) {
                        isFootway = true;
                        break;
                    }
                }
                
                // If way is not specifically for foot traffic, check if it's not a highway or motorway
                if (!isFootway) {
                    bool isWalkable = true;
                    
                    for (size_t j = 0; j < way->AttributeCount(); ++j) {
                        std::string key = way->GetAttributeKey(j);
                        std::string value = way->GetAttribute(key);
                        
                        if (key == "highway" && (value == "motorway" || value == "motorway_link")) {
                            isWalkable = false;
                            break;
                        }
                    }
                    
                    isFootway = isWalkable;
                }
                
                if (isFootway) {
                    // Add edges for walking
                    for (size_t j = 1; j < way->NodeCount(); ++j) {
                        TNodeID node1ID = way->GetNodeID(j - 1);
                        TNodeID node2ID = way->GetNodeID(j);
                        
                        if (DNodeIDToWalkVertexID.find(node1ID) != DNodeIDToWalkVertexID.end() &&
                            DNodeIDToWalkVertexID.find(node2ID) != DNodeIDToWalkVertexID.end()) {
                            
                            auto node1 = DStreetMap->NodeByID(node1ID);
                            auto node2 = DStreetMap->NodeByID(node2ID);
                            
                            if (node1 && node2) {
                                double distance = CalculateDistance(node1->Location(), node2->Location());
                                double walkTime = distance / DWalkSpeed;
                                
                                DWalkRouter->AddEdge(
                                    DNodeIDToWalkVertexID[node1ID], 
                                    DNodeIDToWalkVertexID[node2ID], 
                                    walkTime, 
                                    true);
                            }
                        }
                    }
                }
            }
        }
    }
    
    void BuildBikeRouter() {
        DNodeIDToBikeVertexID.clear();
        DBikeVertexIDToNodeID.clear();
        
        // Add all nodes as vertices
        for (auto &node : DSortedNodes) {
            auto vertexID = DBikeRouter->AddVertex(node->ID());
            DNodeIDToBikeVertexID[node->ID()] = vertexID;
            DBikeVertexIDToNodeID[vertexID] = node->ID();
        }
        
        // Add edges based on ways
        for (size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way && way->NodeCount() >= 2) {
                // Check if way is bikeable
                bool isBikeway = false;
                double speedLimit = DDefaultSpeedLimit;
                
                for (size_t j = 0; j < way->AttributeCount(); ++j) {
                    std::string key = way->GetAttributeKey(j);
                    std::string value = way->GetAttribute(key);
                    
                    if ((key == "highway" && (value == "cycleway" || value == "path")) ||
                        (key == "bicycle" && value == "yes")) {
                        isBikeway = true;
                    }
                    
                    if (key == "maxspeed") {
                        try {
                            speedLimit = std::stod(value);
                        } catch (...) {
                            // Use default speed if parsing fails
                        }
                    }
                }
                
                // If way is not specifically for bike traffic, check if it's not a highway or motorway
                if (!isBikeway) {
                    bool isBikeable = true;
                    
                    for (size_t j = 0; j < way->AttributeCount(); ++j) {
                        std::string key = way->GetAttributeKey(j);
                        std::string value = way->GetAttribute(key);
                        
                        if (key == "highway" && (value == "motorway" || value == "motorway_link")) {
                            isBikeable = false;
                            break;
                        }
                    }
                    
                    isBikeway = isBikeable;
                }
                
                if (isBikeway) {
                    // Add edges for biking
                    for (size_t j = 1; j < way->NodeCount(); ++j) {
                        TNodeID node1ID = way->GetNodeID(j - 1);
                        TNodeID node2ID = way->GetNodeID(j);
                        
                        if (DNodeIDToBikeVertexID.find(node1ID) != DNodeIDToBikeVertexID.end() &&
                            DNodeIDToBikeVertexID.find(node2ID) != DNodeIDToBikeVertexID.end()) {
                            
                            auto node1 = DStreetMap->NodeByID(node1ID);
                            auto node2 = DStreetMap->NodeByID(node2ID);
                            
                            if (node1 && node2) {
                                double distance = CalculateDistance(node1->Location(), node2->Location());
                                double bikeTime = distance / std::min(DBikeSpeed, speedLimit);
                                
                                DBikeRouter->AddEdge(
                                    DNodeIDToBikeVertexID[node1ID], 
                                    DNodeIDToBikeVertexID[node2ID], 
                                    bikeTime, 
                                    true);
                            }
                        }
                    }
                }
            }
        }
    }
    
    void BuildBusRouter() {
        DNodeIDToBusVertexID.clear();
        DBusVertexIDToNodeID.clear();
        DBusStopConnections.clear();
        
        // Add all nodes as vertices
        for (auto &node : DSortedNodes) {
            auto vertexID = DBusRouter->AddVertex(node->ID());
            DNodeIDToBusVertexID[node->ID()] = vertexID;
            DBusVertexIDToNodeID[vertexID] = node->ID();
        }
        
        // Process bus routes
        for (size_t routeIndex = 0; routeIndex < DBusSystem->RouteCount(); ++routeIndex) {
            auto route = DBusSystem->RouteByIndex(routeIndex);
            if (route && route->StopCount() >= 2) {
                std::string routeName = route->Name();
                
                for (size_t stopIndex = 1; stopIndex < route->StopCount(); ++stopIndex) {
                    TNodeID prevStopID = route->GetStopID(stopIndex - 1);
                    TNodeID currStopID = route->GetStopID(stopIndex);
                    
                    if (DNodeIDToBusVertexID.find(prevStopID) != DNodeIDToBusVertexID.end() &&
                        DNodeIDToBusVertexID.find(currStopID) != DNodeIDToBusVertexID.end()) {
                        
                        double time = route->GetStopTime(stopIndex) - route->GetStopTime(stopIndex - 1);
                        
                        if (time > 0) {
                            // Add edge for bus travel
                            DBusRouter->AddEdge(
                                DNodeIDToBusVertexID[prevStopID],
                                DNodeIDToBusVertexID[currStopID],
                                time + DBusStopTime,
                                false); // Buses run in one direction
                            
                            // Store bus route information for path description
                            DBusStopConnections[prevStopID].push_back({currStopID, routeName});
                        }
                    }
                }
            }
        }
        
        // Add walking connections between bus stops and nearby locations
        for (auto &node : DSortedNodes) {
            // Check if this is a bus stop
            bool isBusStop = false;
            for (size_t routeIndex = 0; routeIndex < DBusSystem->RouteCount(); ++routeIndex) {
                auto route = DBusSystem->RouteByIndex(routeIndex);
                if (route) {
                    for (size_t stopIndex = 0; stopIndex < route->StopCount(); ++stopIndex) {
                        if (route->GetStopID(stopIndex) == node->ID()) {
                            isBusStop = true;
                            break;
                        }
                    }
                    if (isBusStop) break;
                }
            }
            
            if (isBusStop) {
                // Add walking connections to nearby nodes
                for (auto &otherNode : DSortedNodes) {
                    if (node->ID() == otherNode->ID()) continue;
                    
                    double distance = CalculateDistance(node->Location(), otherNode->Location());
                    double walkThreshold = 500.0; // Consider connections within 500 meters
                    
                    if (distance <= walkThreshold) {
                        double walkTime = distance / DWalkSpeed;
                        
                        // Add walking edge both ways
                        DBusRouter->AddEdge(
                            DNodeIDToBusVertexID[node->ID()],
                            DNodeIDToBusVertexID[otherNode->ID()],
                            walkTime,
                            true);
                    }
                }
            }
        }
    }
    
    double CalculateDistance(const CStreetMap::TLocation &loc1, const CStreetMap::TLocation &loc2) const {
        // Haversine formula for distance calculation
        const double EarthRadius = 6371000.0; // Earth radius in meters
        
        double lat1 = loc1.first * M_PI / 180.0;
        double lon1 = loc1.second * M_PI / 180.0;
        double lat2 = loc2.first * M_PI / 180.0;
        double lon2 = loc2.second * M_PI / 180.0;
        
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;
        
        double a = sin(dLat/2) * sin(dLat/2) +
                   cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        
        return EarthRadius * c;
    }
    
    bool GetBusRouteBetweenStops(TNodeID src, TNodeID dest, std::string &routeName) const {
        auto it = DBusStopConnections.find(src);
        if (it != DBusStopConnections.end()) {
            for (const auto &connection : it->second) {
                if (connection.first == dest) {
                    routeName = connection.second;
                    return true;
                }
            }
        }
        return false;
    }
};

class CTransportationPlanner::CImplementation : public CTransportationPlanner {
private:
    std::unique_ptr<SImplementation> DImplementation;
    
public:
    CImplementation(const SConfiguration &config) {
        DImplementation = std::make_unique<SImplementation>(config);
    }
    
    ~CImplementation() = default;
    
    std::size_t NodeCount() const noexcept override {
        return DImplementation->DSortedNodes.size();
    }
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept override {
        if (index < DImplementation->DSortedNodes.size()) {
            return DImplementation->DSortedNodes[index];
        }
        return nullptr;
    }
    
    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) override {
        path.clear();
        
        if (src == dest) {
            path.push_back(src);
            return 0.0;
        }
        
        auto srcIt = DImplementation->DNodeIDToWalkVertexID.find(src);
        auto destIt = DImplementation->DNodeIDToWalkVertexID.find(dest);
        
        if (srcIt == DImplementation->DNodeIDToWalkVertexID.end() || 
            destIt == DImplementation->DNodeIDToWalkVertexID.end()) {
            return CPathRouter::NoPathExists;
        }
        
        std::vector<CPathRouter::TVertexID> routerPath;
        double distance = DImplementation->DWalkRouter->FindShortestPath(srcIt->second, destIt->second, routerPath);
        
        if (distance != CPathRouter::NoPathExists) {
            for (auto vertexID : routerPath) {
                path.push_back(DImplementation->DWalkVertexIDToNodeID[vertexID]);
            }
            
            // Convert time to distance
            double totalDistance = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                auto node1 = DImplementation->DStreetMap->NodeByID(path[i-1]);
                auto node2 = DImplementation->DStreetMap->NodeByID(path[i]);
                
                if (node1 && node2) {
                    totalDistance += DImplementation->CalculateDistance(node1->Location(), node2->Location());
                }
            }
            
            return totalDistance;
        }
        
        return CPathRouter::NoPathExists;
    }
    
    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) override {
        path.clear();
        
        if (src == dest) {
            path.push_back({ETransportationMode::Walk, src});
            return 0.0;
        }
        
        // Try all transportation methods and choose the fastest
        std::vector<TNodeID> walkPath;
        double walkTime = TryWalkPath(src, dest, walkPath);
        
        std::vector<TNodeID> bikePath;
        double bikeTime = TryBikePath(src, dest, bikePath);
        
        std::vector<CPathRouter::TVertexID> busRouterPath;
        double busTime = TryBusPath(src, dest, busRouterPath);
        
        // Determine which mode is fastest
        if (walkTime != CPathRouter::NoPathExists && 
            (walkTime <= bikeTime || bikeTime == CPathRouter::NoPathExists) && 
            (walkTime <= busTime || busTime == CPathRouter::NoPathExists)) {
            
            // Walk is fastest
            for (auto nodeID : walkPath) {
                path.push_back({ETransportationMode::Walk, nodeID});
            }
            return walkTime;
        }
        else if (bikeTime != CPathRouter::NoPathExists && 
                (bikeTime <= busTime || busTime == CPathRouter::NoPathExists)) {
            
            // Bike is fastest
            for (auto nodeID : bikePath) {
                path.push_back({ETransportationMode::Bike, nodeID});
            }
            return bikeTime;
        }
        else if (busTime != CPathRouter::NoPathExists) {
            // Bus is fastest, need to determine segments
            ConstructBusPath(busRouterPath, path);
            return busTime;
        }
        
        return CPathRouter::NoPathExists;
    }
    
    double TryWalkPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        path.clear();
        
        auto srcIt = DImplementation->DNodeIDToWalkVertexID.find(src);
        auto destIt = DImplementation->DNodeIDToWalkVertexID.find(dest);
        
        if (srcIt == DImplementation->DNodeIDToWalkVertexID.end() || 
            destIt == DImplementation->DNodeIDToWalkVertexID.end()) {
            return CPathRouter::NoPathExists;
        }
        
        std::vector<CPathRouter::TVertexID> routerPath;
        double time = DImplementation->DWalkRouter->FindShortestPath(srcIt->second, destIt->second, routerPath);
        
        if (time != CPathRouter::NoPathExists) {
            for (auto vertexID : routerPath) {
                path.push_back(DImplementation->DWalkVertexIDToNodeID[vertexID]);
            }
        }
        
        return time;
    }
    
    double TryBikePath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        path.clear();
        
        auto srcIt = DImplementation->DNodeIDToBikeVertexID.find(src);
        auto destIt = DImplementation->DNodeIDToBikeVertexID.find(dest);
        
        if (srcIt == DImplementation->DNodeIDToBikeVertexID.end() || 
            destIt == DImplementation->DNodeIDToBikeVertexID.end()) {
            return CPathRouter::NoPathExists;
        }
        
        std::vector<CPathRouter::TVertexID> routerPath;
        double time = DImplementation->DBikeRouter->FindShortestPath(srcIt->second, destIt->second, routerPath);
        
        if (time != CPathRouter::NoPathExists) {
            for (auto vertexID : routerPath) {
                path.push_back(DImplementation->DBikeVertexIDToNodeID[vertexID]);
            }
        }
        
        return time;
    }
    
    double TryBusPath(TNodeID src, TNodeID dest, std::vector<CPathRouter::TVertexID> &routerPath) {
        routerPath.clear();
        
        auto srcIt = DImplementation->DNodeIDToBusVertexID.find(src);
        auto destIt = DImplementation->DNodeIDToBusVertexID.find(dest);
        
        if (srcIt == DImplementation->DNodeIDToBusVertexID.end() || 
            destIt == DImplementation->DNodeIDToBusVertexID.end()) {
            return CPathRouter::NoPathExists;
        }
        
        return DImplementation->DBusRouter->FindShortestPath(srcIt->second, destIt->second, routerPath);
    }
    
    void ConstructBusPath(const std::vector<CPathRouter::TVertexID> &routerPath, std::vector<TTripStep> &path) {
        path.clear();
        
        if (routerPath.empty()) {
            return;
        }
        
        // First node is always included
        TNodeID currentNodeID = DImplementation->DBusVertexIDToNodeID[routerPath[0]];
        path.push_back({ETransportationMode::Walk, currentNodeID});
        
        ETransportationMode currentMode = ETransportationMode::Walk;
        
        for (size_t i = 1; i < routerPath.size(); ++i) {
            TNodeID nextNodeID = DImplementation->DBusVertexIDToNodeID[routerPath[i]];
            
            // Determine if this is a bus connection
            std::string routeName;
            if (DImplementation->GetBusRouteBetweenStops(currentNodeID, nextNodeID, routeName)) {
                // Switch to bus if not already on bus
                if (currentMode != ETransportationMode::Bus) {
                    currentMode = ETransportationMode::Bus;
                }
            } else {
                // Switch to walk if not already walking
                if (currentMode != ETransportationMode::Walk) {
                    currentMode = ETransportationMode::Walk;
                }
            }
            
            // Add the next step
            path.push_back({currentMode, nextNodeID});
            currentNodeID = nextNodeID;
        }
    }
    
    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const override {
        desc.clear();
        
        if (path.empty()) {
            return false;
        }
        
        if (path.size() == 1) {
            desc.push_back("Stay at current location.");
            return true;
        }
        
        ETransportationMode currentMode = path[0].first;
        TNodeID currentNodeID = path[0].second;
        TNodeID startNodeID = currentNodeID;
        std::string currentBusRoute;
        double segmentDistance = 0.0;
        
        for (size_t i = 1; i < path.size(); ++i) {
            ETransportationMode nextMode = path[i].first;
            TNodeID nextNodeID = path[i].second;
            
            auto currentNode = DImplementation->DStreetMap->NodeByID(currentNodeID);
            auto nextNode = DImplementation->DStreetMap->NodeByID(nextNodeID);
            
            if (!currentNode || !nextNode) {
                continue;
            }
            
            if (nextMode != currentMode || i == path.size() - 1) {
                // Create description for completed segment
                std::ostringstream ss;
                
                if (currentMode == ETransportationMode::Walk) {
                    if (i == path.size() - 1) {
                        segmentDistance += DImplementation->CalculateDistance(currentNode->Location(), nextNode->Location());
                    }
                    
                    ss << "Walk from ";
                    
                    auto startNode = DImplementation->DStreetMap->NodeByID(startNodeID);
                    if (startNode && startNode->HasAttribute("name")) {
                        ss << startNode->GetAttribute("name");
                    } else {
                        ss << "Node " << startNodeID;
                    }
                    
                    ss << " to ";
                    
                    if (nextNode->HasAttribute("name")) {
                        ss << nextNode->GetAttribute("name");
                    } else {
                        ss << "Node " << nextNodeID;
                    }
                    
                    ss << " (" << static_cast<int>(segmentDistance) << " meters).";
                    desc.push_back(ss.str());
                    
                    segmentDistance = 0.0;
                }
                else if (currentMode == ETransportationMode::Bike) {
                    if (i == path.size() - 1) {
                        segmentDistance += DImplementation->CalculateDistance(currentNode->Location(), nextNode->Location());
                    }
                    
                    ss << "Bike from ";
                    
                    auto startNode = DImplementation->DStreetMap->NodeByID(startNodeID);
                    if (startNode && startNode->HasAttribute("name")) {
                        ss << startNode->GetAttribute("name");
                    } else {
                        ss << "Node " << startNodeID;
                    }
                    
                    ss << " to ";
                    
                    if (nextNode->HasAttribute("name")) {
                        ss << nextNode->GetAttribute("name");
                    } else {
                        ss << "Node " << nextNodeID;
                    }
                    
                    ss << " (" << static_cast<int>(segmentDistance) << " meters).";
                    desc.push_back(ss.str());
                    
                    segmentDistance = 0.0;
                }
                else if (currentMode == ETransportationMode::Bus) {
                    ss << "Take bus " << currentBusRoute << " from ";
                    
                    auto startNode = DImplementation->DStreetMap->NodeByID(startNodeID);
                    if (startNode && startNode->HasAttribute("name")) {
                        ss << startNode->GetAttribute("name");
                    } else {
                        ss << "Node " << startNodeID;
                    }
                    
                    ss << " to ";
                    
                    if (nextNode->HasAttribute("name")) {
                        ss << nextNode->GetAttribute("name");
                    } else {
                        ss << "Node " << nextNodeID;
                    }
                    
                    ss << ".";
                    desc.push_back(ss.str());
                    
                    currentBusRoute = "";
                }
                
                // Start new segment
                startNodeID = nextNodeID;
                currentMode = nextMode;
            }
            else if (currentMode == ETransportationMode::Walk || currentMode == ETransportationMode::Bike) {
                // Accumulate distance for walking/biking
                segmentDistance += DImplementation->CalculateDistance(currentNode->Location(), nextNode->Location());
            }
            else if (currentMode == ETransportationMode::Bus && currentBusRoute.empty()) {
                // Get bus route name
                std::string routeName;
                if (DImplementation->GetBusRouteBetweenStops(currentNodeID, nextNodeID, routeName)) {
                    currentBusRoute = routeName;
                }
            }
            
            currentNodeID = nextNodeID;
        }
        
        return !desc.empty();
    }
};

// Factory function
std::unique_ptr<CTransportationPlanner> CTransportationPlanner::CreateTransportationPlanner(
    const CTransportationPlanner::SConfiguration &config) {
    return std::make_unique<CTransportationPlanner::CImplementation>(config);
}