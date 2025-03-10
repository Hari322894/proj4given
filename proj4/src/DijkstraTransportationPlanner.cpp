#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <queue>

struct CDijkstraTransportationPlanner::SImplementation {
    std::shared_ptr<SConfiguration> DConfig;
    std::shared_ptr<CStreetMap> DStreetMap;
    std::shared_ptr<CBusSystem> DBusSystem;
    std::shared_ptr<CDijkstraPathRouter> DPathRouter;
    std::vector<std::shared_ptr<CStreetMap::SNode>> DNodes;
    
    SImplementation(std::shared_ptr<SConfiguration> config) {
        DConfig = config;
        DStreetMap = config->StreetMap();
        DBusSystem = config->BusSystem();
        
        // Initialize path router
        DPathRouter = std::make_shared<CDijkstraPathRouter>();
        
        // Store all nodes for quick lookup
        for (size_t i = 0; i < DStreetMap->NodeCount(); ++i) {
            auto node = DStreetMap->NodeByIndex(i);
            DNodes.push_back(node);
        }
        
        // Sort nodes by ID for consistent indexing
        std::sort(DNodes.begin(), DNodes.end(), 
            [](const auto& a, const auto& b) { return a->ID() < b->ID(); });
        
        // Add vertices and edges to the path router
        for (const auto& node : DNodes) {
            DPathRouter->AddVertex(node->ID());
        }
        
        // Add street edges
        for (size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way->NodeCount() < 2) continue;
            
            double speed = way->HasAttribute("maxspeed") ? 
                            std::stod(way->GetAttribute("maxspeed")) : 
                            config->DefaultSpeedLimit();
            
            for (size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1 = way->GetNodeByIndex(j-1);
                auto node2 = way->GetNodeByIndex(j);
                
                double distance = CalculateDistance(node1, node2);
                DPathRouter->AddEdge(node1->ID(), node2->ID(), distance);
                
                // Add edge in reverse if way is bidirectional
                if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                    DPathRouter->AddEdge(node2->ID(), node1->ID(), distance);
                }
            }
        }
    }
    
    double CalculateDistance(std::shared_ptr<CStreetMap::SNode> node1, 
                            std::shared_ptr<CStreetMap::SNode> node2) const {
        // Haversine formula for calculating distance between two lat/lon points
        const double EarthRadiusKm = 6371.0;
        const double DegreesToRadians = M_PI / 180.0;
        
        double lat1 = node1->Latitude() * DegreesToRadians;
        double lon1 = node1->Longitude() * DegreesToRadians;
        double lat2 = node2->Latitude() * DegreesToRadians;
        double lon2 = node2->Longitude() * DegreesToRadians;
        
        double dlon = lon2 - lon1;
        double dlat = lat2 - lat1;
        
        double a = std::sin(dlat/2) * std::sin(dlat/2) + 
                  std::cos(lat1) * std::cos(lat2) * 
                  std::sin(dlon/2) * std::sin(dlon/2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
        
        return EarthRadiusKm * c;
    }
    
    std::size_t NodeCount() const noexcept {
        return DNodes.size();
    }
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept {
        if (index < DNodes.size()) {
            return DNodes[index];
        }
        return nullptr;
    }

    double FindShortestPath(TNodeID src, TNodeID dest, std::vector<TNodeID> &path) {
        path.clear();
        
        // Check if nodes exist
        bool srcExists = false, destExists = false;
        for (const auto& node : DNodes) {
            if (node->ID() == src) srcExists = true;
            if (node->ID() == dest) destExists = true;
            if (srcExists && destExists) break;
        }
        
        if (!srcExists || !destExists) {
            return -1.0; // No path exists
        }
        
        std::vector<CPathRouter::TVertexID> routerPath;
        double distance = DPathRouter->FindShortestPath(src, dest, routerPath);
        
        if (distance < 0) {
            return -1.0; // No path exists
        }
        
        // Convert router path to node IDs
        for (const auto& vertex : routerPath) {
            path.push_back(vertex);
        }
        
        return distance;
    }

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();
        
        // Simplified implementation - in reality, would consider bus routes, walking speeds, etc.
        std::vector<TNodeID> shortestPath;
        double distance = FindShortestPath(src, dest, shortestPath);
        
        if (distance < 0) {
            return -1.0; // No path exists
        }
        
        // Convert shortest path to trip steps (walking only for simplicity)
        for (const auto& nodeID : shortestPath) {
            TTripStep step;
            step.first = ETransportationMode::Walk;
            step.second = nodeID;
            path.push_back(step);
        }
        
        // Calculate time based on walking speed
        double walkTime = distance / DConfig->WalkSpeed();
        return walkTime;
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();
        
        if (path.empty()) {
            return false;
        }
        
        // Simple implementation - just describe each step
        for (size_t i = 0; i < path.size(); ++i) {
            const auto& step = path[i];
            std::string stepDesc;
            
            switch (step.first) {
                case ETransportationMode::Walk:
                    stepDesc = "Walk to node " + std::to_string(step.second);
                    break;
                case ETransportationMode::Bike:
                    stepDesc = "Bike to node " + std::to_string(step.second);
                    break;
                case ETransportationMode::Bus:
                    stepDesc = "Take bus to node " + std::to_string(step.second);
                    break;
            }
            
            desc.push_back(stepDesc);
        }
        
        return true;
    }
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config) {
    DImplementation = std::make_unique<SImplementation>(config);
}

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner() {
    // Destructor body can be empty when using std::unique_ptr
}

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