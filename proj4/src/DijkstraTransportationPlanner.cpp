#include "DijkstraTransportationPlanner.h"
#include "DijkstraPathRouter.h"
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <queue>
#include <cmath> // Added for math functions

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
        for (std::size_t i = 0; i < DStreetMap->NodeCount(); ++i) {
            auto node = DStreetMap->NodeByIndex(i);
            DNodes.push_back(node);
        }

        // Sort nodes by ID for consistent indexing
        std::sort(DNodes.begin(), DNodes.end(),
                  [](const auto &a, const auto &b) { return a->ID() < b->ID(); });

        // Add vertices and edges to the path router
        for (const auto &node : DNodes) {
            DPathRouter->AddVertex(node->ID());
        }

        // Add street edges
        for (std::size_t i = 0; i < DStreetMap->WayCount(); ++i) {
            auto way = DStreetMap->WayByIndex(i);
            if (way->NodeCount() < 2)
                continue;

            // We won't use this variable directly but it's needed for context
            double speed = config->DefaultSpeedLimit();
            if (way->HasAttribute("maxspeed")) {
                speed = std::stod(way->GetAttribute("maxspeed"));
            }

            for (std::size_t j = 1; j < way->NodeCount(); ++j) {
                auto node1ID = way->GetNodeID(j - 1);
                auto node2ID = way->GetNodeID(j);

                // Get the node objects from their IDs
                std::shared_ptr<CStreetMap::SNode> node1 = nullptr;
                std::shared_ptr<CStreetMap::SNode> node2 = nullptr;

                for (const auto &node : DNodes) {
                    if (node->ID() == node1ID)
                        node1 = node;
                    if (node->ID() == node2ID)
                        node2 = node;
                    if (node1 && node2)
                        break;
                }

                if (node1 && node2) {
                    double distance = CalculateDistance(node1, node2);
                    DPathRouter->AddEdge(node1->ID(), node2->ID(), distance, false);

                    // Add edge in reverse if way is bidirectional
                    if (!way->HasAttribute("oneway") || way->GetAttribute("oneway") != "yes") {
                        DPathRouter->AddEdge(node2->ID(), node1->ID(), distance, false);
                    }
                }
            }
        }
    }

    double CalculateDistance(std::shared_ptr<CStreetMap::SNode> node1,
                             std::shared_ptr<CStreetMap::SNode> node2) const {
        // Haversine formula for calculating distance between two lat/lon points
        const double EarthRadiusKm = 6371.0;
        const double DegreesToRadians = M_PI / 180.0;

        double lat1 = node1->Location().first * DegreesToRadians;
        double lon1 = node1->Location().second * DegreesToRadians;
        double lat2 = node2->Location().first * DegreesToRadians;
        double lon2 = node2->Location().second * DegreesToRadians;

        double dlon = lon2 - lon1;
        double dlat = lat2 - lat1;

        double a = sin(dlat / 2) * sin(dlat / 2) +
                   cos(lat1) * cos(lat2) *
                       sin(dlon / 2) * sin(dlon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

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
        for (const auto &node : DNodes) {
            if (node->ID() == src)
                srcExists = true;
            if (node->ID() == dest)
                destExists = true;
            if (srcExists && destExists)
                break;
        }

        if (!srcExists || !destExists) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        std::vector<CPathRouter::TVertexID> routerPath;
        double distance = DPathRouter->FindShortestPath(src, dest, routerPath);

        if (distance < 0) {
            return std::numeric_limits<double>::max(); // Indicate no path exists
        }

        // Convert router path to node IDs
        for (const auto &vertex : routerPath) {
            path.push_back(vertex);
        }

        return distance;
    }

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector<TTripStep> &path) {
        path.clear();

        // Check if source and destination are the same
        if (src == dest) {
            TTripStep step;
            step.first = ETransportationMode::Walk;
            step.second = src;
            path.push_back(step);
            return 0.0;
        }

        // Find paths using different transportation modes
        std::vector<TNodeID> walkPath;
        double walkDistance = FindShortestPath(src, dest, walkPath);

        if (walkDistance == std::numeric_limits<double>::max()) {
            return std::numeric_limits<double>::max(); // No path exists
        }

        // Convert shortest path to trip steps (walking only for now)
        for (const auto &nodeID : walkPath) {
            TTripStep step;
            step.first = ETransportationMode::Walk;
            step.second = nodeID;
            path.push_back(step);
        }

        // Calculate time based on walking speed
        double walkTime = walkDistance / DConfig->WalkSpeed();
        return walkTime;
    }

    bool GetPathDescription(const std::vector<TTripStep> &path, std::vector<std::string> &desc) const {
        desc.clear();

        if (path.empty()) {
            return false;
        }

        // Build meaningful description of the path
        ETransportationMode currentMode = path[0].first;
        TNodeID currentNodeID = path[0].second;
        std::shared_ptr<CStreetMap::SNode> currentNode = nullptr;

        for (const auto &node : DNodes) {
            if (node->ID() == currentNodeID) {
                currentNode = node;
                break;
            }
        }

        if (!currentNode) {
            return false;
        }

        std::string startDesc = "Start at ";
        if (currentNode->HasAttribute("name")) {
            startDesc += currentNode->GetAttribute("name");
        } else {
            startDesc += "node " + std::to_string(currentNodeID);
        }
        desc.push_back(startDesc);

        for (std::size_t i = 1; i < path.size(); ++i) {
            const auto &step = path[i];
            std::string stepDesc;

            // Find node information
            std::shared_ptr<CStreetMap::SNode> node = nullptr;
            for (const auto &n : DNodes) {
                if (n->ID() == step.second) {
                    node = n;
                    break;
                }
            }

            if (!node)
                continue;

            // Create description based on transportation mode
            switch (step.first) {
            case ETransportationMode::Walk:
                stepDesc = "Walk to ";
                break;
            case ETransportationMode::Bike:
                stepDesc = "Bike to ";
                break;
            case ETransportationMode::Bus:
                stepDesc = "Take bus to ";
                break;
            }

            if (node->HasAttribute("name")) {
                stepDesc += node->GetAttribute("name");
            } else {
                stepDesc += "node " + std::to_string(step.second);
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