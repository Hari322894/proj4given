#include "DijkstraPathRouter.h"
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <any>
#include <memory>

struct CDijkstraPathRouter::SImplementation {
    struct SVertex {
        std::unordered_map<TVertexID, double> DNeighbors;
        std::unordered_map<TVertexID, TEdgeLabel> DLabels;
    };

    std::vector<SVertex> DVertices;
};

CDijkstraPathRouter::CDijkstraPathRouter() {
    DImplementation = std::make_unique<SImplementation>();
}

CDijkstraPathRouter::~CDijkstraPathRouter() = default;

void CDijkstraPathRouter::AddNode(TVertexID node) noexcept {
    if (node >= static_cast<TVertexID>(DImplementation->DVertices.size())) {
        DImplementation->DVertices.resize(node + 1);
    }
}

void CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight) noexcept {
    if (src < static_cast<TVertexID>(DImplementation->DVertices.size()) && dest < static_cast<TVertexID>(DImplementation->DVertices.size()) && weight >= 0) {
        DImplementation->DVertices[src].DNeighbors[dest] = weight;
    }
}

void CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, TEdgeLabel label) noexcept {
    if (src < static_cast<TVertexID>(DImplementation->DVertices.size()) && dest < static_cast<TVertexID>(DImplementation->DVertices.size()) && weight >= 0) {
        DImplementation->DVertices[src].DNeighbors[dest] = weight;
        DImplementation->DVertices[src].DLabels[dest] = label;
    }
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept {
    if (src >= static_cast<TVertexID>(DImplementation->DVertices.size()) || dest >= static_cast<TVertexID>(DImplementation->DVertices.size())) {
        return CPathRouter::NoPathExists;
    }

    std::vector<double> dist(DImplementation->DVertices.size(), std::numeric_limits<double>::infinity());
    std::vector<TVertexID> previous(DImplementation->DVertices.size(), CPathRouter::InvalidVertexID);
    std::unordered_set<TVertexID> visited;
    dist[src] = 0;

    auto cmp = [&dist](TVertexID left, TVertexID right) { return dist[left] > dist[right]; };
    std::priority_queue<TVertexID, std::vector<TVertexID>, decltype(cmp)> queue(cmp);

    queue.push(src);

    while (!queue.empty()) {
        TVertexID u = queue.top();
        queue.pop();

        if (u == dest) {
            while (previous[u] != CPathRouter::InvalidVertexID) {
                path.push_back(u);
                u = previous[u];
            }
            path.push_back(src);
            std::reverse(path.begin(), path.end());
            return dist[dest];
        }

        visited.insert(u);

        for (const auto& neighbor : DImplementation->DVertices[u].DNeighbors) {
            if (visited.find(neighbor.first) == visited.end()) {
                double alt = dist[u] + neighbor.second;
                if (alt < dist[neighbor.first]) {
                    dist[neighbor.first] = alt;
                    previous[neighbor.first] = u;
                    queue.push(neighbor.first);
                }
            }
        }
    }

    return CPathRouter::NoPathExists;
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path, std::vector<TEdgeLabel>& labels) noexcept {
    if (src >= static_cast<TVertexID>(DImplementation->DVertices.size()) || dest >= static_cast<TVertexID>(DImplementation->DVertices.size())) {
        return CPathRouter::NoPathExists;
    }

    std::vector<double> dist(DImplementation->DVertices.size(), std::numeric_limits<double>::infinity());
    std::vector<TVertexID> previous(DImplementation->DVertices.size(), CPathRouter::InvalidVertexID);
    std::unordered_set<TVertexID> visited;
    dist[src] = 0;

    auto cmp = [&dist](TVertexID left, TVertexID right) { return dist[left] > dist[right]; };
    std::priority_queue<TVertexID, std::vector<TVertexID>, decltype(cmp)> queue(cmp);

    queue.push(src);

    while (!queue.empty()) {
        TVertexID u = queue.top();
        queue.pop();

        if (u == dest) {
            while (previous[u] != CPathRouter::InvalidVertexID) {
                path.push_back(u);
                if (previous[u] != CPathRouter::InvalidVertexID) {
                    labels.push_back(DImplementation->DVertices[previous[u]].DLabels[u]);
                }
                u = previous[u];
            }
            path.push_back(src);
            std::reverse(path.begin(), path.end());
            std::reverse(labels.begin(), labels.end());
            return dist[dest];
        }

        visited.insert(u);

        for (const auto& neighbor : DImplementation->DVertices[u].DNeighbors) {
            if (visited.find(neighbor.first) == visited.end()) {
                double alt = dist[u] + neighbor.second;
                if (alt < dist[neighbor.first]) {
                    dist[neighbor.first] = alt;
                    previous[neighbor.first] = u;
                    queue.push(neighbor.first);
                }
            }
        }
    }

    return CPathRouter::NoPathExists;
}

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    // This function is currently a placeholder and does not perform any precomputation.
    return true;
}