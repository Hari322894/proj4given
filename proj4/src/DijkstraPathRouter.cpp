#include "DijkstraPathRouter.h"
#include <algorithm>
#include <queue>
#include <any>
#include <memory>
#include <limits>
#include <unordered_set>

struct CDijkstraPathRouter::SImplementation {
    struct SVertex {
        std::any Tag;
        std::unordered_map<TVertexID, double> DNeighbors;
    };

    std::vector<SVertex> DVertices;
};

CDijkstraPathRouter::CDijkstraPathRouter() {
    DImplementation = std::make_unique<SImplementation>();
}

CDijkstraPathRouter::~CDijkstraPathRouter() = default;

std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->DVertices.size();
}

CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    DImplementation->DVertices.emplace_back(SImplementation::SVertex{tag});
    return DImplementation->DVertices.size() - 1;
}

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    if (id >= 0 && id < static_cast<TVertexID>(DImplementation->DVertices.size())) {
        return DImplementation->DVertices[id].Tag;
    }
    return {};
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    if (src >= static_cast<TVertexID>(DImplementation->DVertices.size()) || 
        dest >= static_cast<TVertexID>(DImplementation->DVertices.size()) || 
        weight < 0) {
        return false;
    }
    DImplementation->DVertices[src].DNeighbors[dest] = weight;
    if (bidir) {
        DImplementation->DVertices[dest].DNeighbors[src] = weight;
    }
    return true;
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept {
    if (src >= static_cast<TVertexID>(DImplementation->DVertices.size()) || 
        dest >= static_cast<TVertexID>(DImplementation->DVertices.size())) {
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

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    // This function is currently a placeholder and does not perform any precomputation.
    return true;
}