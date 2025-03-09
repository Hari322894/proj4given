#include "DijkstraPathRouter.h"
#include <algorithm>
#include <any>
#include <memory>

CDijkstraPathRouter::CDijkstraPathRouter() {
    DImplementation = std::make_unique<SImplementation>();
}

CDijkstraPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    DImplementation->DVertices.emplace_back(SVertex{tag});
    return DImplementation->DVertices.size() - 1;
}

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    if (id >= 0 && id < DImplementation->DVertices.size()) {
        return DImplementation->DVertices[id].Tag;
    }
    return {};
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidirectional) noexcept {
    if (src >= DImplementation->DVertices.size() || dest >= DImplementation->DVertices.size() || weight < 0) {
        return false;
    }
    DImplementation->DVertices[src].DNeighbors[dest] = weight;
    if (bidirectional) {
        DImplementation->DVertices[dest].DNeighbors[src] = weight;
    }
    return true;
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept {
    if (src >= DImplementation->DVertices.size() || dest >= DImplementation->DVertices.size()) {
        return NoPathExists;
    }

    std::vector<double> dist(DImplementation->DVertices.size(), std::numeric_limits<double>::infinity());
    std::vector<TVertexID> previous(DImplementation->DVertices.size(), InvalidVertexID);
    std::unordered_set<TVertexID> visited;
    dist[src] = 0;

    auto cmp = [&dist](TVertexID left, TVertexID right) { return dist[left] > dist[right]; };
    std::priority_queue<TVertexID, std::vector<TVertexID>, decltype(cmp)> queue(cmp);

    queue.push(src);

    while (!queue.empty()) {
        TVertexID u = queue.top();
        queue.pop();

        if (u == dest) {
            while (previous[u] != InvalidVertexID) {
                path.push_back(u);
                u = previous[u];
            }
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

    return NoPathExists;
}