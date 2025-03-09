#include "DijkstraPathRouter.h"
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <any>
#include <memory>
#include <vector>
#include <limits>

struct CDijkstraPathRouter::SImplementation {
    struct SVertex {
        std::unordered_map<TVertexID, double> DNeighbors;
        std::unordered_map<TVertexID, TEdgeLabel> DLabels;
    };

    std::vector<SVertex> DVertices;
    
    // Add precomputed paths storage
    std::unordered_map<TVertexID, std::unordered_map<TVertexID, std::pair<double, std::vector<TVertexID>>>> PrecomputedPaths;
    std::unordered_map<TVertexID, std::unordered_map<TVertexID, std::vector<TEdgeLabel>>> PrecomputedLabels;
    bool HasPrecomputed = false;
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
    if (src < static_cast<TVertexID>(DImplementation->DVertices.size()) && 
        dest < static_cast<TVertexID>(DImplementation->DVertices.size()) && 
        weight >= 0) {
        DImplementation->DVertices[src].DNeighbors[dest] = weight;
    }
}

void CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, TEdgeLabel label) noexcept {
    if (src < static_cast<TVertexID>(DImplementation->DVertices.size()) && 
        dest < static_cast<TVertexID>(DImplementation->DVertices.size()) && 
        weight >= 0) {
        DImplementation->DVertices[src].DNeighbors[dest] = weight;
        DImplementation->DVertices[src].DLabels[dest] = label;
    }
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept {
    path.clear();
    
    // Check if source or destination are invalid
    if (src >= static_cast<TVertexID>(DImplementation->DVertices.size()) || 
        dest >= static_cast<TVertexID>(DImplementation->DVertices.size())) {
        return CPathRouter::NoPathExists;
    }
    
    // Check if path is already precomputed
    if (DImplementation->HasPrecomputed) {
        auto srcIt = DImplementation->PrecomputedPaths.find(src);
        if (srcIt != DImplementation->PrecomputedPaths.end()) {
            auto destIt = srcIt->second.find(dest);
            if (destIt != srcIt->second.end()) {
                path = destIt->second.second;
                return destIt->second.first;
            }
        }
    }

    // If not precomputed or no precomputation, do Dijkstra
    std::vector<double> dist(DImplementation->DVertices.size(), std::numeric_limits<double>::infinity());
    std::vector<TVertexID> previous(DImplementation->DVertices.size(), CPathRouter::InvalidVertexID);
    dist[src] = 0;

    // Use a more efficient priority queue implementation
    using PQElement = std::pair<double, TVertexID>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> queue;
    queue.push({0, src});
    
    // Keep track of processed vertices
    std::vector<bool> processed(DImplementation->DVertices.size(), false);

    while (!queue.empty()) {
        auto [current_dist, u] = queue.top();
        queue.pop();
        
        // Skip if we've already found a better path to this vertex
        if (processed[u] || current_dist > dist[u]) {
            continue;
        }
        
        // Mark as processed
        processed[u] = true;
        
        // Found destination
        if (u == dest) {
            break;
        }

        // Process all neighbors
        for (const auto& [neighbor, weight] : DImplementation->DVertices[u].DNeighbors) {
            if (!processed[neighbor]) {
                double alt = dist[u] + weight;
                if (alt < dist[neighbor]) {
                    dist[neighbor] = alt;
                    previous[neighbor] = u;
                    queue.push({alt, neighbor});
                }
            }
        }
    }
    
    // Check if a path was found
    if (dist[dest] == std::numeric_limits<double>::infinity()) {
        return CPathRouter::NoPathExists;
    }
    
    // Reconstruct path
    TVertexID current = dest;
    while (current != src) {
        path.push_back(current);
        current = previous[current];
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());
    
    return dist[dest];
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path, std::vector<TEdgeLabel>& labels) noexcept {
    path.clear();
    labels.clear();
    
    // Check if source or destination are invalid
    if (src >= static_cast<TVertexID>(DImplementation->DVertices.size()) || 
        dest >= static_cast<TVertexID>(DImplementation->DVertices.size())) {
        return CPathRouter::NoPathExists;
    }
    
    // Check if path is already precomputed
    if (DImplementation->HasPrecomputed) {
        auto srcIt = DImplementation->PrecomputedPaths.find(src);
        if (srcIt != DImplementation->PrecomputedPaths.end()) {
            auto destIt = srcIt->second.find(dest);
            if (destIt != srcIt->second.end()) {
                auto labelsIt = DImplementation->PrecomputedLabels.find(src);
                if (labelsIt != DImplementation->PrecomputedLabels.end()) {
                    auto destLabelsIt = labelsIt->second.find(dest);
                    if (destLabelsIt != labelsIt->second.end()) {
                        path = destIt->second.second;
                        labels = destLabelsIt->second;
                        return destIt->second.first;
                    }
                }
            }
        }
    }

    // If not precomputed or no precomputation, do Dijkstra
    std::vector<double> dist(DImplementation->DVertices.size(), std::numeric_limits<double>::infinity());
    std::vector<TVertexID> previous(DImplementation->DVertices.size(), CPathRouter::InvalidVertexID);
    dist[src] = 0;

    // More efficient priority queue
    using PQElement = std::pair<double, TVertexID>;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> queue;
    queue.push({0, src});
    
    // Keep track of processed vertices
    std::vector<bool> processed(DImplementation->DVertices.size(), false);

    while (!queue.empty()) {
        auto [current_dist, u] = queue.top();
        queue.pop();
        
        // Skip if we've already found a better path or processed this vertex
        if (processed[u] || current_dist > dist[u]) {
            continue;
        }
        
        // Mark as processed
        processed[u] = true;
        
        // Found destination
        if (u == dest) {
            break;
        }

        // Process all neighbors
        for (const auto& [neighbor, weight] : DImplementation->DVertices[u].DNeighbors) {
            if (!processed[neighbor]) {
                double alt = dist[u] + weight;
                if (alt < dist[neighbor]) {
                    dist[neighbor] = alt;
                    previous[neighbor] = u;
                    queue.push({alt, neighbor});
                }
            }
        }
    }
    
    // Check if a path was found
    if (dist[dest] == std::numeric_limits<double>::infinity()) {
        return CPathRouter::NoPathExists;
    }
    
    // Reconstruct path and collect labels
    TVertexID current = dest;
    std::vector<TVertexID> reversePath;
    std::vector<TEdgeLabel> reverseLabels;
    
    while (current != src) {
        reversePath.push_back(current);
        TVertexID prev = previous[current];
        auto labelIt = DImplementation->DVertices[prev].DLabels.find(current);
        if (labelIt != DImplementation->DVertices[prev].DLabels.end()) {
            reverseLabels.push_back(labelIt->second);
        }
        current = prev;
    }
    reversePath.push_back(src);
    
    // Reverse to get correct order
    path.reserve(reversePath.size());
    for (auto it = reversePath.rbegin(); it != reversePath.rend(); ++it) {
        path.push_back(*it);
    }
    
    labels.reserve(reverseLabels.size());
    for (auto it = reverseLabels.rbegin(); it != reverseLabels.rend(); ++it) {
        labels.push_back(*it);
    }
    
    return dist[dest];
}

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    if (DImplementation->HasPrecomputed) {
        return true; // Already precomputed
    }
    
    // Limit precomputation to important nodes or a subset of nodes
    // For simplicity, let's limit to the first 100 nodes or all if fewer
    const size_t maxNodesToPrecompute = 100;
    
    // Get all vertices
    std::vector<TVertexID> vertices;
    for (TVertexID i = 0; i < DImplementation->DVertices.size(); ++i) {
        // Skip empty vertices (no edges)
        if (!DImplementation->DVertices[i].DNeighbors.empty()) {
            vertices.push_back(i);
        }
    }
    
    // Limit number of vertices to precompute if needed
    size_t numToPrecompute = std::min(vertices.size(), maxNodesToPrecompute);
    
    // Precompute paths for important nodes
    for (size_t i = 0; i < numToPrecompute && std::chrono::steady_clock::now() < deadline; ++i) {
        TVertexID src = vertices[i];
        
        // Compute shortest path from this source to all destinations
        std::vector<double> dist(DImplementation->DVertices.size(), std::numeric_limits<double>::infinity());
        std::vector<TVertexID> previous(DImplementation->DVertices.size(), CPathRouter::InvalidVertexID);
        dist[src] = 0;
        
        // Use efficient priority queue
        using PQElement = std::pair<double, TVertexID>;
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> queue;
        queue.push({0, src});
        
        std::vector<bool> processed(DImplementation->DVertices.size(), false);
        
        // Run Dijkstra's algorithm from this source
        while (!queue.empty() && std::chrono::steady_clock::now() < deadline) {
            auto [current_dist, u] = queue.top();
            queue.pop();
            
            if (processed[u] || current_dist > dist[u]) {
                continue;
            }
            
            processed[u] = true;
            
            for (const auto& [neighbor, weight] : DImplementation->DVertices[u].DNeighbors) {
                if (!processed[neighbor]) {
                    double alt = dist[u] + weight;
                    if (alt < dist[neighbor]) {
                        dist[neighbor] = alt;
                        previous[neighbor] = u;
                        queue.push({alt, neighbor});
                    }
                }
            }
        }
        
        // If we ran out of time, return partial precomputation
        if (std::chrono::steady_clock::now() >= deadline) {
            DImplementation->HasPrecomputed = !DImplementation->PrecomputedPaths.empty();
            return false;
        }
        
        // Store precomputed paths from this source
        for (TVertexID dest = 0; dest < DImplementation->DVertices.size(); ++dest) {
            if (dist[dest] != std::numeric_limits<double>::infinity() && src != dest) {
                // Reconstruct path
                std::vector<TVertexID> path;
                std::vector<TEdgeLabel> edgeLabels;
                
                TVertexID current = dest;
                while (current != src) {
                    path.push_back(current);
                    TVertexID prev = previous[current];
                    
                    // Get edge label if exists
                    auto labelIt = DImplementation->DVertices[prev].DLabels.find(current);
                    if (labelIt != DImplementation->DVertices[prev].DLabels.end()) {
                        edgeLabels.push_back(labelIt->second);
                    }
                    
                    current = prev;
                }
                path.push_back(src);
                
                // Reverse to get correct order
                std::reverse(path.begin(), path.end());
                std::reverse(edgeLabels.begin(), edgeLabels.end());
                
                // Store precomputed path
                DImplementation->PrecomputedPaths[src][dest] = {dist[dest], path};
                DImplementation->PrecomputedLabels[src][dest] = edgeLabels;
            }
        }
    }
    
    DImplementation->HasPrecomputed = true;
    return true;
}