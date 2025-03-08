#include "DijkstraPathRouter.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>

struct CDijkstraPathRouter::SImplementation {
    struct SVertex {
        TVertexID ID;
        std::any Tag;
        std::unordered_map<TVertexID, double> Edges; // Maps to destination vertex ID and edge weight
    };
    
    std::vector<SVertex> DVertices;
    
    SImplementation() = default;
    
    struct SDistanceVertexPair {
        double Distance;
        TVertexID VertexID;
        
        bool operator>(const SDistanceVertexPair& other) const {
            return Distance > other.Distance;
        }
    };
};

CDijkstraPathRouter::CDijkstraPathRouter() 
    : DImplementation(std::make_unique<SImplementation>()) {
}

CDijkstraPathRouter::~CDijkstraPathRouter() = default;

std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->DVertices.size();
}

CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    SImplementation::SVertex newVertex;
    newVertex.ID = DImplementation->DVertices.size();
    newVertex.Tag = tag;
    
    DImplementation->DVertices.push_back(newVertex);
    return newVertex.ID;
}

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    if(id < DImplementation->DVertices.size()) {
        return DImplementation->DVertices[id].Tag;
    }
    return std::any();
}

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    if(src >= DImplementation->DVertices.size() || dest >= DImplementation->DVertices.size() || weight < 0) {
        return false;
    }
    
    // Add edge from source to destination
    DImplementation->DVertices[src].Edges[dest] = weight;
    
    // If bidirectional, add edge from destination to source as well
    if(bidir) {
        DImplementation->DVertices[dest].Edges[src] = weight;
    }
    
    return true;
}

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    // No precomputation needed for Dijkstra's algorithm
    return true;
}

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept {
    path.clear();
    
    if(src >= DImplementation->DVertices.size() || dest >= DImplementation->DVertices.size()) {
        return NoPathExists;
    }
    
    if(src == dest) {
        // Source and destination are the same
        path.push_back(src);
        return 0.0;
    }
    
    std::vector<double> distances(DImplementation->DVertices.size(), std::numeric_limits<double>::infinity());
    std::vector<TVertexID> previous(DImplementation->DVertices.size(), InvalidVertexID);
    std::vector<bool> visited(DImplementation->DVertices.size(), false);
    
    // Priority queue to get the vertex with the smallest distance
    std::priority_queue<SImplementation::SDistanceVertexPair, 
                        std::vector<SImplementation::SDistanceVertexPair>,
                        std::greater<SImplementation::SDistanceVertexPair>> pq;
    
    // Distance to source is 0
    distances[src] = 0.0;
    pq.push({0.0, src});
    
    while(!pq.empty()) {
        auto [currentDist, currentVertex] = pq.top();
        pq.pop();
        
        // Skip if already visited
        if(visited[currentVertex]) {
            continue;
        }
        
        // Mark as visited
        visited[currentVertex] = true;
        
        // Found destination
        if(currentVertex == dest) {
            break;
        }
        
        // Check all neighbors
        for(const auto& [neighbor, weight] : DImplementation->DVertices[currentVertex].Edges) {
            double newDist = distances[currentVertex] + weight;
            
            if(newDist < distances[neighbor]) {
                distances[neighbor] = newDist;
                previous[neighbor] = currentVertex;
                pq.push({newDist, neighbor});
            }
        }
    }
    
    // Check if destination was reached
    if(distances[dest] == std::numeric_limits<double>::infinity()) {
        return NoPathExists;
    }
    
    // Construct the path from source to destination
    TVertexID current = dest;
    while(current != InvalidVertexID) {
        path.push_back(current);
        current = previous[current];
    }
    
    // Reverse the path to get it from source to destination
    std::reverse(path.begin(), path.end());
    
    return distances[dest];
}