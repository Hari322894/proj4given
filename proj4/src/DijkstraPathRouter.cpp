#include "DijkstraPathRouter.h"
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>

// Implementation structure for Dijkstra Path Router
struct CDijkstraPathRouter::SImplementation {
    struct SVertex {
        TVertexID ID;
        std::any Tag;
        std::unordered_map<TVertexID, double> Edges;
    };
    
    std::unordered_map<TVertexID, SVertex> DVertices;
    TVertexID DNextID = 0;
    
    // Helper method to implement Dijkstra's algorithm
    double DijkstraSearch(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) {
        // Initialize data structures
        std::unordered_map<TVertexID, double> distances;
        std::unordered_map<TVertexID, TVertexID> previous;
        std::priority_queue<std::pair<double, TVertexID>, 
                            std::vector<std::pair<double, TVertexID>>,
                            std::greater<std::pair<double, TVertexID>>> queue;
        
        // Initialize all distances as infinite
        for (const auto& vertex : DVertices) {
            distances[vertex.first] = std::numeric_limits<double>::infinity();
        }
        
        // Distance to source is 0
        distances[src] = 0.0;
        queue.push({0.0, src});
        
        // Process vertices
        while (!queue.empty()) {
            auto current = queue.top().second;
            auto currentDistance = queue.top().first;
            queue.pop();
            
            // If we reached destination, break
            if (current == dest) {
                break;
            }
            
            // Skip if we already found a better path
            if (currentDistance > distances[current]) {
                continue;
            }
            
            // Check all neighbors
            for (const auto& edge : DVertices[current].Edges) {
                auto neighbor = edge.first;
                auto weight = edge.second;
                auto newDistance = distances[current] + weight;
                
                // If we found a better path
                if (newDistance < distances[neighbor]) {
                    distances[neighbor] = newDistance;
                    previous[neighbor] = current;
                    queue.push({newDistance, neighbor});
                }
            }
        }
        
        // Check if path exists
        if (distances[dest] == std::numeric_limits<double>::infinity()) {
            return CPathRouter::NoPathExists;
        }
        
        // Reconstruct path
        path.clear();
        TVertexID current = dest;
        while (current != src) {
            path.push_back(current);
            current = previous[current];
        }
        path.push_back(src);
        
        // Reverse to get path from src to dest
        std::reverse(path.begin(), path.end());
        
        return distances[dest];
    }
};

// Constructor implementation
CDijkstraPathRouter::CDijkstraPathRouter() {
    DImplementation = std::make_unique<SImplementation>();
}

// Destructor implementation
CDijkstraPathRouter::~CDijkstraPathRouter() {
}

// Returns the number of vertices in the path router
std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->DVertices.size();
}

// Adds a vertex with the provided tag
CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    TVertexID newID = DImplementation->DNextID++;
    DImplementation->DVertices[newID] = {newID, tag, {}};
    return newID;
}

// Gets the tag of the vertex specified by id
std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    auto it = DImplementation->DVertices.find(id);
    if (it == DImplementation->DVertices.end()) {
        return std::any();
    }
    return it->second.Tag;
}

// Adds an edge between src and dest vertices
bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    // Check if vertices exist and weight is valid
    if (DImplementation->DVertices.find(src) == DImplementation->DVertices.end() ||
        DImplementation->DVertices.find(dest) == DImplementation->DVertices.end() ||
        weight < 0.0) {
        return false;
    }
    
    // Add edge from src to dest
    DImplementation->DVertices[src].Edges[dest] = weight;
    
    // If bidirectional, add edge from dest to src
    if (bidir) {
        DImplementation->DVertices[dest].Edges[src] = weight;
    }
    
    return true;
}

// Allows precomputation if needed
bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    // For basic Dijkstra's algorithm, no precomputation is needed
    return true;
}

// Finds the shortest path between src and dest vertices
double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept {
    // Check if vertices exist
    if (DImplementation->DVertices.find(src) == DImplementation->DVertices.end() ||
        DImplementation->DVertices.find(dest) == DImplementation->DVertices.end()) {
        return CPathRouter::NoPathExists;
    }
    
    // Run Dijkstra's algorithm
    return DImplementation->DijkstraSearch(src, dest, path);
}