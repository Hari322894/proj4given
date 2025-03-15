#include "DijkstraPathRouter.h"// including header file for CDijkstraPathRouter class usage
#include <limits>//used for numeric_limits
#include <queue>//used for priority_queue
#include <vector>// used for vector
#include <unordered_map>//used for unordered_map
#include <algorithm>//used for reverse
#include <any>//used for any
#include <memory>//used for shared_ptr
#include <chrono>// used for steady_clock

// Implementation of Dijkstra's algorithm for path routing
struct CDijkstraPathRouter::SImplementation {
    struct IndVertex {
        // Vertex ID
        TVertexID ThisVertexID;
        std::any ThisVertexTag;
        std::vector<TVertexID> ConnectedIDs;
        std::unordered_map<TVertexID, double> MapOfWeights;
        // Constructor
        ~IndVertex() {}
        // Getters:
        TVertexID GetVertexID() {
            return ThisVertexID;
        }
        // Get vertex tag
        std::any GetThisVertexTag() {
            return ThisVertexTag;
        }
        // Get connected vertex count
        std::size_t ConnectedIDCount() {
            return ConnectedIDs.size();
        }
        // Get connected vertex IDs
        std::vector<TVertexID> GetConnectedVertexIDs() {
            return ConnectedIDs;
        }
        // get weight of edge to vertex
        double GetWeight(TVertexID &id) {
            auto Search = MapOfWeights.find(id);
            if(Search == MapOfWeights.end()) {
                return std::numeric_limits<double>::infinity();
            }
            // return weight
            return Search->second;
        }
    };
    // vector of all vertices
    std::vector<std::shared_ptr<IndVertex>> AllVertices;
    size_t IndexKeeper = 0;
    // constructor
    SImplementation() {}
    // destructor
    std::size_t VertexCount() const {
        return AllVertices.size();
    }
    // add vertex
    TVertexID AddVertex(std::any tag) {
        auto NewVertex = std::make_shared<IndVertex>();
        NewVertex->ThisVertexID = IndexKeeper;
        NewVertex->ThisVertexTag = tag;
        AllVertices.push_back(NewVertex);
        return IndexKeeper++;
    }
    // get vertex tag
    std::any GetVertexTag(TVertexID id) const {
        return AllVertices[id]->GetThisVertexTag();
    }
    // add edge
    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir = false) {
        if (weight > 0) {
            AllVertices[src]->MapOfWeights[dest] = weight;
            AllVertices[src]->ConnectedIDs.push_back(dest);
         // if bidirectional, add edge in reverse   
            if (bidir) {
                AllVertices[dest]->MapOfWeights[src] = weight;
                AllVertices[dest]->ConnectedIDs.push_back(src);
            }
            return true;
        }
        return false;
    }
    // precompute
    bool Precompute(std::chrono::steady_clock::time_point deadline) {
        return true;
    }
    // find shortest path
    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) {
        // clear the path vector first
        path.clear();
        
        // check if src and dest are valid vertices
        if(src >= VertexCount() || dest >= VertexCount()) {
            return NoPathExists;
        }
        
        // create a min-heap priority queue
        // pair of (distance, vertex)
        typedef std::pair<double, TVertexID> DistVertex;
        std::priority_queue<DistVertex, std::vector<DistVertex>, std::greater<DistVertex>> pq;
        
        // create distance array and predecessor array
        std::vector<double> distance(VertexCount(), std::numeric_limits<double>::infinity());
        std::vector<TVertexID> predecessor(VertexCount(), std::numeric_limits<TVertexID>::max());
        
        // initialize source distance and add to queue
        distance[src] = 0;
        pq.push({0, src});
        
        // dijkstra's algorithm
        while(!pq.empty()) {
            double dist = pq.top().first;
            TVertexID u = pq.top().second;
            pq.pop();
            
            // skip if we've found a better path already
            if(dist > distance[u]) continue;
            
            // if we reached destination, break
            if(u == dest) break;
            
            // Check all neighbors of u
            for(TVertexID v : AllVertices[u]->GetConnectedVertexIDs()) {
                double weight = AllVertices[u]->GetWeight(v);
                
                // Relaxation
                if(distance[u] + weight < distance[v]) {
                    distance[v] = distance[u] + weight;
                    predecessor[v] = u;
                    pq.push({distance[v], v});
                }
            }
        }
        
        // check if path exists
        if(distance[dest] == std::numeric_limits<double>::infinity()) {
            return NoPathExists;
        }
        
        // reconstruct path
        for(TVertexID at = dest; at != src; at = predecessor[at]) {
            // check for unreachable vertex (indicated by max value)
            if(predecessor[at] == std::numeric_limits<TVertexID>::max()) {
                path.clear(); // no path exists
                return NoPathExists;
            }
            path.push_back(at);
        }
        
        // add the source vertex
        path.push_back(src);
        
        // reverse to get path from src to dest
        std::reverse(path.begin(), path.end());
        
        return distance[dest];
    }
};
// constructor
CDijkstraPathRouter::CDijkstraPathRouter() {
    DImplementation = std::make_unique<SImplementation>();
}
// destructor
CDijkstraPathRouter::~CDijkstraPathRouter() {}
// vertex count
std::size_t CDijkstraPathRouter::VertexCount() const noexcept {
    return DImplementation->VertexCount();
}
// add vertex
CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept {
    return DImplementation->AddVertex(tag);
}
// get vertex tag
std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept {
    return DImplementation->GetVertexTag(id);
}
// add edge
bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept {
    return DImplementation->AddEdge(src, dest, weight, bidir);
}
// precompute
bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept {
    return DImplementation->Precompute(deadline);
}
// find shortest path
double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept {
    return DImplementation->FindShortestPath(src, dest, path);
}