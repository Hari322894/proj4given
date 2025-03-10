#include "DijkstraPathRouter.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <queue>
#include <algorithm>
#include <limits>

struct CDijkstraPathRouter::SImplementation{
    struct IndVertex{
        TVertexID ThisVertexID;
        std::any ThisVertexTag;
        std::vector< TVertexID > ConnectedIDs;
        std::unordered_map<TVertexID,double> MapOfWeights;

        ~IndVertex(){};
        TVertexID GetVertexID(){
            return ThisVertexID;
        }

        std::any GetThisVertexTag(){
            return ThisVertexTag;
        }

        std::size_t ConnectedIDCount(){
            return ConnectedIDs.size();
        }
        
        std::vector< TVertexID > GetConnectedVertexIDs(){
            return ConnectedIDs;
        }

        double GetWeight(TVertexID &id){
            auto Search = MapOfWeights.find(id);

            if(Search == MapOfWeights.end()){
                return false;
            }
            return Search->second;
        }
    };

    std::vector< std::shared_ptr< IndVertex > > AllVertices;
    size_t IndexKeeper = -1;
    
    SImplementation(){};

    std::size_t VertexCount() const{
        return AllVertices.size();
    };

    TVertexID AddVertex(std::any tag){
        auto NewVertex = std::make_shared<IndVertex>();
        IndexKeeper += 1;
        NewVertex->ThisVertexID = IndexKeeper;
        NewVertex->ThisVertexTag = tag;
        AllVertices.push_back(NewVertex);
        return IndexKeeper;
    };
    
    std::any GetVertexTag(TVertexID id) const{
        return AllVertices[id]->GetThisVertexTag();
    };
    
    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir = false) {
        if (weight > 0)
        {
            AllVertices[src]->MapOfWeights[dest] = weight;
            AllVertices[src]->ConnectedIDs.push_back(dest);
            
            if (bidir){
                AllVertices[dest]->MapOfWeights[src] = weight;
                AllVertices[dest]->ConnectedIDs.push_back(src);
            }
            return true;
        }
        return false;
    };
    
    bool Precompute(std::chrono::steady_clock::time_point deadline) {
        return true;
    };

    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) {
        // Clear the path vector first
        path.clear();
        
        // Check if src and dest are valid vertices
        if(src >= VertexCount() || dest >= VertexCount()) {
            return NoPathExists;
        }
        
        // Create a min-heap priority queue
        // Pair of (distance, vertex)
        typedef std::pair<double, TVertexID> DistVertex;
        std::priority_queue<DistVertex, std::vector<DistVertex>, std::greater<DistVertex>> pq;
        
        // Create distance array and predecessor array
        std::vector<double> distance(VertexCount(), std::numeric_limits<double>::infinity());
        std::vector<TVertexID> predecessor(VertexCount(), std::numeric_limits<TVertexID>::max()); // Use max value instead of -1
        
        // Initialize source distance and add to queue
        distance[src] = 0;
        pq.push({0, src});
        
        // Dijkstra's algorithm
        while(!pq.empty()) {
            double dist = pq.top().first;
            TVertexID u = pq.top().second;
            pq.pop();
            
            // Skip if we've found a better path already
            if(dist > distance[u]) continue;
            
            // If we reached destination, break
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
        
        // Check if path exists
        if(distance[dest] == std::numeric_limits<double>::infinity()) {
            return NoPathExists;
        }
        
        // Reconstruct path
        for(TVertexID at = dest; at != src; at = predecessor[at]) {
            // Check for unreachable vertex (indicated by max value)
            if(predecessor[at] == std::numeric_limits<TVertexID>::max()) {
                path.clear(); // No path exists
                return NoPathExists;
            }
            path.push_back(at);
        }
        
        // Add the source vertex
        path.push_back(src);
        
        // Reverse to get path from src to dest
        std::reverse(path.begin(), path.end());
        
        return distance[dest];
    };
};

//---------------------------------------------
CDijkstraPathRouter::CDijkstraPathRouter(){
    DImplementation = std::make_unique<SImplementation>();
};

CDijkstraPathRouter::~CDijkstraPathRouter(){};

std::size_t CDijkstraPathRouter::VertexCount() const noexcept{
    return DImplementation->VertexCount();
};

CPathRouter::TVertexID CDijkstraPathRouter::AddVertex(std::any tag) noexcept{
    return DImplementation->AddVertex(tag);
};

std::any CDijkstraPathRouter::GetVertexTag(TVertexID id) const noexcept{
    return DImplementation->GetVertexTag(id);
};

bool CDijkstraPathRouter::AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir) noexcept{
    return DImplementation->AddEdge(src, dest, weight, bidir);
};

bool CDijkstraPathRouter::Precompute(std::chrono::steady_clock::time_point deadline) noexcept{
    return DImplementation->Precompute(deadline);
};

double CDijkstraPathRouter::FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept{
    return DImplementation->FindShortestPath(src, dest, path);
};