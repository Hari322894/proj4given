#include "DijkstraPathRouter.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <queue>
#include <algorithm>

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

   // std::vector<size_t> ListOfVertices;//A vector of index or IDs
    std::vector< std::shared_ptr< IndVertex > > AllVertices;
   // std::unordered_map<TVertexID, std::shared_ptr< IndVertex > > MapOfVertices;//a map of the index(orID) to the IndVertex
    size_t IndexKeeper = -1;
    



    
    SImplementation(){

    };

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
            if (bidir){
                //std::cout<<"BIDIR IS TRUE"<<std::endl;
                AllVertices[src]->ConnectedIDs.push_back(dest);
            }
            AllVertices[src]->MapOfWeights.insert({dest,weight});
            AllVertices[src]->ConnectedIDs.push_back(dest);
            return true;
        }
        return false;
    };
    
    bool Precompute(std::chrono::steady_clock::time_point deadline) {
        return true;
    };

    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) {
        std::unordered_map <TVertexID, std::pair<double, TVertexID>> DP;
        std::priority_queue <TVertexID> pq;
        for (int i = 0; i < VertexCount(); i++)
        {
            std::pair<double, TVertexID> V;
            V.first = std::numeric_limits<double>::infinity();
            V.second = -1;
            DP.insert({AllVertices[i]->GetVertexID(), V});
        }

        DP[src].first = 0;
        DP[src].second = src;

        pq.push(src);
        //finish initialize

        while(!pq.empty())
        {
            TVertexID u = pq.top();
            pq.pop();
            for(int i = 0; i < AllVertices[u]->ConnectedIDCount(); i++)
            {
                TVertexID v = AllVertices[u]->GetConnectedVertexIDs()[i];
                double w = AllVertices[u]->GetWeight(AllVertices[u]->GetConnectedVertexIDs()[i]);

                if(DP[v].first > DP[u].first + w)
                {
                    DP[v].first = DP[u].first + w;
                    DP[v].second = u;
                    pq.push(v);
                }
            }
        }
        
        //check the path
        TVertexID current = dest;
        while(current != src)
        {
            path.push_back(current);
            current = DP[current].second;
        }
        path.push_back(current);
        reverse(path.begin(),path.end());

        if(DP[dest].first == std::numeric_limits<double>::infinity())
        {
            return NoPathExists;
        }
        return DP[dest].first;
    };

};
//---------------------------------------------
CDijkstraPathRouter::CDijkstraPathRouter(){
    DImplementation = std::make_unique<SImplementation>();

};

CDijkstraPathRouter::~CDijkstraPathRouter(){
    
};

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