#ifndef DIJKSTRA_PATH_ROUTER_H
#define DIJKSTRA_PATH_ROUTER_H

#include "PathRouter.h"
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <any>

class CDijkstraPathRouter : public CPathRouter {
public:
    using TVertexID = int;

    TVertexID AddVertex(std::any tag) noexcept;
    std::any GetVertexTag(TVertexID id) const noexcept;
    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidirectional) noexcept;
    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept;

private:
    struct SVertex {
        std::any Tag;
        std::unordered_map<TVertexID, double> DNeighbors;
    };

    struct SImplementation {
        std::vector<SVertex> DVertices;
    };

    std::unique_ptr<SImplementation> DImplementation;
};

#endif // DIJKSTRA_PATH_ROUTER_H