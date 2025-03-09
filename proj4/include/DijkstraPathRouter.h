#ifndef DIJKSTRAPATHROUTER_H
#define DIJKSTRAPATHROUTER_H

#include "PathRouter.h"
#include <memory>
#include <any>

class CDijkstraPathRouter : public CPathRouter {
private:
    struct SImplementation;
    std::unique_ptr<SImplementation> DImplementation;

public:
    CDijkstraPathRouter();
    ~CDijkstraPathRouter();

    void AddNode(TVertexID node) noexcept override;
    void AddEdge(TVertexID src, TVertexID dest, double weight) noexcept override;
    void AddEdge(TVertexID src, TVertexID dest, double weight, TEdgeLabel label) noexcept override;
    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept override;
    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path, std::vector<TEdgeLabel> &labels) noexcept override;
    bool Precompute(std::chrono::steady_clock::time_point deadline) noexcept override;
};

#endif // DIJKSTRA_PATH_ROUTER_H