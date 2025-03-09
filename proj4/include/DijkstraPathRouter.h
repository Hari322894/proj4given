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

    std::size_t VertexCount() const noexcept;
    TVertexID AddVertex(std::any tag) noexcept override;
    std::any GetVertexTag(TVertexID id) const noexcept override;
    bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidir = false) noexcept override;
    bool Precompute(std::chrono::steady_clock::time_point deadline) noexcept override;
    double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept override;
};

#endif