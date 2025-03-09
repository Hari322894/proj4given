#ifndef PATH_ROUTER_H
#define PATH_ROUTER_H

#include <chrono>
#include <memory>
#include <limits>
#include <vector>

class CPathRouter {
public:
    using TVertexID = int;
    static constexpr double NoPathExists = std::numeric_limits<double>::infinity();
    static constexpr TVertexID InvalidVertexID = -1;

    virtual ~CPathRouter() = default;
    virtual TVertexID AddVertex(std::any tag) noexcept = 0;
    virtual std::any GetVertexTag(TVertexID id) const noexcept = 0;
    virtual bool AddEdge(TVertexID src, TVertexID dest, double weight, bool bidirectional) noexcept = 0;
    virtual double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID>& path) noexcept = 0;

    static std::shared_ptr<CPathRouter> CreatePathRouter();
};

#endif // PATH_ROUTER_H