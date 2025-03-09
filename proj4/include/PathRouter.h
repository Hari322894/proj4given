#ifndef PATHROUTER_H
#define PATHROUTER_H

#include <vector>
#include <unordered_map>
#include <chrono>
#include <memory>
#include <any>
#include <limits>

class CPathRouter {
public:
    using TVertexID = int; // Assuming TVertexID is an integer based on typical usage
    using TEdgeLabel = int; // Assuming TEdgeLabel is an integer based on typical usage
    static constexpr double NoPathExists = std::numeric_limits<double>::infinity();
    static constexpr TVertexID InvalidVertexID = -1;

    virtual ~CPathRouter() {}

    virtual void AddNode(TVertexID node) noexcept = 0;
    virtual void AddEdge(TVertexID src, TVertexID dest, double weight) noexcept = 0;
    virtual void AddEdge(TVertexID src, TVertexID dest, double weight, TEdgeLabel label) noexcept = 0;
    virtual double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path) noexcept = 0;
    virtual double FindShortestPath(TVertexID src, TVertexID dest, std::vector<TVertexID> &path, std::vector<TEdgeLabel> &labels) noexcept = 0;
    virtual bool Precompute(std::chrono::steady_clock::time_point deadline) noexcept = 0;

    static std::shared_ptr<CPathRouter> CreatePathRouter();
};

#endif