#include "DijkstraPathRouter.h"
#include <gtest/gtest.h>
#include <string>
#include <chrono>
#include <algorithm>

TEST(DijkstraPathRouterTest, EmptyRouter) {
    CDijkstraPathRouter Router;
    
    EXPECT_EQ(Router.VertexCount(), 0);
    
    std::vector<CPathRouter::TVertexID> Path;
    EXPECT_EQ(Router.FindShortestPath(0, 0, Path), CPathRouter::NoPathExists);
    EXPECT_TRUE(Path.empty());
}

TEST(DijkstraPathRouterTest, SingleVertex) {
    CDijkstraPathRouter Router;
    auto VertexID = Router.AddVertex("Test");
    
    EXPECT_EQ(Router.VertexCount(), 1);
    EXPECT_EQ(VertexID, 0);
    
    std::any Tag = Router.GetVertexTag(VertexID);
    EXPECT_TRUE(Tag.has_value());
    EXPECT_EQ(std::any_cast<std::string>(Tag), "Test");
    
    std::vector<CPathRouter::TVertexID> Path;
    EXPECT_EQ(Router.FindShortestPath(VertexID, VertexID, Path), 0.0);
    EXPECT_EQ(Path.size(), 1);
    EXPECT_EQ(Path[0], VertexID);
}

TEST(DijkstraPathRouterTest, TwoVertices) {
    CDijkstraPathRouter Router;
    auto VertexID1 = Router.AddVertex("Vertex1");
    auto VertexID2 = Router.AddVertex("Vertex2");
    
    EXPECT_EQ(Router.VertexCount(), 2);
    
    std::vector<CPathRouter::TVertexID> Path;
    EXPECT_EQ(Router.FindShortestPath(VertexID1, VertexID2, Path), CPathRouter::NoPathExists);
    EXPECT_TRUE(Path.empty());
    
    // Add edge
    EXPECT_TRUE(Router.AddEdge(VertexID1, VertexID2, 5.0));
    
    // Test path from 1 to 2
    EXPECT_EQ(Router.FindShortestPath(VertexID1, VertexID2, Path), 5.0);
    EXPECT_EQ(Path.size(), 2);
    EXPECT_EQ(Path[0], VertexID1);
    EXPECT_EQ(Path[1], VertexID2);
    
    // Test no path from 2 to 1 (uni-directional edge)
    Path.clear();
    EXPECT_EQ(Router.FindShortestPath(VertexID2, VertexID1, Path), CPathRouter::NoPathExists);
    EXPECT_TRUE(Path.empty());
    
    // Add bidirectional edge
    EXPECT_TRUE(Router.AddEdge(VertexID1, VertexID2, 3.0, true));
    
    // Test shorter path 1 to 2
    EXPECT_EQ(Router.FindShortestPath(VertexID1, VertexID2, Path), 3.0);
    EXPECT_EQ(Path.size(), 2);
    EXPECT_EQ(Path[0], VertexID1);
    EXPECT_EQ(Path[1], VertexID2);
    
    // Test path from 2 to 1
    Path.clear();
    EXPECT_EQ(Router.FindShortestPath(VertexID2, VertexID1, Path), 3.0);
    EXPECT_EQ(Path.size(), 2);
    EXPECT_EQ(Path[0], VertexID2);
    EXPECT_EQ(Path[1], VertexID1);
}

TEST(DijkstraPathRouterTest, ComplexGraph) {
    CDijkstraPathRouter Router;
    /*
        Create a graph:
        0 --- 1 --- 2
        |     |     |
        3 --- 4 --- 5
        
        With various weights
    */
    auto V0 = Router.AddVertex("V0");
    auto V1 = Router.AddVertex("V1");
    auto V2 = Router.AddVertex("V2");
    auto V3 = Router.AddVertex("V3");
    auto V4 = Router.AddVertex("V4");
    auto V5 = Router.AddVertex("V5");
    
    EXPECT_EQ(Router.VertexCount(), 6);
    
    // Add edges
    Router.AddEdge(V0, V1, 2.0, true);  // 0 <-> 1
    Router.AddEdge(V1, V2, 3.0, true);  // 1 <-> 2
    Router.AddEdge(V0, V3, 1.0, true);  // 0 <-> 3
    Router.AddEdge(V1, V4, 2.0, true);  // 1 <-> 4
    Router.AddEdge(V2, V5, 1.0, true);  // 2 <-> 5
    Router.AddEdge(V3, V4, 4.0, true);  // 3 <-> 4
    Router.AddEdge(V4, V5, 3.0, true);  // 4 <-> 5
    
    // Run precompute
    Router.Precompute(std::chrono::steady_clock::now() + std::chrono::seconds(1));
    
    // Test path from 0 to 5
    std::vector<CPathRouter::TVertexID> Path;
    // Shortest path: 0 -> 1 -> 2 -> 5 (2 + 3 + 1 = 6)
    EXPECT_EQ(Router.FindShortestPath(V0, V5, Path), 6.0);
    EXPECT_EQ(Path.size(), 4);
    EXPECT_EQ(Path[0], V0);
    EXPECT_EQ(Path[1], V1);
    EXPECT_EQ(Path[2], V2);
    EXPECT_EQ(Path[3], V5);
    
    // Test path from 3 to 2
    Path.clear();
    // Shortest path: 3 -> 0 -> 1 -> 2 (1 + 2 + 3 = 6)
    EXPECT_EQ(Router.FindShortestPath(V3, V2, Path), 6.0);
    EXPECT_EQ(Path.size(), 4);
    EXPECT_EQ(Path[0], V3);
    EXPECT_EQ(Path[1], V0);
    EXPECT_EQ(Path[2], V1);
    EXPECT_EQ(Path[3], V2);
}

TEST(DijkstraPathRouterTest, InvalidVertexIDs) {
    CDijkstraPathRouter Router;
    auto V0 = Router.AddVertex("V0");
    auto V1 = Router.AddVertex("V1");
    
    // Add edge
    EXPECT_TRUE(Router.AddEdge(V0, V1, 3.0, true));
    
    // Invalid vertex IDs
    EXPECT_FALSE(Router.AddEdge(V0, 999, 1.0));
    EXPECT_FALSE(Router.AddEdge(999, V1, 1.0));
    EXPECT_FALSE(Router.AddEdge(999, 999, 1.0));
    
    // Invalid negative weight
    EXPECT_FALSE(Router.AddEdge(V0, V1, -1.0));
    
    // Test paths with invalid vertex IDs
    std::vector<CPathRouter::TVertexID> Path;
    EXPECT_EQ(Router.FindShortestPath(V0, 999, Path), CPathRouter::NoPathExists);
    EXPECT_TRUE(Path.empty());
    
    Path.clear();
    EXPECT_EQ(Router.FindShortestPath(999, V1, Path), CPathRouter::NoPathExists);
    EXPECT_TRUE(Path.empty());
    
    // Test GetVertexTag with invalid ID
    std::any Tag = Router.GetVertexTag(999);
    EXPECT_FALSE(Tag.has_value());
}