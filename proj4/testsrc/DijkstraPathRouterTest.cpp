#include "DijkstraPathRouter.h"
#include <gtest/gtest.h>
#include <any>
#include <string>


class DijkstraPathRouterTest : public ::testing::Test {
protected:
// Set up the test fixture.
    void SetUp() override {
        
        router = std::make_unique<CDijkstraPathRouter>(); 
    }
// Tear down the test fixture.
    void TearDown() override {
        
        router.reset(); // clean up after every test
    }
// Declare any objects you plan to use
    std::unique_ptr<CDijkstraPathRouter> router;
};

// test adding vertices
TEST_F(DijkstraPathRouterTest, AddVertex) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));

    EXPECT_EQ(router->VertexCount(), 2);

    // get the tag from std::any and compare
    EXPECT_EQ(std::any_cast<std::string>(router->GetVertexTag(v1)), "A");
    EXPECT_EQ(std::any_cast<std::string>(router->GetVertexTag(v2)), "B");
}

// test adding edges
TEST_F(DijkstraPathRouterTest, AddEdge) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));

    EXPECT_TRUE(router->AddEdge(v1, v2, 1.0));
    EXPECT_FALSE(router->AddEdge(v1, v2, -1.0)); // negative weight should fail
}

// test finding the shortest path
TEST_F(DijkstraPathRouterTest, FindShortestPath) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));
    auto v3 = router->AddVertex(std::string("C"));

    router->AddEdge(v1, v2, 1.0);
    router->AddEdge(v2, v3, 2.0);
    router->AddEdge(v1, v3, 4.0);

    std::vector<CPathRouter::TVertexID> path;
    double distance = router->FindShortestPath(v1, v3, path);

    EXPECT_EQ(distance, 3.0); // shortest path dist should be 3.0
    EXPECT_EQ(path.size(), 3); // path shud contain 3 vertices
    EXPECT_EQ(path[0], v1);    // path shud start at v1
    EXPECT_EQ(path[1], v2);   // path shud go through v2
    EXPECT_EQ(path[2], v3);    // path should end at v3
}

// test no path exists
TEST_F(DijkstraPathRouterTest, NoPathExists) {
    auto v1 = router->AddVertex(std::string("A"));
    auto v2 = router->AddVertex(std::string("B"));
    auto v3 = router->AddVertex(std::string("C"));

    router->AddEdge(v1, v2, 1.0); // no edge between v2 and v3

    std::vector<CPathRouter::TVertexID> path;
    double distance = router->FindShortestPath(v1, v3, path);

    EXPECT_EQ(distance, CDijkstraPathRouter::NoPathExists); // no path should exist
    EXPECT_TRUE(path.empty()); // path should be empty
}