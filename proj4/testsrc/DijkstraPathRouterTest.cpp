#include <gtest/gtest.h>
#include "DijkstraPathRouter.h"
#include<iostream>
#include<string>

TEST(DijkstraPathRouter, AddVertexTest){
    CDijkstraPathRouter DijkstraPathRouter;
    int id1 = DijkstraPathRouter.AddVertex('A');
    int id2 = DijkstraPathRouter.AddVertex(1);
    int id3 = DijkstraPathRouter.AddVertex(3.2);
    ASSERT_EQ(DijkstraPathRouter.VertexCount(), 3);
}

TEST(DijkstraPathRouter, FindVertexTest){
    CDijkstraPathRouter DijkstraPathRouter;
    int id1 = DijkstraPathRouter.AddVertex('A');
    int id2 = DijkstraPathRouter.AddVertex('B');
    ASSERT_EQ(DijkstraPathRouter.VertexCount(), 2);
    EXPECT_EQ(std::any_cast<char>(DijkstraPathRouter.GetVertexTag(id1)), 'A');
    EXPECT_EQ(std::any_cast<char>(DijkstraPathRouter.GetVertexTag(id2)), 'B');
}

TEST(DijkstraPathRouter, AddEdgeTest){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    Vertices.push_back(DijkstraPathRouter.AddVertex(0));
    Vertices.push_back(DijkstraPathRouter.AddVertex(1));
    Vertices.push_back(DijkstraPathRouter.AddVertex(2));
    Vertices.push_back(DijkstraPathRouter.AddVertex(3));
    Vertices.push_back(DijkstraPathRouter.AddVertex(4));

    ASSERT_EQ(DijkstraPathRouter.VertexCount(), 5);
    EXPECT_TRUE(DijkstraPathRouter.AddEdge(Vertices[0],Vertices[2],1)); 
    EXPECT_TRUE(!DijkstraPathRouter.AddEdge(Vertices[0],Vertices[1],-5));
}


TEST(DijkstraPathRouter, ShortestSimpleTest){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    Vertices.push_back(DijkstraPathRouter.AddVertex(0));
    Vertices.push_back(DijkstraPathRouter.AddVertex(1));

    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[1],5);

    std::vector< CPathRouter::TVertexID > Path;
    std::vector< CPathRouter::TVertexID > ExpectedPath = {Vertices[0],Vertices[1]};


    EXPECT_EQ(5.0 ,DijkstraPathRouter.FindShortestPath(Vertices[0],Vertices[1],Path));
    EXPECT_EQ(Path,ExpectedPath);
}


TEST(DijkstraPathRouter, VerySimpleTest){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    Vertices.push_back(DijkstraPathRouter.AddVertex(0));
    Vertices.push_back(DijkstraPathRouter.AddVertex(1));
    Vertices.push_back(DijkstraPathRouter.AddVertex(2));
    Vertices.push_back(DijkstraPathRouter.AddVertex(3));
    Vertices.push_back(DijkstraPathRouter.AddVertex(4));

    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[1],5);
    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[2],1);
    DijkstraPathRouter.AddEdge(Vertices[1],Vertices[4],3);
    DijkstraPathRouter.AddEdge(Vertices[2],Vertices[3],2);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[0],6);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[1],1);
    DijkstraPathRouter.AddEdge(Vertices[4],Vertices[3],7);
    std::vector< CPathRouter::TVertexID > Path;
    std::vector< CPathRouter::TVertexID > ExpectedPath = {Vertices[0],Vertices[2],Vertices[3],Vertices[1],Vertices[4]};


    EXPECT_EQ(7.0,DijkstraPathRouter.FindShortestPath(Vertices[0],Vertices[4],Path));
    EXPECT_EQ(Path,ExpectedPath);
}

TEST(DijkstraPathRouter, TwoShortestPath){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    Vertices.push_back(DijkstraPathRouter.AddVertex(0));
    Vertices.push_back(DijkstraPathRouter.AddVertex(1));
    Vertices.push_back(DijkstraPathRouter.AddVertex(2));
    Vertices.push_back(DijkstraPathRouter.AddVertex(3));
    Vertices.push_back(DijkstraPathRouter.AddVertex(4));

    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[1],4);//shorter by one. now it has same path weight as past one
    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[2],1);
    DijkstraPathRouter.AddEdge(Vertices[1],Vertices[4],3);
    DijkstraPathRouter.AddEdge(Vertices[2],Vertices[3],2);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[0],6);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[1],1);
    DijkstraPathRouter.AddEdge(Vertices[4],Vertices[3],7);
    std::vector< CPathRouter::TVertexID > Path;
    std::vector< CPathRouter::TVertexID > ExpectedPath = {Vertices[0],Vertices[1],Vertices[4]};


    EXPECT_EQ(7.0,DijkstraPathRouter.FindShortestPath(Vertices[0],Vertices[4],Path));
    EXPECT_EQ(Path,ExpectedPath);
}

TEST(DijkstraPathRouter, NegativeEdge){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    Vertices.push_back(DijkstraPathRouter.AddVertex(0));
    Vertices.push_back(DijkstraPathRouter.AddVertex('k'));
    Vertices.push_back(DijkstraPathRouter.AddVertex(2));
    Vertices.push_back(DijkstraPathRouter.AddVertex('lol'));
    Vertices.push_back(DijkstraPathRouter.AddVertex(8.99));

    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[1],5);
    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[2],-1);//doesnt get created. has to reroute path
    DijkstraPathRouter.AddEdge(Vertices[1],Vertices[4],3);
    DijkstraPathRouter.AddEdge(Vertices[2],Vertices[3],2);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[0],6);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[1],1);
    DijkstraPathRouter.AddEdge(Vertices[4],Vertices[3],7);
    std::vector< CPathRouter::TVertexID > Path;
    std::vector< CPathRouter::TVertexID > ExpectedPath = {Vertices[0],Vertices[1],Vertices[4]};


    EXPECT_EQ(8.0,DijkstraPathRouter.FindShortestPath(Vertices[0],Vertices[4],Path));
    EXPECT_EQ(Path,ExpectedPath);
}

TEST(DijkstraPathRouter, BiDirisTrue){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    Vertices.push_back(DijkstraPathRouter.AddVertex(0));
    Vertices.push_back(DijkstraPathRouter.AddVertex(1));
    Vertices.push_back(DijkstraPathRouter.AddVertex(2));
    Vertices.push_back(DijkstraPathRouter.AddVertex(3));
    Vertices.push_back(DijkstraPathRouter.AddVertex(4));

    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[1],5);
    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[2],1,true);
    DijkstraPathRouter.AddEdge(Vertices[1],Vertices[4],3,true);
    DijkstraPathRouter.AddEdge(Vertices[2],Vertices[3],2);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[0],6);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[1],1);
    DijkstraPathRouter.AddEdge(Vertices[4],Vertices[3],7);
    std::vector< CPathRouter::TVertexID > Path;
    std::vector< CPathRouter::TVertexID > ExpectedPath = {Vertices[0],Vertices[2],Vertices[3],Vertices[1],Vertices[4]};


    EXPECT_EQ(7.0,DijkstraPathRouter.FindShortestPath(Vertices[0],Vertices[4],Path));
    EXPECT_EQ(Path,ExpectedPath);
}

TEST(DijkstraPathRouter, NoPathTest){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    Vertices.push_back(DijkstraPathRouter.AddVertex(0));
    Vertices.push_back(DijkstraPathRouter.AddVertex(1));
    Vertices.push_back(DijkstraPathRouter.AddVertex(2));
    Vertices.push_back(DijkstraPathRouter.AddVertex(3));
    Vertices.push_back(DijkstraPathRouter.AddVertex(4));

    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[1],5);
    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[2],1);
    DijkstraPathRouter.AddEdge(Vertices[2],Vertices[3],2);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[0],6);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[1],1);
    std::vector< CPathRouter::TVertexID > Path;


    EXPECT_EQ(CPathRouter::NoPathExists,DijkstraPathRouter.FindShortestPath(Vertices[0],Vertices[4],Path));
}


TEST(DijkstraPathRouter, SimpleTest){
    CDijkstraPathRouter DijkstraPathRouter;
    std::vector< CPathRouter::TVertexID > Vertices;
    for(std::size_t Index = 0; Index < 6; Index++){
        Vertices.push_back(DijkstraPathRouter.AddVertex(Index));
        EXPECT_EQ(Index, std::any_cast<std::size_t>(DijkstraPathRouter.GetVertexTag(Vertices.back())));
    }
    DijkstraPathRouter.AddEdge(Vertices[0],Vertices[4],2);
    DijkstraPathRouter.AddEdge(Vertices[1],Vertices[3],10);
    DijkstraPathRouter.AddEdge(Vertices[2],Vertices[0],0);
    DijkstraPathRouter.AddEdge(Vertices[2],Vertices[1],3);
    DijkstraPathRouter.AddEdge(Vertices[3],Vertices[2],7);
    DijkstraPathRouter.AddEdge(Vertices[4],Vertices[5],100);
    DijkstraPathRouter.AddEdge(Vertices[5],Vertices[3],5);
    std::vector< CPathRouter::TVertexID > Path;
    std::vector< CPathRouter::TVertexID > ExpectedPath = {Vertices[2],Vertices[1],Vertices[3]};
    EXPECT_EQ(13.0,DijkstraPathRouter.FindShortestPath(Vertices[2],Vertices[3],Path));
    EXPECT_EQ(Path,ExpectedPath);
}