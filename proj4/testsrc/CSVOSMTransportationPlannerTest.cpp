#include <gtest/gtest.h>
#include "XMLReader.h"
#include "StringUtils.h"
#include "StringDataSource.h"
#include "OpenStreetMap.h"
#include "CSVBusSystem.h"
#include "TransportationPlannerConfig.h"
#include "DijkstraTransportationPlanner.h"
#include "GeographicUtils.h"

TEST(CSVOSMTransporationPlanner, SimpleTest){
    auto InStreamOSM = std::make_shared<CStringDataSource>( "<?xml version='1.0' encoding='UTF-8'?>"
                                                            "<osm version=\"0.6\" generator=\"osmconvert 0.8.5\">"
                                                            "</osm>");
    auto InStreamStops = std::make_shared<CStringDataSource>("stop_id,node_id");
    auto InStreamRoutes = std::make_shared<CStringDataSource>("route,stop_id");
    auto XMLReader = std::make_shared<CXMLReader>(InStreamOSM);
    auto CSVReaderStops = std::make_shared<CDSVReader>(InStreamStops,',');
    auto CSVReaderRoutes = std::make_shared<CDSVReader>(InStreamRoutes,',');
    auto StreetMap = std::make_shared<COpenStreetMap>(XMLReader);
    auto BusSystem = std::make_shared<CCSVBusSystem>(CSVReaderStops, CSVReaderRoutes);
    auto Config = std::make_shared<STransportationPlannerConfig>(StreetMap,BusSystem);
    CDijkstraTransportationPlanner Planner(Config);
    std::vector< CTransportationPlanner::TNodeID > ShortestPath;
    std::vector< CTransportationPlanner::TTripStep > FastestPath;
    EXPECT_EQ(Planner.FindShortestPath(0,1,ShortestPath),CPathRouter::NoPathExists);
    EXPECT_EQ(Planner.FindFastestPath(0,1,FastestPath),CPathRouter::NoPathExists);
}

TEST(CSVOSMTransporationPlanner, SortedNodeTest){
    auto InStreamOSM = std::make_shared<CStringDataSource>( "<?xml version='1.0' encoding='UTF-8'?>"
                                                            "<osm version=\"0.6\" generator=\"osmconvert 0.8.5\">"
                                                            "<node id=\"4\" lat=\"38.5\" lon=\"-121.7\"/>"
                                                            "<node id=\"2\" lat=\"38.6\" lon=\"-121.7\"/>"
                                                            "<node id=\"1\" lat=\"38.6\" lon=\"-121.8\"/>"
                                                            "<node id=\"3\" lat=\"38.5\" lon=\"-121.8\"/>"
                                                            "<way id=\"10\">"
                                                            "<nd ref=\"1\"/>"
                                                            "<nd ref=\"2\"/>"
                                                            "<nd ref=\"3\"/>"
                                                            "<tag k=\"oneway\" v=\"yes\"/>"
                                                            "</way>"
                                                            "<way id=\"11\">"
                                                            "<nd ref=\"3\"/>"
                                                            "<nd ref=\"4\"/>"
                                                            "<nd ref=\"1\"/>"
                                                            "<tag k=\"oneway\" v=\"yes\"/>"
                                                            "</way>"
                                                            "</osm>");
    // 6.9090909 mil 1 -> 2
    // 5.4 mile  2 -> 3
    // 6.9090909 mil 3 -> 4
    // 5.407386 mi 4 -> 1
    auto InStreamStops = std::make_shared<CStringDataSource>("stop_id,node_id");
    auto InStreamRoutes = std::make_shared<CStringDataSource>("route,stop_id");
    auto XMLReader = std::make_shared<CXMLReader>(InStreamOSM);
    auto CSVReaderStops = std::make_shared<CDSVReader>(InStreamStops,',');
    auto CSVReaderRoutes = std::make_shared<CDSVReader>(InStreamRoutes,',');
    auto StreetMap = std::make_shared<COpenStreetMap>(XMLReader);
    auto BusSystem = std::make_shared<CCSVBusSystem>(CSVReaderStops, CSVReaderRoutes);
    auto Config = std::make_shared<STransportationPlannerConfig>(StreetMap,BusSystem);
    CDijkstraTransportationPlanner Planner(Config);
    EXPECT_EQ(Planner.NodeCount(),4);
    for(std::size_t Index = 0; Index < Planner.NodeCount(); Index++){
        auto Node = Planner.SortedNodeByIndex(Index);
        ASSERT_TRUE(Node);
        EXPECT_EQ(Node->ID(),Index+1);
    }
}

