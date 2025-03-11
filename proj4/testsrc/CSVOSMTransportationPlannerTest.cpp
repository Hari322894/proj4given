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
