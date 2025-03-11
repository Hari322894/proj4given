#include <gtest/gtest.h>
#include "XMLReader.h"
#include "StringUtils.h"
#include "StringDataSource.h"
#include "DSVReader.h"
#include "CSVBusSystem.h"

TEST(CSVBusSystem, EmptyDataTest) {
    // Create empty data sources
    auto stopDataSource = std::make_shared<CStringDataSource>("stop_id,node_id");
    auto routeDataSource = std::make_shared<CStringDataSource>("route_name,stop_id");
    
    // Create CSV readers
    auto stopReader = std::make_shared<CDSVReader>(stopDataSource, ',');
    auto routeReader = std::make_shared<CDSVReader>(routeDataSource, ',');
    
    // Create bus system with empty data
    CCSVBusSystem busSystem(stopReader, routeReader);
    
    // Verify empty system properties
    EXPECT_EQ(busSystem.StopCount(), 0);
    EXPECT_EQ(busSystem.RouteCount(), 0);
    EXPECT_EQ(busSystem.StopByIndex(0), nullptr);
    EXPECT_EQ(busSystem.RouteByIndex(0), nullptr);
}

TEST(CSVBusSystem, StopsOnlyTest) {
    // Create stop data with header and two stops
    auto stopDataSource = std::make_shared<CStringDataSource>(
        "stop_id,node_id\n"
        "42,1042\n"
        "73,1073"
    );
    auto routeDataSource = std::make_shared<CStringDataSource>("route_name,stop_id");
    
    // Create CSV readers
    auto stopReader = std::make_shared<CDSVReader>(stopDataSource, ',');
    auto routeReader = std::make_shared<CDSVReader>(routeDataSource, ',');
    
    // Create bus system with stops but no routes
    CCSVBusSystem busSystem(stopReader, routeReader);
    
    // Verify system has correct stops
    EXPECT_EQ(busSystem.StopCount(), 2);
    EXPECT_EQ(busSystem.RouteCount(), 0);
    
    // Test first stop access by index and ID
    auto firstStopByIndex = busSystem.StopByIndex(0);
    auto firstStopById = busSystem.StopByID(42);
    ASSERT_NE(firstStopByIndex, nullptr);
    ASSERT_NE(firstStopById, nullptr);
    EXPECT_EQ(firstStopByIndex, firstStopById);
    EXPECT_EQ(firstStopByIndex->ID(), 42);
    EXPECT_EQ(firstStopByIndex->NodeID(), 1042);
    
    // Test second stop access by index and ID
    auto secondStopByIndex = busSystem.StopByIndex(1);
    auto secondStopById = busSystem.StopByID(73);
    ASSERT_NE(secondStopByIndex, nullptr);
    ASSERT_NE(secondStopById, nullptr);
    EXPECT_EQ(secondStopByIndex, secondStopById);
    EXPECT_EQ(secondStopByIndex->ID(), 73);
    EXPECT_EQ(secondStopByIndex->NodeID(), 1073);
    
    // Test invalid accesses
    EXPECT_EQ(busSystem.StopByIndex(2), nullptr);
    EXPECT_EQ(busSystem.StopByID(99), nullptr);
}

TEST(CSVBusSystem, CompleteSystemTest) {
    // Create stop data with header and multiple stops
    auto stopDataSource = std::make_shared<CStringDataSource>(
        "stop_id,node_id\n"
        "10,2010\n"
        "20,2020\n"
        "30,2030"
    );
    
    // Create route data with header and routes
    auto routeDataSource = std::make_shared<CStringDataSource>(
        "route_name,stop_id\n"
        "Blue,10\n"
        "Blue,20\n"
        "Red,20\n"
        "Red,30\n"
        "Red,10"
    );
    
    // Create CSV readers
    auto stopReader = std::make_shared<CDSVReader>(stopDataSource, ',');
    auto routeReader = std::make_shared<CDSVReader>(routeDataSource, ',');
    
    // Create complete bus system with stops and routes
    CCSVBusSystem busSystem(stopReader, routeReader);
    
    // Verify system has correct stops and routes
    EXPECT_EQ(busSystem.StopCount(), 3);
    EXPECT_EQ(busSystem.RouteCount(), 2);
    
    // Test route access by name
    auto blueRoute = busSystem.RouteByName("Blue");
    ASSERT_NE(blueRoute, nullptr);
    EXPECT_EQ(blueRoute->Name(), "Blue");
    EXPECT_EQ(blueRoute->StopCount(), 2);
    EXPECT_EQ(blueRoute->GetStopID(0), 10);
    EXPECT_EQ(blueRoute->GetStopID(1), 20);
    
    // Test route access by index
    auto redRoute = busSystem.RouteByIndex(1);
    ASSERT_NE(redRoute, nullptr);
    EXPECT_EQ(redRoute->Name(), "Red");
    EXPECT_EQ(redRoute->StopCount(), 3);
    EXPECT_EQ(redRoute->GetStopID(0), 20);
    EXPECT_EQ(redRoute->GetStopID(1), 30);
    EXPECT_EQ(redRoute->GetStopID(2), 10);
    
    // Verify route access by both methods matches
    EXPECT_EQ(busSystem.RouteByIndex(0), busSystem.RouteByName("Blue"));
    EXPECT_EQ(busSystem.RouteByIndex(1), busSystem.RouteByName("Red"));
    
    // Test invalid route accesses
    EXPECT_EQ(busSystem.RouteByIndex(2), nullptr);
    EXPECT_EQ(busSystem.RouteByName("Green"), nullptr);
}