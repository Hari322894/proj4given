#include "CSVBusSystem.h"
#include "StringDataSource.h"
#include "DSVReader.h"
#include "gtest/gtest.h"
#include <string>
#include <vector>
#include <memory>

class MockDSVReader : public CDSVReader {
    private:
        size_t CurrentRow;
        std::vector<std::vector<std::string>> MockData;
        
    public:
        // Initialize with the correct base class constructor
        MockDSVReader(const std::vector<std::vector<std::string>>& data) 
            : CDSVReader(std::make_shared<CStringDataSource>(""), ','), 
              CurrentRow(0),
              MockData(data) {}
        
        // Use override to ensure we're actually overriding the base class method
        bool ReadRow(std::vector<std::string> &row) override {
            if (CurrentRow < MockData.size()) {
                row = MockData[CurrentRow];
                CurrentRow++;
                return true;
            }
            return false;
        }
        
        // Also override the End method to make it compatible
        bool End() const override {
            return CurrentRow >= MockData.size();
        }
        
        // Reset for reuse in tests
        void Reset() {
            CurrentRow = 0;
        }
    };

// Test fixture for CSVBusSystem
class CSVBusSystemTest : public ::testing::Test {
protected:
    // Stop data in the format expected by the implementation: stop_id, node_id
    std::vector<std::vector<std::string>> TestStopData = {
        {"1", "100"},
        {"2", "200"}
    };
    
    // Route data in the format expected by the implementation: route_name, stop_id
    std::vector<std::vector<std::string>> TestRouteData = {
        {"Route1", "1"},
        {"Route1", "2"},
        {"Route2", "2"},
        {"Route2", "1"}
    };
};

// Test basic stop functionality
TEST_F(CSVBusSystemTest, StopCountAndAccess) {
    // Create mock readers
    auto stopReader = std::make_shared<MockDSVReader>(TestStopData);
    auto routeReader = std::make_shared<MockDSVReader>(TestRouteData);
    
    // Create bus system
    CCSVBusSystem busSystem(stopReader, routeReader);
    
    // Test stop count
    EXPECT_EQ(busSystem.StopCount(), 2);
    
    // Test stop by index
    auto stop1 = busSystem.StopByIndex(0);
    ASSERT_NE(stop1, nullptr);
    EXPECT_EQ(stop1->ID(), 1);
    
    auto stop2 = busSystem.StopByIndex(1);
    ASSERT_NE(stop2, nullptr);
    EXPECT_EQ(stop2->ID(), 2);
    
    // Test stop by ID
    auto stopById1 = busSystem.StopByID(1);
    ASSERT_NE(stopById1, nullptr);
    EXPECT_EQ(stopById1->NodeID(), 100);
    
    auto stopById2 = busSystem.StopByID(2);
    ASSERT_NE(stopById2, nullptr);
    EXPECT_EQ(stopById2->NodeID(), 200);
    
    // Test invalid stop index
    EXPECT_EQ(busSystem.StopByIndex(2), nullptr);
    
    // Test invalid stop ID
    EXPECT_EQ(busSystem.StopByID(3), nullptr);
}

// Test route functionality
TEST_F(CSVBusSystemTest, RouteCountAndAccess) {
    // Create mock readers
    auto stopReader = std::make_shared<MockDSVReader>(TestStopData);
    auto routeReader = std::make_shared<MockDSVReader>(TestRouteData);
    
    // Create bus system
    CCSVBusSystem busSystem(stopReader, routeReader);
    
    // Test route count
    EXPECT_EQ(busSystem.RouteCount(), 2);
    
    // Test route by index
    auto route1 = busSystem.RouteByIndex(0);
    ASSERT_NE(route1, nullptr);
    EXPECT_EQ(route1->Name(), "Route1");
    EXPECT_EQ(route1->StopCount(), 2);
    EXPECT_EQ(route1->GetStopID(0), 1);
    EXPECT_EQ(route1->GetStopID(1), 2);
    
    auto route2 = busSystem.RouteByIndex(1);
    ASSERT_NE(route2, nullptr);
    EXPECT_EQ(route2->Name(), "Route2");
    EXPECT_EQ(route2->StopCount(), 2);
    EXPECT_EQ(route2->GetStopID(0), 2);
    EXPECT_EQ(route2->GetStopID(1), 1);
    
    // Test route by name
    auto routeByName1 = busSystem.RouteByName("Route1");
    ASSERT_NE(routeByName1, nullptr);
    EXPECT_EQ(routeByName1->StopCount(), 2);
    
    auto routeByName2 = busSystem.RouteByName("Route2");
    ASSERT_NE(routeByName2, nullptr);
    EXPECT_EQ(routeByName2->StopCount(), 2);
    
    // Test invalid route index
    EXPECT_EQ(busSystem.RouteByIndex(2), nullptr);
    
    // Test invalid route name
    EXPECT_EQ(busSystem.RouteByName("InvalidRoute"), nullptr);
}

// Test with empty data
TEST_F(CSVBusSystemTest, EmptyData) {
    // Create empty mock readers
    auto emptyStopReader = std::make_shared<MockDSVReader>(std::vector<std::vector<std::string>>());
    auto emptyRouteReader = std::make_shared<MockDSVReader>(std::vector<std::vector<std::string>>());
    
    // Create bus system
    CCSVBusSystem busSystem(emptyStopReader, emptyRouteReader);
    
    // Test counts
    EXPECT_EQ(busSystem.StopCount(), 0);
    EXPECT_EQ(busSystem.RouteCount(), 0);
    
    // Test access
    EXPECT_EQ(busSystem.StopByIndex(0), nullptr);
    EXPECT_EQ(busSystem.StopByID(1), nullptr);
    EXPECT_EQ(busSystem.RouteByIndex(0), nullptr);
    EXPECT_EQ(busSystem.RouteByName("Route1"), nullptr);
}