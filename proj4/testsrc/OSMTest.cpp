#include "OpenStreetMap.h"
#include "XMLReader.h"
#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <string>

class MockXMLReader : public CXMLReader {
public:
    std::vector<SXMLEntity> Data;
    size_t CurrentIndex = 0;

    MockXMLReader() : CXMLReader(nullptr) {}

    bool ReadEntity(SXMLEntity &entity, bool skipcdata = false) override {
        if (CurrentIndex < Data.size()) {
            entity = Data[CurrentIndex++];
            return true;
        }
        return false;
    }
};

class OpenStreetMapTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(OpenStreetMapTest, NodeCountTest) {
    auto xmlReader = std::make_shared<MockXMLReader>();
    xmlReader->Data = {
        {SXMLEntity::EType::StartElement, "node"},
        {SXMLEntity::EType::StartElement, "node"},
        {SXMLEntity::EType::StartElement, "node"}
    };

    COpenStreetMap osmMap(xmlReader);
    EXPECT_EQ(osmMap.NodeCount(), 3);
}

TEST_F(OpenStreetMapTest, WayCountTest) {
    auto xmlReader = std::make_shared<MockXMLReader>();
    xmlReader->Data = {
        {SXMLEntity::EType::StartElement, "way"},
        {SXMLEntity::EType::StartElement, "way"}
    };

    COpenStreetMap osmMap(xmlReader);
    EXPECT_EQ(osmMap.WayCount(), 2);
}

TEST_F(OpenStreetMapTest, EmptyMapTest) {
    auto xmlReader = std::make_shared<MockXMLReader>();
    COpenStreetMap osmMap(xmlReader);
    EXPECT_EQ(osmMap.NodeCount(), 0);
    EXPECT_EQ(osmMap.WayCount(), 0);
}

TEST_F(OpenStreetMapTest, MixedElementsTest) {
    auto xmlReader = std::make_shared<MockXMLReader>();
    xmlReader->Data = {
        {SXMLEntity::EType::StartElement, "node"},
        {SXMLEntity::EType::StartElement, "way"},
        {SXMLEntity::EType::StartElement, "node"},
        {SXMLEntity::EType::StartElement, "way"},
        {SXMLEntity::EType::StartElement, "node"}
    };

    COpenStreetMap osmMap(xmlReader);
    EXPECT_EQ(osmMap.NodeCount(), 3);
    EXPECT_EQ(osmMap.WayCount(), 2);
}