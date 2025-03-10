#include "XMLReader.h"
#include "XMLWriter.h"
#include "StringDataSource.h"
#include "StringDataSink.h"
#include <gtest/gtest.h>

TEST(XMLTest, BasicReadWrite) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("<tag>data</tag>");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CXMLReader reader(src);
    CXMLWriter writer(sink);

    SXMLEntity entity;
    while (!reader.End()) {
        if (reader.ReadEntity(entity)) {
            writer.WriteEntity(entity);
        }
    }

    EXPECT_EQ(sink->String(), "<tag>data</tag>");
}

TEST(XMLTest, MultipleElements) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("<root><child>value</child></root>");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CXMLReader reader(src);
    CXMLWriter writer(sink);

    SXMLEntity entity;
    while (!reader.End()) {
        if (reader.ReadEntity(entity)) {
            writer.WriteEntity(entity);
        }
    }

    EXPECT_EQ(sink->String(), "<root><child>value</child></root>");
}

TEST(XMLTest, SelfClosingTag) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("<tag/>");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CXMLReader reader(src);
    CXMLWriter writer(sink);

    SXMLEntity entity;
    while (!reader.End()) {
        if (reader.ReadEntity(entity)) {
            writer.WriteEntity(entity);
        }
    }

    EXPECT_EQ(sink->String(), "<tag/>");
}

TEST(XMLTest, AttributesHandling) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("<tag attr=\"value\">data</tag>");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CXMLReader reader(src);
    CXMLWriter writer(sink);

    SXMLEntity entity;
    while (!reader.End()) {
        if (reader.ReadEntity(entity)) {
            writer.WriteEntity(entity);
        }
    }

    EXPECT_EQ(sink->String(), "<tag attr=\"value\">data</tag>");
}

TEST(XMLTest, NestedElements) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("<root><parent><child>value</child></parent></root>");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CXMLReader reader(src);
    CXMLWriter writer(sink);

    SXMLEntity entity;
    while (!reader.End()) {
        if (reader.ReadEntity(entity)) {
            writer.WriteEntity(entity);
        }
    }

    EXPECT_EQ(sink->String(), "<root><parent><child>value</child></parent></root>");
}

TEST(XMLTest, SpecialCharacterEscaping) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("<tag>value &amp; more</tag>");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CXMLReader reader(src);
    CXMLWriter writer(sink);

    SXMLEntity entity;
    while (!reader.End()) {
        if (reader.ReadEntity(entity)) {
            writer.WriteEntity(entity);
        }
    }

    EXPECT_EQ(sink->String(), "<tag>value &amp; more</tag>");
}