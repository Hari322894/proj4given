#include "DSVReader.h"
#include "DSVWriter.h"
#include "StringDataSource.h"
#include "StringDataSink.h"
#include <gtest/gtest.h>

TEST(DSVTest, BasicReadWrite) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("a,b,c\n1,2,3\n");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CDSVReader reader(src, ',');
    CDSVWriter writer(sink, ',');

    std::vector<std::string> row;
    while (!reader.End()) {
        if (reader.ReadRow(row)) {
            writer.WriteRow(row);
        }
    }

    EXPECT_EQ(sink->String(), "a,b,c\n1,2,3\n");
}

TEST(DSVTest, EmptyInput) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CDSVReader reader(src, ',');
    CDSVWriter writer(sink, ',');

    std::vector<std::string> row;
    while (!reader.End()) {
        if (reader.ReadRow(row)) {
            writer.WriteRow(row);
        }
    }

    EXPECT_EQ(sink->String(), "");  // Expect empty output for empty input.
}

TEST(DSVTest, SingleRowWithoutNewline) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("x,y,z");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CDSVReader reader(src, ',');
    CDSVWriter writer(sink, ',');

    std::vector<std::string> row;
    while (!reader.End()) {
        if (reader.ReadRow(row)) {
            writer.WriteRow(row);
        }
    }

    EXPECT_EQ(sink->String(), "x,y,z\n");  // Expect newline to be added after the row.
}

TEST(DSVTest, MultipleRowsWithDifferentLengths) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("a,b\n1,2,3\nx,y,z,w");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CDSVReader reader(src, ',');
    CDSVWriter writer(sink, ',');

    std::vector<std::string> row;
    while (!reader.End()) {
        if (reader.ReadRow(row)) {
            writer.WriteRow(row);
        }
    }

    EXPECT_EQ(sink->String(), "a,b\n1,2,3\nx,y,z,w\n");  // Expect all rows to be written with newlines.
}

TEST(DSVTest, QuotedValuesWithCommas) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("\"value, with, commas\",b,c\n");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CDSVReader reader(src, ',');
    CDSVWriter writer(sink, ',');

    std::vector<std::string> row;
    while (!reader.End()) {
        if (reader.ReadRow(row)) {
            writer.WriteRow(row);
        }
    }

    EXPECT_EQ(sink->String(), "\"value, with, commas\",b,c\n");  // Ensure commas inside quotes are preserved.
}

TEST(DSVTest, DifferentDelimiter) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("a|b|c\n1|2|3\n");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CDSVReader reader(src, '|');
    CDSVWriter writer(sink, '|');

    std::vector<std::string> row;
    while (!reader.End()) {
        if (reader.ReadRow(row)) {
            writer.WriteRow(row);
        }
    }

    EXPECT_EQ(sink->String(), "a|b|c\n1|2|3\n");  // Verify that the pipe delimiter is used correctly.
}

TEST(DSVTest, ExtraWhitespaceHandling) {
    std::shared_ptr<CStringDataSource> src = std::make_shared<CStringDataSource>("  a , b ,c\n1,2 , 3\n");
    std::shared_ptr<CStringDataSink> sink = std::make_shared<CStringDataSink>();

    CDSVReader reader(src, ',');
    CDSVWriter writer(sink, ',');

    std::vector<std::string> row;
    while (!reader.End()) {
        if (reader.ReadRow(row)) {
            writer.WriteRow(row);
        }
    }

    EXPECT_EQ(sink->String(), "  a , b ,c\n1,2 , 3\n");  // Expect the original spacing to be preserved.
}