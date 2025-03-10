#include <gtest/gtest.h>
#include "StringUtils.h"

TEST(StringUtilsTest, Slice) {
    EXPECT_EQ(StringUtils::Slice("hello world", 0, 5), "hello");
    EXPECT_EQ(StringUtils::Slice("hello world", -5), "world");
    EXPECT_EQ(StringUtils::Slice("hello world", 2, -3), "llo wo");
    EXPECT_EQ(StringUtils::Slice("hello world", 0, 0), "hello world");
}

TEST(StringUtilsTest, Capitalize) {
    EXPECT_EQ(StringUtils::Capitalize("hello"), "Hello");
    EXPECT_EQ(StringUtils::Capitalize("hELLO"), "HELLO");
    EXPECT_EQ(StringUtils::Capitalize(""), "");
}

TEST(StringUtilsTest, Upper) {
    EXPECT_EQ(StringUtils::Upper("hello"), "HELLO");
    EXPECT_EQ(StringUtils::Upper("HeLLo"), "HELLO");
    EXPECT_EQ(StringUtils::Upper(""), "");
}

TEST(StringUtilsTest, Lower) {
    EXPECT_EQ(StringUtils::Lower("HELLO"), "hello");
    EXPECT_EQ(StringUtils::Lower("HeLLo"), "hello");
    EXPECT_EQ(StringUtils::Lower(""), "");
}

TEST(StringUtilsTest, LStrip) {
    EXPECT_EQ(StringUtils::LStrip("   hello"), "hello");
    EXPECT_EQ(StringUtils::LStrip("hello   "), "hello   ");
    EXPECT_EQ(StringUtils::LStrip("   hello   "), "hello   ");
    EXPECT_EQ(StringUtils::LStrip(""), "");
}

TEST(StringUtilsTest, RStrip) {
    EXPECT_EQ(StringUtils::RStrip("hello   "), "hello");
    EXPECT_EQ(StringUtils::RStrip("   hello"), "   hello");
    EXPECT_EQ(StringUtils::RStrip("   hello   "), "   hello");
    EXPECT_EQ(StringUtils::RStrip(""), "");
}

TEST(StringUtilsTest, Strip) {
    EXPECT_EQ(StringUtils::Strip("   hello   "), "hello");
    EXPECT_EQ(StringUtils::Strip("hello"), "hello");
    EXPECT_EQ(StringUtils::Strip("   "), "");
    EXPECT_EQ(StringUtils::Strip(""), "");
}

TEST(StringUtilsTest, Center) {
    EXPECT_EQ(StringUtils::Center("hello", 10, '*'), "**hello***");
    EXPECT_EQ(StringUtils::Center("hello", 5, '-'), "hello");
    EXPECT_EQ(StringUtils::Center("hello", 7, '_'), "_hello_");
}

TEST(StringUtilsTest, LJust) {
    EXPECT_EQ(StringUtils::LJust("hello", 10, '*'), "hello*****");
    EXPECT_EQ(StringUtils::LJust("hello", 5, '-'), "hello");
    EXPECT_EQ(StringUtils::LJust("hello", 7, '_'), "hello__");
}

TEST(StringUtilsTest, RJust) {
    EXPECT_EQ(StringUtils::RJust("hello", 10, '*'), "*****hello");
    EXPECT_EQ(StringUtils::RJust("hello", 5, '-'), "hello");
    EXPECT_EQ(StringUtils::RJust("hello", 7, '_'), "__hello");
}

TEST(StringUtilsTest, Replace) {
    EXPECT_EQ(StringUtils::Replace("hello world", "world", "there"), "hello there");
    EXPECT_EQ(StringUtils::Replace("hello world world", "world", "there"), "hello there there");
    EXPECT_EQ(StringUtils::Replace("hello", "z", "x"), "hello");
}

TEST(StringUtilsTest, Split) {
    std::vector<std::string> expected1 = {"hello", "world"};
    EXPECT_EQ(StringUtils::Split("hello world"), expected1);

    std::vector<std::string> expected2 = {"this", "is", "a", "test"};
    EXPECT_EQ(StringUtils::Split("this is a test"), expected2);

    std::vector<std::string> expected3 = {"hello"};
    EXPECT_EQ(StringUtils::Split("hello"), expected3);
}

TEST(StringUtilsTest, Join) {
    std::vector<std::string> words = {"hello", "world"};
    EXPECT_EQ(StringUtils::Join(" ", words), "hello world");
    EXPECT_EQ(StringUtils::Join("-", words), "hello-world");
    EXPECT_EQ(StringUtils::Join("", words), "helloworld");
}

TEST(StringUtilsTest, ExpandTabs) {
    EXPECT_EQ(StringUtils::ExpandTabs("hello\tworld", 4), "hello    world");
    EXPECT_EQ(StringUtils::ExpandTabs("\t", 4), "    ");
    EXPECT_EQ(StringUtils::ExpandTabs("hello\t", 8), "hello   ");
}

TEST(StringUtilsTest, EditDistance) {
    EXPECT_EQ(StringUtils::EditDistance("kitten", "sitting"), 3);
    EXPECT_EQ(StringUtils::EditDistance("flaw", "lawn"), 2);
    EXPECT_EQ(StringUtils::EditDistance("same", "same"), 0);
    EXPECT_EQ(StringUtils::EditDistance("hello", "HELLO", true), 0);
}