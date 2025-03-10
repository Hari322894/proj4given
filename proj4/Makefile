# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude
LDFLAGS = -lgtest -lgtest_main -pthread -lexpat

# Directories
SRC_DIR = src
TEST_DIR = testsrc
OBJ_DIR = obj
BIN_DIR = bin

# Common source files (all source files except those with main functions)
MAIN_FILES = $(SRC_DIR)/kmlout.cpp $(SRC_DIR)/speedtest.cpp $(SRC_DIR)/transplanner.cpp
COMMON_SRC_FILES = $(filter-out $(MAIN_FILES), $(wildcard $(SRC_DIR)/*.cpp))

# Test files
TEST_FILES = $(wildcard $(TEST_DIR)/*.cpp)

# Object files
COMMON_OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(COMMON_SRC_FILES))
TEST_OBJ_FILES = $(patsubst $(TEST_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(TEST_FILES))

# Executables
TRANSPLANNER = $(BIN_DIR)/transplanner
KMLOUT = $(BIN_DIR)/kmlout
SPEEDTEST = $(BIN_DIR)/speedtest
GTEST_TARGET = $(BIN_DIR)/runtests

# Default target
all: $(TRANSPLANNER) $(KMLOUT) $(SPEEDTEST) $(GTEST_TARGET)

# Rule to build individual executables
$(TRANSPLANNER): $(COMMON_OBJ_FILES) $(OBJ_DIR)/transplanner.o | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(KMLOUT): $(COMMON_OBJ_FILES) $(OBJ_DIR)/kmlout.o | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

$(SPEEDTEST): $(COMMON_OBJ_FILES) $(OBJ_DIR)/speedtest.o | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Rule to build the Google Test binary
$(GTEST_TARGET): $(COMMON_OBJ_FILES) $(TEST_OBJ_FILES) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Rule to compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to compile test files
$(OBJ_DIR)/%.o: $(TEST_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create directories if they don't exist
$(OBJ_DIR) $(BIN_DIR):
	@mkdir -p $@

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Run tests
test: $(GTEST_TARGET)
	./$(GTEST_TARGET)

# Run the main application
run: $(TRANSPLANNER)
	./$(TRANSPLANNER)

# Phony targets
.PHONY: all clean test run