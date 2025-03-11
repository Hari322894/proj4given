# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude
LDFLAGS = -lgtest -lgtest_main -pthread -lexpat

# Directories
SRC_DIR = src
TEST_DIR = testsrc
OBJ_DIR = obj
BIN_DIR = bin

# Source files (excluding main program files)
MAIN_SRC_FILES = $(SRC_DIR)/kmlout.cpp $(SRC_DIR)/speedtest.cpp $(SRC_DIR)/transplanner.cpp
SRC_FILES = $(filter-out $(MAIN_SRC_FILES), $(wildcard $(SRC_DIR)/*.cpp))
TEST_FILES = $(wildcard $(TEST_DIR)/*.cpp)

# Object files
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))
TEST_OBJ_FILES = $(patsubst $(TEST_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(TEST_FILES))
MAIN_OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(MAIN_SRC_FILES))

# Output binaries
GTEST_TARGET = $(BIN_DIR)/runtests
KMLOUT_TARGET = $(BIN_DIR)/kmlout
SPEEDTEST_TARGET = $(BIN_DIR)/speedtest
TRANSPLANNER_TARGET = $(BIN_DIR)/transplanner

# Default target
all: $(GTEST_TARGET) $(KMLOUT_TARGET) $(SPEEDTEST_TARGET) $(TRANSPLANNER_TARGET)

# Rule to build the Google Test binary (without the main programs)
$(GTEST_TARGET): $(OBJ_FILES) $(TEST_OBJ_FILES)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Rules to build individual executables
$(KMLOUT_TARGET): $(OBJ_DIR)/kmlout.o $(OBJ_FILES)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ -lexpat

$(SPEEDTEST_TARGET): $(OBJ_DIR)/speedtest.o $(OBJ_FILES)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ -lexpat

$(TRANSPLANNER_TARGET): $(OBJ_DIR)/transplanner.o $(OBJ_FILES)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ -lexpat

# Rule to compile source and test files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/%.o: $(TEST_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ensure the object directory exists
$(OBJ_DIR):
	@mkdir -p $(OBJ_DIR)

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Run tests
test: $(GTEST_TARGET)
	./$(GTEST_TARGET)

# Phony targets
.PHONY: all clean test