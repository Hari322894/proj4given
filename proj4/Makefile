# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Iinclude
LDFLAGS = -lgtest -lgtest_main -pthread -lexpat

# Directories
SRC_DIR = src
TEST_DIR = testsrc
SRC_OBJ_DIR = obj/src
TEST_OBJ_DIR = obj/test
BIN_DIR = bin

# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
TEST_FILES = $(wildcard $(TEST_DIR)/*.cpp)

# Object files
SRC_OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(SRC_OBJ_DIR)/%.o,$(SRC_FILES))
TEST_OBJ_FILES = $(patsubst $(TEST_DIR)/%.cpp,$(TEST_OBJ_DIR)/%.o,$(TEST_FILES))

# Output binary
GTEST_TARGET = $(BIN_DIR)/runtests

# Default target
all: $(GTEST_TARGET)

# Rule to build the Google Test binary
$(GTEST_TARGET): $(SRC_OBJ_FILES) $(TEST_OBJ_FILES)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Rule to compile source files
$(SRC_OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(SRC_OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to compile test files
$(TEST_OBJ_DIR)/%.o: $(TEST_DIR)/%.cpp | $(TEST_OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ensure the object directories exist
$(SRC_OBJ_DIR):
	@mkdir -p $(SRC_OBJ_DIR)

$(TEST_OBJ_DIR):
	@mkdir -p $(TEST_OBJ_DIR)

# Clean build artifacts
clean:
	rm -rf $(SRC_OBJ_DIR) $(TEST_OBJ_DIR) $(BIN_DIR)

# Run tests
test: all
	./$(GTEST_TARGET)

# Phony targets
.PHONY: all clean test