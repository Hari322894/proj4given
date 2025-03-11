# Project configuration
PROJECT_NAME = transportation-planner
VERSION = 1.0.0

# Compiler and flags
CXX = g++
CXXFLAGS_COMMON = -std=c++17 -Wall -Iinclude
CXXFLAGS_DEBUG = -g -O0 -DDEBUG
CXXFLAGS_RELEASE = -O3 -DNDEBUG
CXXFLAGS = $(CXXFLAGS_COMMON) $(CXXFLAGS_DEBUG)
LDFLAGS = -lgtest -lgtest_main -pthread -lexpat

# Directories
SRC_DIR = src
TEST_DIR = testsrc
OBJ_DIR = obj
BIN_DIR = bin
DEP_DIR = $(OBJ_DIR)/deps
SRC_OBJ_DIR = $(OBJ_DIR)/src
TEST_OBJ_DIR = $(OBJ_DIR)/test

# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
TEST_FILES = $(wildcard $(TEST_DIR)/*.cpp)

# Object files
SRC_OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(SRC_OBJ_DIR)/%.o,$(SRC_FILES))
TEST_OBJ_FILES = $(patsubst $(TEST_DIR)/%.cpp,$(TEST_OBJ_DIR)/%.o,$(TEST_FILES))

# Dependency files
SRC_DEP_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(DEP_DIR)/%.d,$(SRC_FILES))
TEST_DEP_FILES = $(patsubst $(TEST_DIR)/%.cpp,$(DEP_DIR)/%.d,$(TEST_FILES))

# Output binaries
GTEST_TARGET = $(BIN_DIR)/runtests
MAIN_TARGET = $(BIN_DIR)/$(PROJECT_NAME)

# Default target
all: $(GTEST_TARGET)

# Include dependency files
-include $(SRC_DEP_FILES) $(TEST_DEP_FILES)

# Rule to build the Google Test binary
$(GTEST_TARGET): $(SRC_OBJ_FILES) $(TEST_OBJ_FILES) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Rule to build the main binary (if you decide to add one)
$(MAIN_TARGET): $(SRC_OBJ_FILES) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Rule to compile source files
$(SRC_OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(SRC_OBJ_DIR) $(DEP_DIR)
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(DEP_DIR)/$*.d -c $< -o $@

# Rule to compile test files
$(TEST_OBJ_DIR)/%.o: $(TEST_DIR)/%.cpp | $(TEST_OBJ_DIR) $(DEP_DIR)
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(DEP_DIR)/$*.d -c $< -o $@

# Ensure directories exist
$(BIN_DIR) $(SRC_OBJ_DIR) $(TEST_OBJ_DIR) $(DEP_DIR):
	@mkdir -p $@

# Build targets
debug: CXXFLAGS = $(CXXFLAGS_COMMON) $(CXXFLAGS_DEBUG)
debug: all

release: CXXFLAGS = $(CXXFLAGS_COMMON) $(CXXFLAGS_RELEASE)
release: all

# Clean build artifacts
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Run tests
test: all
	./$(GTEST_TARGET)

# Show help
help:
	@echo "$(PROJECT_NAME) v$(VERSION) build system"
	@echo "Available targets:"
	@echo "  all        - Build all targets (default)"
	@echo "  debug      - Build with debug flags"
	@echo "  release    - Build with release flags"
	@echo "  test       - Run unit tests"
	@echo "  clean      - Remove build artifacts"
	@echo "  help       - Show this help message"

# Phony targets
.PHONY: all debug release clean test help