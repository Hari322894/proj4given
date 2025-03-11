# Directories
SRC_DIR = src
TEST_SRC_DIR = testsrc
INCLUDE_DIR = include
OBJ_DIR = obj
BIN_DIR = bin

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -g -I$(INCLUDE_DIR)
LDFLAGS = -lgtest -lgtest_main -pthread -lexpat

# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
TEST_FILES = $(wildcard $(TEST_SRC_DIR)/*.cpp)

# Object files (excluding main files)
MAIN_SRC_FILES = # Add any main program files to exclude here if needed
SRC_OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(filter-out $(MAIN_SRC_FILES),$(SRC_FILES)))
TEST_OBJ_FILES = $(patsubst $(TEST_SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(TEST_FILES))

# Test executable targets
TEST_TARGETS = $(BIN_DIR)/testcsvbs \
               $(BIN_DIR)/testosm \
               $(BIN_DIR)/testdpr \
               $(BIN_DIR)/testcsvbsi \
               $(BIN_DIR)/testtp \
               $(BIN_DIR)/testcl

# Default target
all: directories $(TEST_TARGETS) runtests

# Create directories if they don't exist
directories:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(OBJ_DIR)

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile test files
$(OBJ_DIR)/%.o: $(TEST_SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Test-specific dependencies
$(BIN_DIR)/testcsvbs: $(OBJ_DIR)/CSVBusSystem.o $(OBJ_DIR)/CSVBusSystemTest.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testosm: $(OBJ_DIR)/OpenStreetMap.o $(OBJ_DIR)/OSMTest.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/StringDataSource.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testdpr: $(OBJ_DIR)/DijkstraPathRouter.o $(OBJ_DIR)/DijkstraPathRouterTest.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testcsvbsi: $(OBJ_DIR)/BusSystemIndexer.o $(OBJ_DIR)/CSVBusSystemIndexerTest.o $(OBJ_DIR)/CSVBusSystem.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testtp: $(OBJ_DIR)/DijkstraTransportationPlanner.o $(OBJ_DIR)/CSVOSMTransportationPlannerTest.o $(OBJ_DIR)/DijkstraPathRouter.o $(OBJ_DIR)/BusSystemIndexer.o $(OBJ_DIR)/CSVBusSystem.o $(OBJ_DIR)/OpenStreetMap.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o $(OBJ_DIR)/GeographicUtils.o
	$(CXX) $^ -o $@ $(LDFLAGS)


# Add rules for main executables if needed
# Example:
# $(BIN_DIR)/transplanner: $(OBJ_DIR)/transplanner.o $(SRC_OBJ_FILES)
#	$(CXX) $^ -o $@ $(LDFLAGS)

# Run tests in the specified order
runtests: $(TEST_TARGETS)
	@for exe in $(TEST_TARGETS); do \
		echo "Running $$exe..."; \
		./$$exe; \
		if [ $$? -ne 0 ]; then \
			echo "Test $$exe failed!"; \
			exit 1; \
		fi; \
	done

# Clean up the build directories
clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR)

# Phony targets
.PHONY: all directories runtests clean