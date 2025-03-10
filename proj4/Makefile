# Directories
BIN_DIR = bin
OBJ_DIR = obj
SRC_DIR = src
TEST_SRC_DIR = testsrc
INCLUDE_DIR = include

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -g -I$(INCLUDE_DIR) -I/opt/homebrew/opt/googletest/include
LDFLAGS = -L/opt/homebrew/opt/googletest/lib -lgtest -lgtest_main -pthread -lexpat

# Executables
EXECUTABLES = $(BIN_DIR)/testcsvbs \
              $(BIN_DIR)/testosm \
              $(BIN_DIR)/testdpr \
              $(BIN_DIR)/testcsvbsi \
              $(BIN_DIR)/testtp \
              $(BIN_DIR)/testcl

# Default target
all: directories $(EXECUTABLES) runtests

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


# Link object files into executables for each test
$(BIN_DIR)/testcsvbs: $(OBJ_DIR)/CSVBusSystem.o $(OBJ_DIR)/CSVBusSystemTest.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testosm: $(OBJ_DIR)/OpenStreetMap.o $(OBJ_DIR)/OSMTest.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/StringDataSource.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testdpr: $(OBJ_DIR)/DijkstraPathRouter.o $(OBJ_DIR)/DijkstraPathRouterTest.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testcsvbsi: $(OBJ_DIR)/BusSystemIndexer.o $(OBJ_DIR)/CSVBusSystemIndexerTest.o $(OBJ_DIR)/CSVBusSystem.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testtp: $(OBJ_DIR)/DijkstraTransportationPlanner.o $(OBJ_DIR)/CSVOSMTransportationPlannerTest.o $(OBJ_DIR)/DijkstraPathRouter.o $(OBJ_DIR)/BusSystemIndexer.o $(OBJ_DIR)/CSVBusSystem.o $(OBJ_DIR)/OpenStreetMap.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o
	$(CXX) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/testcl: $(OBJ_DIR)/TransportationPlannerCommandLine.o $(OBJ_DIR)/TPCommandLineTest.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringDataSink.o
	$(CXX) $^ -o $@ $(LDFLAGS)

# Run tests in the specified order
runtests: $(EXECUTABLES)
	@for exe in $(EXECUTABLES); do \
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