CC=gcc
CXX=g++

INC_DIR=./include
SRC_DIR=./src
TESTSRC_DIR=./testsrc
BIN_DIR=./bin
OBJ_DIR=./obj

CXXFLAG=-std=c++17 -I $(INC_DIR) -I /opt/local/include
LDFLAGS=-L /opt/local/lib -lgtest -lgtest_main -lpthread -lexpat

all:directories teststrutils teststrdatasource teststrdatasink testdsv testxml testcsvbs testosm testcsvbsi testcsvbs testfiledatass testkml testdpr testtpcl testtp

teststrutils: $(BIN_DIR)/teststrutils
	$(BIN_DIR)/teststrutils
	
teststrdatasource: $(BIN_DIR)/teststrdatasource
	$(BIN_DIR)/teststrdatasource 

teststrdatasink: $(BIN_DIR)/teststrdatasink
	$(BIN_DIR)/teststrdatasink
	
testdsv: $(BIN_DIR)/testdsv
	$(BIN_DIR)/testdsv

testxml: $(BIN_DIR)/testxml
	$(BIN_DIR)/testxml

testosm: $(BIN_DIR)/testosm
	$(BIN_DIR)/testosm

testcsvbsi: $(BIN_DIR)/testcsvbsi
	$(BIN_DIR)/testcsvbsi

testcsvbs: $(BIN_DIR)/testcsvbs
	$(BIN_DIR)/testcsvbs

testfiledatass: $(BIN_DIR)/testfiledatass
	$(BIN_DIR)/testfiledatass

testkml: $(BIN_DIR)/testkml
	$(BIN_DIR)/testkml

testdpr: $(BIN_DIR)/testdpr
	$(BIN_DIR)/testdpr

testtpcl: $(BIN_DIR)/testtpcl
	$(BIN_DIR)/testtpcl

testtp: $(BIN_DIR)/testtp
	$(BIN_DIR)/testtp


$(BIN_DIR)/teststrutils: $(OBJ_DIR)/StringDataSourceTest.o $(OBJ_DIR)/StringDataSource.o
	$(CXX) -o $(BIN_DIR)/teststrutils $(OBJ_DIR)/StringDataSourceTest.o $(OBJ_DIR)/StringDataSource.o $(LDFLAGS)

$(BIN_DIR)/teststrdatasource: $(OBJ_DIR)/StringDataSourceTest.o $(OBJ_DIR)/StringDataSource.o
	$(CXX) -o $(BIN_DIR)/teststrdatasource $(OBJ_DIR)/StringDataSourceTest.o $(OBJ_DIR)/StringDataSource.o $(LDFLAGS)

$(BIN_DIR)/teststrdatasink: $(OBJ_DIR)/StringDataSinkTest.o $(OBJ_DIR)/StringDataSink.o
	$(CXX) -o $(BIN_DIR)/teststrdatasink $(OBJ_DIR)/StringDataSinkTest.o $(OBJ_DIR)/StringDataSink.o $(LDFLAGS)

$(BIN_DIR)/testdsv: $(OBJ_DIR)/DSVTest.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/DSVWriter.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringUtils.o
	$(CXX) -o $(BIN_DIR)/testdsv $(OBJ_DIR)/DSVTest.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/DSVWriter.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringUtils.o $(LDFLAGS)

$(BIN_DIR)/testxml: $(OBJ_DIR)/XMLTest.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/XMLWriter.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringUtils.o
	$(CXX) -o $(BIN_DIR)/testxml $(OBJ_DIR)/XMLTest.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/XMLWriter.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringUtils.o $(LDFLAGS)

$(BIN_DIR)/testcsvbs: $(OBJ_DIR)/CSVBSTest.o $(OBJ_DIR)/CSVBS.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o
	$(CXX) -o $(BIN_DIR)/testcsvbs $(OBJ_DIR)/CSVBSTest.o $(OBJ_DIR)/CSVBS.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o $(LDFLAGS)

$(BIN_DIR)/testosm: $(OBJ_DIR)/OSMTest.o $(OBJ_DIR)/OpenStreetMap.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o
	$(CXX) -o $(BIN_DIR)/testosm $(OBJ_DIR)/OSMTest.o $(OBJ_DIR)/OpenStreetMap.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/StringUtils.o $(LDFLAGS)

$(BIN_DIR)/testkml: $(OBJ_DIR)/KMLTest.o $(OBJ_DIR)/KMLWriter.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/XMLWriter.o $(OBJ_DIR)/StringUtils.o
	$(CXX) -o $(BIN_DIR)/testkml $(OBJ_DIR)/KMLTest.o $(OBJ_DIR)/KMLWriter.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/XMLWriter.o $(OBJ_DIR)/StringUtils.o $(LDFLAGS)

$(BIN_DIR)/testfiledatass: $(OBJ_DIR)/FileDataSSTest.o $(OBJ_DIR)/FileDataSource.o $(OBJ_DIR)/FileDataSink.o $(OBJ_DIR)/FileDataFactory.o
	$(CXX) -o $(BIN_DIR)/testfiledatass $(OBJ_DIR)/FileDataSSTest.o $(OBJ_DIR)/FileDataSource.o $(OBJ_DIR)/FileDataSink.o $(OBJ_DIR)/FileDataFactory.o $(LDFLAGS)

$(BIN_DIR)/testdpr: $(OBJ_DIR)/DijkstraPathRouterTest.o $(OBJ_DIR)/DijkstraPathRouter.o
	$(CXX) -o $(BIN_DIR)/testdpr $(OBJ_DIR)/DijkstraPathRouterTest.o $(OBJ_DIR)/DijkstraPathRouter.o $(LDFLAGS)

$(BIN_DIR)/testcsvbsi: $(OBJ_DIR)/CSVBusSystemIndexerTest.o $(OBJ_DIR)/BusSystemIndexer.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/CSVBS.o
	$(CXX) -o $(BIN_DIR)/testcsvbsi $(OBJ_DIR)/CSVBusSystemIndexerTest.o $(OBJ_DIR)/BusSystemIndexer.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/CSVBS.o $(LDFLAGS)

$(BIN_DIR)/testtpcl: $(OBJ_DIR)/TPCommandLineTest.o $(OBJ_DIR)/TransportationPlannerCommandLine.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringDataSource.o
	$(CXX) -o $(BIN_DIR)/testtpcl $(OBJ_DIR)/TPCommandLineTest.o $(OBJ_DIR)/TransportationPlannerCommandLine.o $(OBJ_DIR)/StringDataSink.o $(OBJ_DIR)/StringDataSource.o $(LDFLAGS) -lgmock

$(BIN_DIR)/testtp: $(OBJ_DIR)/CSVOSMTransportationPlannerTest.o $(OBJ_DIR)/DijkstraTransportationPlanner.o $(OBJ_DIR)/GeographicUtils.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/CSVBS.o $(OBJ_DIR)/StringUtils.o $(OBJ_DIR)/OpenStreetMap.o
	$(CXX) -o $(BIN_DIR)/testtp $(OBJ_DIR)/CSVOSMTransportationPlannerTest.o $(OBJ_DIR)/DijkstraTransportationPlanner.o $(OBJ_DIR)/GeographicUtils.o $(OBJ_DIR)/StringDataSource.o $(OBJ_DIR)/DSVReader.o $(OBJ_DIR)/XMLReader.o $(OBJ_DIR)/CSVBS.o $(OBJ_DIR)/StringUtils.o $(OBJ_DIR)/OpenStreetMap.o $(LDFLAGS)

$(OBJ_DIR)/StringUtilsTest.o: $(INC_DIR)/StringUtils.h
	$(CXX) -o $(OBJ_DIR)/StringUtilsTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/StringUtilsTest.cpp

$(OBJ_DIR)/StringDataSinkTest.o: $(INC_DIR)/StringDataSink.h
	$(CXX) -o $(OBJ_DIR)/StringDataSinkTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/StringDataSinkTest.cpp

$(OBJ_DIR)/StringDataSourceTest.o:$(INC_DIR)/StringDataSource.h
	$(CXX) -o $(OBJ_DIR)/StringDataSourceTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/StringDataSourceTest.cpp

$(OBJ_DIR)/StringUtils.o: $(SRC_DIR)/StringUtils.cpp $(INC_DIR)/StringUtils.h
	$(CXX) -o $(OBJ_DIR)/StringUtils.o $(CXXFLAG) -c $(SRC_DIR)/StringUtils.cpp

$(OBJ_DIR)/StringDataSink.o: $(SRC_DIR)/StringDataSink.cpp $(INC_DIR)/StringDataSink.h
	$(CXX) -o $(OBJ_DIR)/StringDataSink.o $(CXXFLAG) -c $(SRC_DIR)/StringDataSink.cpp

$(OBJ_DIR)/StringDataSource.o: $(SRC_DIR)/StringDataSource.cpp $(INC_DIR)/StringDataSource.h
	$(CXX) -o $(OBJ_DIR)/StringDataSource.o $(CXXFLAG) -c $(SRC_DIR)/StringDataSource.cpp

$(OBJ_DIR)/DSVTest.o: $(TESTSRC_DIR)/DSVTest.cpp $(INC_DIR)/DSVReader.h $(INC_DIR)/DSVWriter.h
	$(CXX) -o $(OBJ_DIR)/DSVTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/DSVTest.cpp

$(OBJ_DIR)/XMLTest.o: $(TESTSRC_DIR)/XMLTest.cpp $(INC_DIR)/XMLReader.h $(INC_DIR)/XMLWriter.h
	$(CXX) -o $(OBJ_DIR)/XMLTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/XMLTest.cpp

$(OBJ_DIR)/DSVReader.o: $(SRC_DIR)/DSVReader.cpp $(INC_DIR)/DSVReader.h
	$(CXX) -o $(OBJ_DIR)/DSVReader.o $(CXXFLAG) -c $(SRC_DIR)/DSVReader.cpp

$(OBJ_DIR)/DSVWriter.o: $(SRC_DIR)/DSVWriter.cpp $(INC_DIR)/DSVWriter.h
	$(CXX) -o $(OBJ_DIR)/DSVWriter.o $(CXXFLAG) -c $(SRC_DIR)/DSVWriter.cpp

$(OBJ_DIR)/XMLReader.o: $(SRC_DIR)/XMLReader.cpp $(INC_DIR)/XMLReader.h $(INC_DIR)/XMLEntity.h
	$(CXX) -o $(OBJ_DIR)/XMLReader.o $(CXXFLAG) -c $(SRC_DIR)/XMLReader.cpp

$(OBJ_DIR)/XMLWriter.o: $(SRC_DIR)/XMLWriter.cpp $(INC_DIR)/XMLWriter.h $(INC_DIR)/XMLEntity.h
	$(CXX) -o $(OBJ_DIR)/XMLWriter.o $(CXXFLAG) -c $(SRC_DIR)/XMLWriter.cpp

$(OBJ_DIR)/CSVBS.o: $(SRC_DIR)/CSVBusSystem.cpp $(INC_DIR)/CSVBusSystem.h $(INC_DIR)/BusSystem.h
	$(CXX) -o $(OBJ_DIR)/CSVBS.o $(CXXFLAG) -c $(SRC_DIR)/CSVBusSystem.cpp

$(OBJ_DIR)/OpenStreetMap.o: $(SRC_DIR)/OpenStreetMap.cpp $(INC_DIR)/OpenStreetMap.h $(INC_DIR)/StreetMap.h
	$(CXX) -o $(OBJ_DIR)/OpenStreetMap.o $(CXXFLAG) -c $(SRC_DIR)/OpenStreetMap.cpp
	
$(OBJ_DIR)/CSVBSTest.o: $(TESTSRC_DIR)/CSVBSTest.cpp $(INC_DIR)/CSVBusSystem.h $(INC_DIR)/BusSystem.h
	$(CXX) -o $(OBJ_DIR)/CSVBSTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/CSVBSTest.cpp

$(OBJ_DIR)/OSMTest.o: $(TESTSRC_DIR)/OSMTest.cpp $(INC_DIR)/OpenStreetMap.h $(INC_DIR)/StreetMap.h
	$(CXX) -o $(OBJ_DIR)/OSMTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/OSMTest.cpp

$(OBJ_DIR)/KMLTest.o: $(TESTSRC_DIR)/KMLTest.cpp $(INC_DIR)/KMLWriter.h
	$(CXX) -o $(OBJ_DIR)/KMLTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/KMLTest.cpp

$(OBJ_DIR)/KMLWriter.o: $(SRC_DIR)/KMLWriter.cpp $(INC_DIR)/KMLWriter.h
	$(CXX) -o $(OBJ_DIR)/KMLWriter.o $(CXXFLAG) -c $(SRC_DIR)/KMLWriter.cpp

$(OBJ_DIR)/FileDataSSTest.o: $(TESTSRC_DIR)/FileDataSSTest.cpp $(INC_DIR)/FileDataSink.h $(INC_DIR)/FileDataSource.h $(INC_DIR)/FileDataFactory.h
	$(CXX) -o $(OBJ_DIR)/FileDataSSTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/FileDataSSTest.cpp

$(OBJ_DIR)/FileDataSource.o: $(SRC_DIR)/FileDataSource.cpp $(INC_DIR)/FileDataSource.h
	$(CXX) -o $(OBJ_DIR)/FileDataSource.o $(CXXFLAG) -c $(SRC_DIR)/FileDataSource.cpp

$(OBJ_DIR)/FileDataSink.o: $(SRC_DIR)/FileDataSink.cpp $(INC_DIR)/FileDataSink.h
	$(CXX) -o $(OBJ_DIR)/FileDataSink.o $(CXXFLAG) -c $(SRC_DIR)/FileDataSink.cpp

$(OBJ_DIR)/FileDataFactory.o: $(SRC_DIR)/FileDataFactory.cpp $(INC_DIR)/FileDataFactory.h
	$(CXX) -o $(OBJ_DIR)/FileDataFactory.o $(CXXFLAG) -c $(SRC_DIR)/FileDataFactory.cpp

$(OBJ_DIR)/DijkstraPathRouterTest.o: $(TESTSRC_DIR)/DijkstraPathRouterTest.cpp $(INC_DIR)/DijkstraPathRouter.h
	$(CXX) -o $(OBJ_DIR)/DijkstraPathRouterTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/DijkstraPathRouterTest.cpp

$(OBJ_DIR)/DijkstraPathRouter.o: $(SRC_DIR)/DijkstraPathRouter.cpp $(INC_DIR)/DijkstraPathRouter.h
	$(CXX) -o $(OBJ_DIR)/DijkstraPathRouter.o $(CXXFLAG) -c $(SRC_DIR)/DijkstraPathRouter.cpp

$(OBJ_DIR)/CSVBusSystemIndexerTest.o: $(TESTSRC_DIR)/CSVBusSystemIndexerTest.cpp $(INC_DIR)/BusSystemIndexer.h
	$(CXX) -o $(OBJ_DIR)/CSVBusSystemIndexerTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/CSVBusSystemIndexerTest.cpp

$(OBJ_DIR)/BusSystemIndexer.o: $(SRC_DIR)/BusSystemIndexer.cpp $(INC_DIR)/BusSystemIndexer.h
	$(CXX) -o $(OBJ_DIR)/BusSystemIndexer.o $(CXXFLAG) -c $(SRC_DIR)/BusSystemIndexer.cpp

$(OBJ_DIR)/TPCommandLineTest.o: $(TESTSRC_DIR)/TPCommandLineTest.cpp $(INC_DIR)/TransportationPlannerCommandLine.h
	$(CXX) -o $(OBJ_DIR)/TPCommandLineTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/TPCommandLineTest.cpp

$(OBJ_DIR)/TransportationPlannerCommandLine.o: $(SRC_DIR)/TransportationPlannerCommandLine.cpp $(INC_DIR)/TransportationPlannerCommandLine.h
	$(CXX) -o $(OBJ_DIR)/TransportationPlannerCommandLine.o $(CXXFLAG) -c $(SRC_DIR)/TransportationPlannerCommandLine.cpp

$(OBJ_DIR)/CSVOSMTransportationPlannerTest.o: $(TESTSRC_DIR)/CSVOSMTransportationPlannerTest.cpp $(INC_DIR)/DijkstraTransportationPlanner.h
	$(CXX) -o $(OBJ_DIR)/CSVOSMTransportationPlannerTest.o $(CXXFLAG) -c $(TESTSRC_DIR)/CSVOSMTransportationPlannerTest.cpp

$(OBJ_DIR)/DijkstraTransportationPlanner.o: $(SRC_DIR)/DijkstraTransportationPlanner.cpp $(INC_DIR)/DijkstraTransportationPlanner.h
	$(CXX) -o $(OBJ_DIR)/DijkstraTransportationPlanner.o $(CXXFLAG) -c $(SRC_DIR)/DijkstraTransportationPlanner.cpp

$(OBJ_DIR)/GeographicUtils.o: $(SRC_DIR)/GeographicUtils.cpp $(INC_DIR)/GeographicUtils.h
	$(CXX) -o $(OBJ_DIR)/GeographicUtils.o $(CXXFLAG) -c $(SRC_DIR)/GeographicUtils.cpp



directories:
	mkdir -p $(BIN_DIR)
	mkdir -p $(OBJ_DIR)
clean:
	rm -rf $(BIN_DIR)
	rm -rf $(OBJ_DIR)
