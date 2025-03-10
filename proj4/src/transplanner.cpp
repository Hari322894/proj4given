#include "TransportationPlannerCommandLine.h"
#include "FileDataSource.h"
#include "FileDataSink.h"
#include "FileDataFactory.h"
#include "DijkstraTransportationPlanner.h"
#include "TransportationPlannerConfig.h"
#include "CSVBusSystem.h"
#include "OpenStreetMap.h"
#include "StandardDataSource.h"
#include "StandardDataSink.h"
#include "StandardErrorDataSink.h"
#include "DSVReader.h"
#include <memory>
#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
    if(argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <map_file> <stop_file> <route_file> [command_file] [output_file] [error_file]" << std::endl;
        return 1;
    }
    // Parse command line arguments
    std::string mapFilename = argv[1];
    std::string stopFilename = argv[2];
    std::string routeFilename = argv[3];
   
    // Optional command, output, and error files
    std::string commandFilename = (argc > 4) ? argv[4] : "stdin";
    std::string outputFilename = (argc > 5) ? argv[5] : "stdout";
    std::string errorFilename = (argc > 6) ? argv[6] : "stderr";
   
    try {
        // Create data sources and sinks
        std::shared_ptr<CDataSource> commandSource;
        std::shared_ptr<CDataSink> outputSink;
        std::shared_ptr<CDataSink> errorSink;
       
        // Command source setup
        if(commandFilename == "stdin") {
            commandSource = std::make_shared<CStandardDataSource>();
        } else {
            commandSource = std::make_shared<CFileDataSource>(commandFilename);
        }
       
        // Output sink setup
        if(outputFilename == "stdout") {
            outputSink = std::make_shared<CStandardDataSink>();
        } else {
            outputSink = std::make_shared<CFileDataSink>(outputFilename);
        }
       
        // Error sink setup
        if(errorFilename == "stderr") {
            errorSink = std::make_shared<CStandardErrorDataSink>();
        } else {
            errorSink = std::make_shared<CFileDataSink>(errorFilename);
        }
       
        // Create results factory
        auto resultsFactory = std::make_shared<CFileDataFactory>("");
       
        // Load street map
        auto streetMap = std::make_shared<COpenStreetMap>(mapFilename);

        // Create DSV readers for stop and route files
        auto stopReader = std::make_shared<CDSVReader>(std::make_shared<CFileDataSource>(stopFilename));
        auto routeReader = std::make_shared<CDSVReader>(std::make_shared<CFileDataSource>(routeFilename));
        
        // Create bus system using DSV readers
        auto busSystem = std::make_shared<CCSVBusSystem>(stopReader, routeReader);
       
        // Create transportation planner configuration
        auto config = std::make_shared<STransportationPlannerConfig>(streetMap, busSystem);
       
        // Create transportation planner
        auto planner = std::make_shared<CDijkstraTransportationPlanner>(config);
       
        // Create and run command line processor
        CTransportationPlannerCommandLine commandLine(commandSource, outputSink, errorSink, resultsFactory, planner);
        if(!commandLine.ProcessCommands()) {
            return 1;
        }
       
        return 0;
    }
    catch(const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
}