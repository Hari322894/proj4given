#include "DSVReader.h"
#include "DSVWriter.h"
#include "XMLReader.h"
#include "CSVBusSystem.h"
#include "OpenStreetMap.h"
#include "StandardDataSource.h"
#include "StandardDataSink.h"
#include "StandardErrorDataSink.h"
#include "FileDataSource.h"
#include "FileDataFactory.h"
#include "StringUtils.h"
#include "DijkstraTransportationPlanner.h"
#include "TransportationPlannerCommandLine.h"
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

void PrintUsage(std::ostream &out) {
    out << "Usage: transplanner --map <osm file> --bus <bus system CSV directory>" << std::endl;
}

int main(int argc, char *argv[]) {
    std::unordered_map<std::string, std::string> options;
    
    // Parse command line arguments
    for(int i = 1; i < argc; i += 2) {
        if(i + 1 >= argc) {
            std::cerr << "Error: Missing value for option " << argv[i] << std::endl;
            PrintUsage(std::cerr);
            return 1;
        }
        
        std::string option = argv[i];
        std::string value = argv[i + 1];
        
        if(option == "--help" || option == "-h") {
            PrintUsage(std::cout);
            return 0;
        }
        
        options[option] = value;
    }
    
    // Check for required arguments
    if(options.find("--map") == options.end() || options.find("--bus") == options.end()) {
        std::cerr << "Error: Missing required arguments" << std::endl;
        PrintUsage(std::cerr);
        return 1;
    }
    
    // Create data sources and sinks
    auto InputSource = std::make_shared<CStandardDataSource>();
    auto OutputSink = std::make_shared<CStandardDataSink>();
    auto ErrorSink = std::make_shared<CStandardErrorDataSink>();
    auto ResultsFactory = std::make_shared<CFileDataFactory>();
    
    // Load street map
    auto MapSource = std::make_shared<CFileDataSource>(options["--map"]);
    auto MapReader = std::make_shared<CXMLReader>(MapSource);
    auto StreetMap = std::make_shared<COpenStreetMap>(MapReader);
    
    if(!StreetMap->Load()) {
        *ErrorSink << "Error: Failed to load map from " << options["--map"] << std::endl;
        return 1;
    }
    
    // Load bus system
    auto BusSystem = std::make_shared<CCSVBusSystem>();
    if(!BusSystem->LoadFromDirectory(options["--bus"])) {
        *ErrorSink << "Error: Failed to load bus system from " << options["--bus"] << std::endl;
        return 1;
    }
    
    // Create transportation planner configuration
    struct STransportationPlannerConfig : public CTransportationPlanner::SConfiguration {
        std::shared_ptr<CStreetMap> DStreetMap;
        std::shared_ptr<CBusSystem> DBusSystem;
        
        STransportationPlannerConfig(std::shared_ptr<CStreetMap> streetmap, 
                                   std::shared_ptr<CBusSystem> bussystem)
            : DStreetMap(streetmap), DBusSystem(bussystem) {
        }
        
        std::shared_ptr<CStreetMap> StreetMap() const noexcept override {
            return DStreetMap;
        }
        
        std::shared_ptr<CBusSystem> BusSystem() const noexcept override {
            return DBusSystem;
        }
        
        double WalkSpeed() const noexcept override {
            return 3.0;
        }
        
        double BikeSpeed() const noexcept override {
            return 8.0;
        }
        
        double DefaultSpeedLimit() const noexcept override {
            return 25.0;
        }
    };
    
    auto Config = std::make_shared<STransportationPlannerConfig>(StreetMap, BusSystem);
    
    // Create transportation planner
    auto Planner = std::make_shared<CDijkstraTransportationPlanner>(Config);
    
    // Create command line interface
    auto CommandLine = std::make_unique<CTransportationPlannerCommandLine>(
        InputSource, OutputSink, ErrorSink, ResultsFactory, Planner
    );
    
    // Process commands
    *OutputSink << "Transportation Planner" << std::endl;
    *OutputSink << "Type 'help' for a list of commands" << std::endl;
    
    CommandLine->ProcessCommands();
    
    return 0;
}