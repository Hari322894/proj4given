#include <iostream>
#include <string>
#include <sstream>
#include <memory>
#include <vector>
#include <algorithm>
#include "CSVBusSystem.h"
#include "OpenStreetMap.h"
#include "DijkstraPathRouter.h"
#include "CSVOSMTransportationPlanner.h"
#include "StringUtils.h"
#include "GeographicUtils.h"
#include "KMLWriter.h"
#include "FileDataSink.h"

// Function to print help information
void PrintHelp() {
    std::cout << "TransPlanner Commands:\n"
              << "  help                           - Display this help message\n"
              << "  load bus <csv_file>            - Load bus system data from CSV file\n"
              << "  load map <osm_file>            - Load map data from OSM file\n"
              << "  find path <start> <end>        - Find path between locations (coordinates or addresses)\n"
              << "  export kml <file> <path>       - Export path to KML file\n"
              << "  info                           - Display loaded data information\n"
              << "  exit                           - Exit the program\n";
}

// Function to parse coordinates from string input
bool ParseCoordinates(const std::string& input, double& lat, double& lon) {
    std::istringstream ss(input);
    char comma;
    if (ss >> lat >> comma >> lon && comma == ',') {
        return true;
    }
    return false;
}

int main() {
    std::cout << "TransPlanner - Transportation Planning System\n";
    std::cout << "Type 'help' for a list of commands.\n";

    std::shared_ptr<CCSVBusSystem> BusSystem;
    std::shared_ptr<COpenStreetMap> StreetMap;
    std::shared_ptr<CCSVOSMTransportationPlanner> Planner;
    std::vector<std::shared_ptr<STransportationPlanner::SLocation>> LastPath;

    std::string line;
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, line);
        line = StringUtils::Trim(line);

        if (line.empty()) {
            continue;
        }

        std::istringstream iss(line);
        std::string command;
        iss >> command;

        if (command == "exit") {
            break;
        }
        else if (command == "help") {
            PrintHelp();
        }
        else if (command == "load") {
            std::string dataType;
            iss >> dataType;

            if (dataType == "bus") {
                std::string filename;
                iss >> filename;
                try {
                    BusSystem = std::make_shared<CCSVBusSystem>(filename);
                    std::cout << "Bus system data loaded from " << filename << "\n";

                    // If both data sources are loaded, create the planner
                    if (BusSystem && StreetMap) {
                        Planner = std::make_shared<CCSVOSMTransportationPlanner>(BusSystem, StreetMap);
                        std::cout << "Transportation planner created with loaded data.\n";
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Error loading bus data: " << e.what() << "\n";
                }
            }
            else if (dataType == "map") {
                std::string filename;
                iss >> filename;
                try {
                    StreetMap = std::make_shared<COpenStreetMap>(filename);
                    std::cout << "Map data loaded from " << filename << "\n";

                    // If both data sources are loaded, create the planner
                    if (BusSystem && StreetMap) {
                        Planner = std::make_shared<CCSVOSMTransportationPlanner>(BusSystem, StreetMap);
                        std::cout << "Transportation planner created with loaded data.\n";
                    }
                }
                catch (const std::exception& e) {
                    std::cerr << "Error loading map data: " << e.what() << "\n";
                }
            }
            else {
                std::cerr << "Unknown data type for loading. Use 'bus' or 'map'.\n";
            }
        }
        else if (command == "find") {
            std::string subcommand;
            iss >> subcommand;

            if (subcommand == "path") {
                if (!Planner) {
                    std::cerr << "Error: Transportation planner not initialized. Load bus and map data first.\n";
                    continue;
                }

                std::string startInput, endInput;
                // Get the remaining line for start and end locations
                std::getline(iss, startInput);
                startInput = StringUtils::Trim(startInput);
                
                // Split the input to get start and end locations
                size_t separatorPos = startInput.find(" to ");
                if (separatorPos == std::string::npos) {
                    std::cerr << "Error: Invalid format. Use 'find path <start> to <end>'\n";
                    continue;
                }
                
                startInput = StringUtils::Trim(startInput.substr(0, separatorPos));
                endInput = StringUtils::Trim(startInput.substr(separatorPos + 4));
                
                // Parse locations (either as coordinates or addresses)
                std::shared_ptr<STransportationPlanner::SLocation> startLoc, endLoc;
                double startLat, startLon, endLat, endLon;
                
                if (ParseCoordinates(startInput, startLat, startLon)) {
                    // Input is coordinates
                    startLoc = Planner->FindClosestLocation(startLat, startLon);
                }
                else {
                    // Input is an address
                    auto locations = Planner->FindLocationByAddress(startInput);
                    if (locations.empty()) {
                        std::cerr << "Error: Could not find start location: " << startInput << "\n";
                        continue;
                    }
                    startLoc = locations[0];
                }
                
                if (ParseCoordinates(endInput, endLat, endLon)) {
                    // Input is coordinates
                    endLoc = Planner->FindClosestLocation(endLat, endLon);
                }
                else {
                    // Input is an address
                    auto locations = Planner->FindLocationByAddress(endInput);
                    if (locations.empty()) {
                        std::cerr << "Error: Could not find end location: " << endInput << "\n";
                        continue;
                    }
                    endLoc = locations[0];
                }
                
                // Find path
                LastPath.clear();
                bool pathFound = Planner->FindPath(startLoc, endLoc, LastPath);
                
                if (pathFound) {
                    std::cout << "Path found with " << LastPath.size() << " locations.\n";
                    
                    // Display the path details
                    for (size_t i = 0; i < LastPath.size(); ++i) {
                        const auto& loc = LastPath[i];
                        std::cout << i + 1 << ". ";
                        
                        // Print location info
                        if (!loc->Name.empty()) {
                            std::cout << loc->Name;
                        }
                        else {
                            std::cout << "(" << loc->Latitude << ", " << loc->Longitude << ")";
                        }
                        
                        // Print node type
                        std::cout << " - " << (loc->Type == STransportationPlanner::ELocationType::BusStop ? "Bus Stop" : "Street Corner") << "\n";
                    }
                }
                else {
                    std::cerr << "No path found between the specified locations.\n";
                }
            }
            else {
                std::cerr << "Unknown find subcommand. Use 'path'.\n";
            }
        }
        else if (command == "export") {
            std::string format, filename;
            iss >> format >> filename;
            
            if (format == "kml") {
                if (LastPath.empty()) {
                    std::cerr << "No path to export. Find a path first.\n";
                    continue;
                }
                
                try {
                    // Create a KML writer and export the path
                    auto sink = std::make_shared<CFileDataSink>(filename);
                    CKMLWriter writer(sink);
                    
                    // Start the KML document
                    writer.StartDocument("TransPlanner Path");
                    writer.StartFolder("Path");
                    
                    // Add path as a placemark
                    writer.StartPlacemark("Path");
                    writer.StartLineString();
                    
                    for (const auto& loc : LastPath) {
                        writer.AddCoordinate(loc->Longitude, loc->Latitude, 0);
                    }
                    
                    writer.EndLineString();
                    writer.EndPlacemark();
                    
                    // Add points for each location
                    for (size_t i = 0; i < LastPath.size(); ++i) {
                        const auto& loc = LastPath[i];
                        std::string name = loc->Name.empty() ? 
                            "Point " + std::to_string(i + 1) : 
                            loc->Name;
                        
                        writer.StartPlacemark(name);
                        writer.AddPoint(loc->Longitude, loc->Latitude, 0);
                        writer.EndPlacemark();
                    }
                    
                    writer.EndFolder();
                    writer.EndDocument();
                    
                    std::cout << "Path exported to " << filename << "\n";
                }
                catch (const std::exception& e) {
                    std::cerr << "Error exporting to KML: " << e.what() << "\n";
                }
            }
            else {
                std::cerr << "Unsupported export format. Use 'kml'.\n";
            }
        }
        else if (command == "info") {
            if (BusSystem) {
                std::cout << "Bus System: Loaded with " << BusSystem->NodeCount() << " stops and " 
                          << BusSystem->RouteCount() << " routes.\n";
            }
            else {
                std::cout << "Bus System: Not loaded\n";
            }
            
            if (StreetMap) {
                std::cout << "Street Map: Loaded with " << StreetMap->NodeCount() << " nodes and "
                          << StreetMap->WayCount() << " ways.\n";
            }
            else {
                std::cout << "Street Map: Not loaded\n";
            }
            
            if (Planner) {
                std::cout << "Transportation Planner: Initialized\n";
            }
            else {
                std::cout << "Transportation Planner: Not initialized\n";
            }
        }
        else {
            std::cerr << "Unknown command: " << command << "\n";
            std::cerr << "Type 'help' for a list of commands.\n";
        }
    }
    
    std::cout << "TransPlanner exiting. Goodbye!\n";
    return 0;
}