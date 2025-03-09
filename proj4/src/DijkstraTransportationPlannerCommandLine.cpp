#include "TransportationPlannerCommandLine.h"
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>

struct CTransportationPlannerCommandLine::SImplementation {
    std::shared_ptr<CDataSource> DCommandSource;
    std::shared_ptr<CDataSink> DOutputSink;
    std::shared_ptr<CDataSink> DErrorSink;
    std::shared_ptr<CDataFactory> DResultsFactory;
    std::shared_ptr<CTransportationPlanner> DPlanner;

    SImplementation(std::shared_ptr<CDataSource> cmdsrc, 
                   std::shared_ptr<CDataSink> outsink, 
                   std::shared_ptr<CDataSink> errsink, 
                   std::shared_ptr<CDataFactory> results,
                   std::shared_ptr<CTransportationPlanner> planner)
        : DCommandSource(cmdsrc), DOutputSink(outsink), DErrorSink(errsink),
          DResultsFactory(results), DPlanner(planner) {
    }

    // Helper to convert string to lowercase
    std::string ToLower(const std::string &str) const {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), 
                      [](unsigned char c){ return std::tolower(c); });
        return result;
    }

    // Helper to parse node ID from string
    bool ParseNodeID(const std::string &str, CStreetMap::TNodeID &nodeID) const {
        try {
            nodeID = std::stoull(str);
            return true;
        } catch(...) {
            return false;
        }
    }

    // Display help information
    void ShowHelp() const {
        std::stringstream ss;
        ss << "Commands:" << std::endl;
        ss << "  help               - Show this help message" << std::endl;
        ss << "  shortest <start> <end> - Find shortest path from start to end node" << std::endl;
        ss << "  fastest <start> <end>  - Find fastest path from start to end node" << std::endl;
        ss << "  describe <start> <end> - Describe path from start to end node" << std::endl;
        ss << "  exit               - Exit the command line interface" << std::endl;
        *DOutputSink << ss.str();
    }

    // Process a command from the source
    bool ProcessCommand(const std::string &command) {
        std::stringstream ss(command);
        std::string cmd;
        ss >> cmd;
        cmd = ToLower(cmd);

        if(cmd == "exit" || cmd == "quit") {
            return false; // Stop processing commands
        } else if(cmd == "help") {
            ShowHelp();
        } else if(cmd == "shortest" || cmd == "fastest" || cmd == "describe") {
            CStreetMap::TNodeID StartNode, EndNode;
            std::string StartNodeStr, EndNodeStr;
            ss >> StartNodeStr >> EndNodeStr;

            if(!ParseNodeID(StartNodeStr, StartNode) || !ParseNodeID(EndNodeStr, EndNode)) {
                *DErrorSink << "Invalid node IDs provided." << std::endl;
                return true;
            }

            // Get transportation planner node IDs
            auto SrcNodeID = DPlanner->GetNodeID(StartNode);
            auto DestNodeID = DPlanner->GetNodeID(EndNode);

            if(SrcNodeID == CTransportationPlanner::InvalidNodeID || 
               DestNodeID == CTransportationPlanner::InvalidNodeID) {
                *DErrorSink << "Invalid node IDs provided: not found in transportation network." << std::endl;
                return true;
            }

            if(cmd == "shortest") {
                std::vector<CTransportationPlanner::TNodeID> Path;
                double Distance = DPlanner->FindShortestPath(SrcNodeID, DestNodeID, Path);
                
                if(Distance == CPathRouter::NoPathExists) {
                    *DErrorSink << "No path found from " << StartNode << " to " << EndNode << "." << std::endl;
                } else {
                    *DOutputSink << "Shortest path from " << StartNode << " to " << EndNode << " is " 
                                << std::fixed << std::setprecision(2) << Distance << " miles." << std::endl;
                    
                    if(!Path.empty()) {
                        *DOutputSink << "Path: ";
                        for(size_t i = 0; i < Path.size(); i++) {
                            auto StreetNodeID = DPlanner->GetStreetMapNodeID(Path[i]);
                            auto BusStopID = DPlanner->GetBusSystemStopID(Path[i]);
                            
                            if(StreetNodeID != CStreetMap::InvalidNodeID) {
                                *DOutputSink << StreetNodeID;
                            } else if(BusStopID != CBusSystem::InvalidStopID) {
                                *DOutputSink << "B" << BusStopID;
                            } else {
                                *DOutputSink << Path[i];
                            }
                            
                            if(i < Path.size() - 1) {
                                *DOutputSink << " ";
                            }
                        }
                        *DOutputSink << std::endl;
                    }
                }
            } else if(cmd == "fastest") {
                std::vector<CTransportationPlanner::TTripStep> Path;
                double Time = DPlanner->FindFastestPath(SrcNodeID, DestNodeID, Path);
                
                if(Time == CPathRouter::NoPathExists) {
                    *DErrorSink << "No path found from " << StartNode << " to " << EndNode << "." << std::endl;
                } else {
                    *DOutputSink << "Fastest path from " << StartNode << " to " << EndNode 
                                << " takes " << std::fixed << std::setprecision(2) << Time << " hours." << std::endl;
                    
                    if(!Path.empty()) {
                        *DOutputSink << "Path: ";
                        for(size_t i = 0; i < Path.size(); i++) {
                            *DOutputSink << "(" << static_cast<int>(Path[i].first) << ", ";
                            
                            auto StreetNodeID = DPlanner->GetStreetMapNodeID(Path[i].second);
                            auto BusStopID = DPlanner->GetBusSystemStopID(Path[i].second);
                            
                            if(StreetNodeID != CStreetMap::InvalidNodeID) {
                                *DOutputSink << StreetNodeID;
                            } else if(BusStopID != CBusSystem::InvalidStopID) {
                                *DOutputSink << "B" << BusStopID;
                            } else {
                                *DOutputSink << Path[i].second;
                            }
                            
                            *DOutputSink << ")";
                            
                            if(i < Path.size() - 1) {
                                *DOutputSink << " ";
                            }
                        }
                        *DOutputSink << std::endl;
                    }
                }
            } else if(cmd == "describe") {
                std::vector<CTransportationPlanner::TTripStep> Path;
                double Time = DPlanner->FindFastestPath(SrcNodeID, DestNodeID, Path);
                
                if(Time == CPathRouter::NoPathExists) {
                    *DErrorSink << "No path found from " << StartNode << " to " << EndNode << "." << std::endl;
                } else {
                    std::vector<std::string> Description;
                    if(DPlanner->GetPathDescription(Path, Description)) {
                        for(const auto &Line : Description) {
                            *DOutputSink << Line << std::endl;
                        }
                    } else {
                        *DErrorSink << "Failed to get path description." << std::endl;
                    }
                }
            }
        } else {
            *DErrorSink << "Unknown command: " << cmd << std::endl;
        }

        return true;
    }

    // Main loop to process commands
    bool ProcessCommands() {
        std::string Command;
        while(DCommandSource->Read(Command)) {
            if(!ProcessCommand(Command)) {
                break;
            }
        }
        return true;
    }
};

CTransportationPlannerCommandLine::CTransportationPlannerCommandLine(std::shared_ptr<CDataSource> cmdsrc, 
                                                                     std::shared_ptr<CDataSink> outsink, 
                                                                     std::shared_ptr<CDataSink> errsink, 
                                                                     std::shared_ptr<CDataFactory> results, 
                                                                     std::shared_ptr<CTransportationPlanner> planner)
    : DImplementation(std::make_unique<SImplementation>(cmdsrc, outsink, errsink, results, planner)) {
}

CTransportationPlannerCommandLine::~CTransportationPlannerCommandLine() {
}

bool CTransportationPlannerCommandLine::ProcessCommands() {
    return DImplementation->ProcessCommands();
}