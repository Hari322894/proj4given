#include "TransportationPlannerCommandLine.h"
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <algorithm>

struct CTransportationPlannerCommandLine::SImplementation {
    std::shared_ptr<CDataSource> DCommandSource;
    std::shared_ptr<CDataSink> DOutputSink;
    std::shared_ptr<CDataSink> DErrorSink;
    std::shared_ptr<CDataFactory> DResultFactory;
    std::shared_ptr<CTransportationPlanner> DPlanner;

    SImplementation(std::shared_ptr<CDataSource> cmdsrc, 
                   std::shared_ptr<CDataSink> outsink, 
                   std::shared_ptr<CDataSink> errsink, 
                   std::shared_ptr<CDataFactory> results, 
                   std::shared_ptr<CTransportationPlanner> planner) 
        : DCommandSource(cmdsrc), DOutputSink(outsink), 
          DErrorSink(errsink), DResultFactory(results), DPlanner(planner) {
    }

    // Helper function to convert string to vector<char>
    std::vector<char> StringToVector(const std::string &str) {
        return std::vector<char>(str.begin(), str.end());
    }

    bool ProcessCommands() {
        std::vector<char> buffer(1024); // Buffer for reading commands
        std::string command;
        
        // Keep reading commands until end of input
        size_t bytesRead;
        while((bytesRead = DCommandSource->Read(buffer, buffer.size())) > 0) {
            // Convert buffer to string with proper length
            command = std::string(buffer.data(), bytesRead);
            
            // Check for newline character and process complete commands
            if(command.find('\n') != std::string::npos) {
                std::stringstream CommandStream(command);
                std::string CommandType;
                CommandStream >> CommandType;
    
                // Handle different command types
                if(CommandType == "help") {
                    ProcessHelpCommand();
                }
                else if(CommandType == "count") {
                    ProcessCountCommand();
                }
                else if(CommandType == "node") {
                    ProcessNodeCommand(CommandStream);
                }
                else if(CommandType == "shortest") {
                    ProcessShortestPathCommand(CommandStream);
                }
                else if(CommandType == "fastest") {
                    ProcessFastestPathCommand(CommandStream);
                }
                else if(CommandType.empty() || CommandType[0] == '\n' || CommandType[0] == '\r') {
                    // Empty line, just ignore
                    continue;
                }
                else {
                    // Unknown command
                    std::string ErrorMessage = "Unknown command: " + CommandType + "\n";
                    DErrorSink->Write(StringToVector(ErrorMessage));
                }
            }
            else {
                // Incomplete command, write error
                std::string ErrorMessage = "Incomplete command (missing newline)\n";
                DErrorSink->Write(StringToVector(ErrorMessage));
            }
            
            // Clear buffer for next read
            std::fill(buffer.begin(), buffer.end(), 0);
        }
        
        return true;
    }
    
    void ProcessHelpCommand() {
        std::string HelpMessage = 
            "help - Display this help message\n"
            "count - Display the number of nodes in the transportation network\n"
            "node <index> - Display information about the node at the specified index\n"
            "shortest <srcID> <destID> - Find the shortest path between two nodes\n"
            "fastest <srcID> <destID> - Find the fastest path between two nodes\n";
        
        DOutputSink->Write(StringToVector(HelpMessage));
    }

    void ProcessCountCommand() {
        std::string CountMessage = "Node Count: " + std::to_string(DPlanner->NodeCount()) + "\n";
        DOutputSink->Write(StringToVector(CountMessage));
    }

    void ProcessNodeCommand(std::stringstream &CommandStream) {
        std::size_t NodeIndex;
        
        if(CommandStream >> NodeIndex) {
            auto Node = DPlanner->SortedNodeByIndex(NodeIndex);
            
            if(Node) {
                std::stringstream NodeInfo;
                NodeInfo << "Node[" << NodeIndex << "]:\n";
                NodeInfo << "  ID: " << Node->ID() << "\n";
                NodeInfo << "  Lat, Lon: " << std::fixed << std::setprecision(6) 
                        << Node->Location().first << ", " 
                        << Node->Location().second << "\n";
                
                DOutputSink->Write(StringToVector(NodeInfo.str()));
            }
            else {
                std::string ErrorMessage = "Invalid node index: " + std::to_string(NodeIndex) + "\n";
                DErrorSink->Write(StringToVector(ErrorMessage));
            }
        }
        else {
            std::string ErrorMsg = "Invalid node command format. Usage: node <index>\n";
            DErrorSink->Write(StringToVector(ErrorMsg));
        }
    }

    void ProcessShortestPathCommand(std::stringstream &CommandStream) {
        CTransportationPlanner::TNodeID SrcID, DestID;
        
        if(CommandStream >> SrcID >> DestID) {
            std::vector<CTransportationPlanner::TNodeID> Path;
            double Distance = DPlanner->FindShortestPath(SrcID, DestID, Path);
            
            if(Distance != std::numeric_limits<double>::max()) {
                std::stringstream PathInfo;
                PathInfo << "Shortest Path from " << SrcID << " to " << DestID << ":\n";
                PathInfo << "  Distance: " << std::fixed << std::setprecision(2) << Distance << " km\n";
                PathInfo << "  Path: ";
                
                for(std::size_t i = 0; i < Path.size(); ++i) {
                    PathInfo << Path[i];
                    if(i < Path.size() - 1) {
                        PathInfo << " -> ";
                    }
                }
                PathInfo << "\n";
                
                DOutputSink->Write(StringToVector(PathInfo.str()));
            }
            else {
                std::string ErrorMessage = "No path exists between " + std::to_string(SrcID) + " and " + std::to_string(DestID) + "\n";
                DErrorSink->Write(StringToVector(ErrorMessage));
            }
        }
        else {
            std::string ErrorMsg = "Invalid shortest command format. Usage: shortest <srcID> <destID>\n";
            DErrorSink->Write(StringToVector(ErrorMsg));
        }
    }

    void ProcessFastestPathCommand(std::stringstream &CommandStream) {
        CTransportationPlanner::TNodeID SrcID, DestID;
        
        if(CommandStream >> SrcID >> DestID) {
            std::vector<CTransportationPlanner::TTripStep> Path;
            double Time = DPlanner->FindFastestPath(SrcID, DestID, Path);
            
            if(Time != std::numeric_limits<double>::max()) {
                std::stringstream PathInfo;
                PathInfo << "Fastest Path from " << SrcID << " to " << DestID << ":\n";
                PathInfo << "  Time: " << std::fixed << std::setprecision(2) << Time << " min\n";
                
                // Get path description if possible
                std::vector<std::string> Description;
                if(DPlanner->GetPathDescription(Path, Description)) {
                    PathInfo << "  Description:\n";
                    for(const auto &Step : Description) {
                        PathInfo << "    " << Step << "\n";
                    }
                }
                else {
                    // Simple path output
                    PathInfo << "  Path: ";
                    for(std::size_t i = 0; i < Path.size(); ++i) {
                        std::string ModeStr;
                        switch(Path[i].first) {
                            case CTransportationPlanner::ETransportationMode::Walk: ModeStr = "Walk"; break;
                            case CTransportationPlanner::ETransportationMode::Bike: ModeStr = "Bike"; break;
                            case CTransportationPlanner::ETransportationMode::Bus: ModeStr = "Bus"; break;
                        }
                        
                        PathInfo << ModeStr << "(" << Path[i].second << ")";
                        if(i < Path.size() - 1) {
                            PathInfo << " -> ";
                        }
                    }
                    PathInfo << "\n";
                }
                
                DOutputSink->Write(StringToVector(PathInfo.str()));
            }
            else {
                std::string ErrorMessage = "No path exists between " + std::to_string(SrcID) + " and " + std::to_string(DestID) + "\n";
                DErrorSink->Write(StringToVector(ErrorMessage));
            }
        }
        else {
            std::string ErrorMsg = "Invalid fastest command format. Usage: fastest <srcID> <destID>\n";
            DErrorSink->Write(StringToVector(ErrorMsg));
        }
    }
};

CTransportationPlannerCommandLine::CTransportationPlannerCommandLine(std::shared_ptr<CDataSource> cmdsrc, 
                                                                   std::shared_ptr<CDataSink> outsink, 
                                                                   std::shared_ptr<CDataSink> errsink, 
                                                                   std::shared_ptr<CDataFactory> results, 
                                                                   std::shared_ptr<CTransportationPlanner> planner) {
    DImplementation = std::make_unique<SImplementation>(cmdsrc, outsink, errsink, results, planner);
}

CTransportationPlannerCommandLine::~CTransportationPlannerCommandLine() = default;

bool CTransportationPlannerCommandLine::ProcessCommands() {
    return DImplementation->ProcessCommands();
}