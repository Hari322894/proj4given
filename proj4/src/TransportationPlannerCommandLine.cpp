#include "TransportationPlannerCommandLine.h"
#include <sstream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <cctype>

struct CTransportationPlannerCommandLine::SImplementation {
    std::shared_ptr<CDataSource> DCommandSource;
    std::shared_ptr<CDataSink> DOutputSink;
    std::shared_ptr<CDataSink> DErrorSink;
    std::shared_ptr<CDataFactory> DResultsFactory;
    std::shared_ptr<CTransportationPlanner> DPlanner;
    
    // Last calculated path data
    bool DPathValid;
    double DPathTime;
    CTransportationPlanner::TNodeID DPathSourceID;
    CTransportationPlanner::TNodeID DPathDestinationID;
    std::vector<CTransportationPlanner::TTripStep> DFastestPath;
    std::vector<CTransportationPlanner::TNodeID> DShortestPath;
    bool DIsShortestPath;
    
    SImplementation(std::shared_ptr<CDataSource> cmdsrc, 
                   std::shared_ptr<CDataSink> outsink, 
                   std::shared_ptr<CDataSink> errsink, 
                   std::shared_ptr<CDataFactory> results, 
                   std::shared_ptr<CTransportationPlanner> planner)
        : DCommandSource(cmdsrc), DOutputSink(outsink), DErrorSink(errsink),
          DResultsFactory(results), DPlanner(planner), DPathValid(false), 
          DIsShortestPath(false) {
    }
    
    bool ProcessCommands() {
        std::string Command, Line;
        std::vector<char> Buffer(1024);
        
        while(DCommandSource->Read(Buffer, 1024)) {
            Line.assign(Buffer.begin(), Buffer.end());
            std::stringstream LineStream(Line);
            
            // Print prompt and clear command
            DOutputSink->Write(std::vector<char>{'>', ' '});
            Command.clear();
            
            // Get the command
            LineStream >> Command;
            
            if(Command == "exit") {
                return true;
            }
            else if(Command == "help") {
                HandleHelpCommand();
            }
            else if(Command == "count") {
                HandleCountCommand();
            }
            else if(Command == "node") {
                HandleNodeCommand(LineStream);
            }
            else if(Command == "shortest") {
                HandleShortestCommand(LineStream);
            }
            else if(Command == "fastest") {
                HandleFastestCommand(LineStream);
            }
            else if(Command == "save") {
                HandleSaveCommand();
            }
            else if(Command == "print") {
                HandlePrintCommand();
            }
            else if(!Command.empty()) {
                DErrorSink->Write(GetVectorFromString("Unknown command \"" + Command + "\" type help for help.\n"));
            }
        }
        
        return true;
    }
    
    void HandleHelpCommand() {
        DOutputSink->Write(GetVectorFromString("------------------------------------------------------------------------\n"));
        DOutputSink->Write(GetVectorFromString("help     Display this help menu\n"));
        DOutputSink->Write(GetVectorFromString("exit     Exit the program\n"));
        DOutputSink->Write(GetVectorFromString("count    Output the number of nodes in the map\n"));
        DOutputSink->Write(GetVectorFromString("node     Syntax \"node [0, count)\" \n"));
        DOutputSink->Write(GetVectorFromString("         Will output node ID and Lat/Lon for node\n"));
        DOutputSink->Write(GetVectorFromString("fastest  Syntax \"fastest start end\" \n"));
        DOutputSink->Write(GetVectorFromString("         Calculates the time for fastest path from start to end\n"));
        DOutputSink->Write(GetVectorFromString("shortest Syntax \"shortest start end\" \n"));
        DOutputSink->Write(GetVectorFromString("         Calculates the distance for the shortest path from start to end\n"));
        DOutputSink->Write(GetVectorFromString("save     Saves the last calculated path to file\n"));
        DOutputSink->Write(GetVectorFromString("print    Prints the steps for the last calculated path\n"));
    }
    
    void HandleCountCommand() {
        size_t NodeCount = DPlanner->NodeCount();
        DOutputSink->Write(GetVectorFromString(std::to_string(NodeCount) + " nodes\n"));
    }
    
    void HandleNodeCommand(std::stringstream &lineStream) {
        int NodeIndex;
        
        if(!(lineStream >> NodeIndex)) {
            DErrorSink->Write(GetVectorFromString("Invalid node command, see help.\n"));
            return;
        }
        
        if(NodeIndex < 0 || static_cast<size_t>(NodeIndex) >= DPlanner->NodeCount()) {
            DErrorSink->Write(GetVectorFromString("Invalid node parameter, see help.\n"));
            return;
        }
        
        auto Node = DPlanner->SortedNodeByIndex(NodeIndex);
        auto Location = Node->Location();
        double Latitude = Location.first;
        double Longitude = Location.second;
        
        // Convert to degrees, minutes, seconds
        int LatDegrees = static_cast<int>(Latitude);
        int LatMinutes = static_cast<int>((Latitude - LatDegrees) * 60);
        int LatSeconds = static_cast<int>(((Latitude - LatDegrees) * 60 - LatMinutes) * 60);
        
        int LongDegrees = static_cast<int>(std::abs(Longitude));
        int LongMinutes = static_cast<int>((std::abs(Longitude) - LongDegrees) * 60);
        int LongSeconds = static_cast<int>(((std::abs(Longitude) - LongDegrees) * 60 - LongMinutes) * 60);
        
        std::string LatDirection = Latitude >= 0 ? "N" : "S";
        std::string LongDirection = Longitude >= 0 ? "E" : "W";
        
        std::stringstream OutputString;
        OutputString << "Node " << NodeIndex << ": id = " << Node->ID() 
                     << " is at " << std::abs(LatDegrees) << "d " << LatMinutes << "' " << LatSeconds << "\" " << LatDirection
                     << ", " << LongDegrees << "d " << LongMinutes << "' " << LongSeconds << "\" " << LongDirection << "\n";
        
        DOutputSink->Write(GetVectorFromString(OutputString.str()));
    }
    
    void HandleShortestCommand(std::stringstream &lineStream) {
        CTransportationPlanner::TNodeID SourceID, DestinationID;
        
        if(!(lineStream >> SourceID >> DestinationID)) {
            DErrorSink->Write(GetVectorFromString("Invalid shortest command, see help.\n"));
            return;
        }
        
        // Try to parse the IDs as numbers
        try {
            std::stoul(std::to_string(SourceID));
            std::stoul(std::to_string(DestinationID));
        } catch(...) {
            DErrorSink->Write(GetVectorFromString("Invalid shortest parameter, see help.\n"));
            return;
        }
        
        DShortestPath.clear();
        double Distance = DPlanner->FindShortestPath(SourceID, DestinationID, DShortestPath);
        
        DPathValid = true;
        DIsShortestPath = true;
        DPathSourceID = SourceID;
        DPathDestinationID = DestinationID;
        DPathTime = Distance;
        
        DOutputSink->Write(GetVectorFromString("Shortest path is " + std::to_string(Distance) + " mi.\n"));
    }
    
    void HandleFastestCommand(std::stringstream &lineStream) {
        CTransportationPlanner::TNodeID SourceID, DestinationID;
        
        if(!(lineStream >> SourceID >> DestinationID)) {
            DErrorSink->Write(GetVectorFromString("Invalid fastest command, see help.\n"));
            return;
        }
        
        // Try to parse the IDs as numbers
        try {
            std::stoul(std::to_string(SourceID));
            std::stoul(std::to_string(DestinationID));
        } catch(...) {
            DErrorSink->Write(GetVectorFromString("Invalid fastest parameter, see help.\n"));
            return;
        }
        
        DFastestPath.clear();
        double Time = DPlanner->FindFastestPath(SourceID, DestinationID, DFastestPath);
        
        DPathValid = true;
        DIsShortestPath = false;
        DPathSourceID = SourceID;
        DPathDestinationID = DestinationID;
        DPathTime = Time;
        
        // Format the time
        if(Time < 1.0) {
            // Less than an hour, show in minutes
            int Minutes = static_cast<int>(Time * 60);
            DOutputSink->Write(GetVectorFromString("Fastest path takes " + std::to_string(Minutes) + " min.\n"));
        } else {
            // Hours, minutes, and seconds
            int Hours = static_cast<int>(Time);
            int Minutes = static_cast<int>((Time - Hours) * 60);
            int Seconds = static_cast<int>(((Time - Hours) * 60 - Minutes) * 60);
            
            std::stringstream TimeStr;
            TimeStr << "Fastest path takes " << Hours << " hr";
            
            if(Minutes > 0 || Seconds > 0) {
                TimeStr << " " << Minutes << " min";
            }
            
            if(Seconds > 0) {
                TimeStr << " " << Seconds << " sec";
            }
            
            TimeStr << ".\n";
            DOutputSink->Write(GetVectorFromString(TimeStr.str()));
        }
    }
    
    void HandleSaveCommand() {
        if(!DPathValid) {
            DErrorSink->Write(GetVectorFromString("No valid path to save, see help.\n"));
            return;
        }
        
        std::string Filename = std::to_string(DPathSourceID) + "_" + 
                              std::to_string(DPathDestinationID) + "_" + 
                              std::to_string(DPathTime) + 
                              (DIsShortestPath ? "mi.csv" : "hr.csv");
        
        auto SaveSink = DResultsFactory->CreateSink(Filename);
        
        if(!SaveSink) {
            DErrorSink->Write(GetVectorFromString("Unable to create save file.\n"));
            return;
        }
        
        // Write header
        SaveSink->Write(GetVectorFromString("mode,node_id\n"));
        
        // Write data
        if(DIsShortestPath) {
            // Shortest path only has node IDs
            for(const auto &NodeID : DShortestPath) {
                SaveSink->Write(GetVectorFromString("Walk," + std::to_string(NodeID) + "\n"));
            }
        } else {
            // Fastest path has mode and node ID
            for(const auto &Step : DFastestPath) {
                std::string Mode;
                
                switch(Step.first) {
                    case CTransportationPlanner::ETransportationMode::Walk:
                        Mode = "Walk";
                        break;
                    case CTransportationPlanner::ETransportationMode::Bike:
                        Mode = "Bike";
                        break;
                    case CTransportationPlanner::ETransportationMode::Bus:
                        Mode = "Bus";
                        break;
                }
                
                SaveSink->Write(GetVectorFromString(Mode + "," + std::to_string(Step.second) +
                               ((&Step != &DFastestPath.back()) ? "\n" : "")));
            }
        }
        
        DOutputSink->Write(GetVectorFromString("Path saved to <results>/" + Filename + "\n"));
    }
    
    void HandlePrintCommand() {
        if(!DPathValid) {
            DErrorSink->Write(GetVectorFromString("No valid path to print, see help.\n"));
            return;
        }
        
        if(DIsShortestPath) {
            // For shortest path, we don't have detailed steps
            DOutputSink->Write(GetVectorFromString("Shortest path is " + std::to_string(DPathTime) + " mi.\n"));
        } else {
            // For fastest path, get the description
            std::vector<std::string> PathDescription;
            
            if(DPlanner->GetPathDescription(DFastestPath, PathDescription)) {
                for(const auto &Line : PathDescription) {
                    DOutputSink->Write(GetVectorFromString(Line + "\n"));
                }
            } else {
                DErrorSink->Write(GetVectorFromString("Unable to get path description.\n"));
            }
        }
    }

    std::vector<char> GetVectorFromString(const std::string &str) {
        return std::vector<char>(str.begin(), str.end());
    }
};

CTransportationPlannerCommandLine::CTransportationPlannerCommandLine(
    std::shared_ptr<CDataSource> cmdsrc, 
    std::shared_ptr<CDataSink> outsink, 
    std::shared_ptr<CDataSink> errsink, 
    std::shared_ptr<CDataFactory> results, 
    std::shared_ptr<CTransportationPlanner> planner) {
    
    DImplementation = std::make_unique<SImplementation>(cmdsrc, outsink, errsink, results, planner);
}

CTransportationPlannerCommandLine::~CTransportationPlannerCommandLine() {
    // Destructor definition
}

bool CTransportationPlannerCommandLine::ProcessCommands() {
    return DImplementation->ProcessCommands();
}