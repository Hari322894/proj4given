#include "TransportationPlannerCommandLine.h"
#include <iostream>

struct CTransportationPlannerCommandLine::SImplementation {
    std::shared_ptr<CDataSource> CmdSource;
    std::shared_ptr<CDataSink> OutSink;
    std::shared_ptr<CDataSink> ErrSink;
    std::shared_ptr<CDataFactory> DataFactory;
    std::shared_ptr<CTransportationPlanner> Planner;

    SImplementation(std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink,
                    std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results,
                    std::shared_ptr<CTransportationPlanner> planner)
        : CmdSource(cmdsrc), OutSink(outsink), ErrSink(errsink), DataFactory(results), Planner(planner) {}

    bool ProcessCommands() {
        // Implement command processing here
        std::string command;
        while (!CmdSource->End()) {
            if (CmdSource->Get(command)) {
                if (command == "NodeCount") {
                    auto nodeCount = Planner->NodeCount();
                    OutSink->Write("NodeCount: " + std::to_string(nodeCount) + "\n");
                } else if (command == "FindShortestPath") {
                    // Example processing for FindShortestPath command
                    // You need to implement actual command parsing and processing logic
                    TNodeID src, dest;
                    std::vector<TNodeID> path;
                    double distance = Planner->FindShortestPath(src, dest, path);
                    OutSink->Write("Shortest Path Distance: " + std::to_string(distance) + "\n");
                } else if (command == "FindFastestPath") {
                    // Example processing for FindFastestPath command
                    // You need to implement actual command parsing and processing logic
                    TNodeID src, dest;
                    std::vector<TTripStep> path;
                    double time = Planner->FindFastestPath(src, dest, path);
                    OutSink->Write("Fastest Path Time: " + std::to_string(time) + "\n");
                } else {
                    ErrSink->Write("Unknown command: " + command + "\n");
                }
            }
        }
        return true;
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