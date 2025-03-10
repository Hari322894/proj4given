#include "TransportationPlannerCommandLine.h"
#include <iostream>
#include <string>
#include <vector>

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
        std::vector<char> commandBuffer;
        std::string command;
        while (!CmdSource->End()) {
            char ch;
            while (CmdSource->Get(ch)) {
                if (ch == '\n') {
                    break;
                }
                commandBuffer.push_back(ch);
            }
            command = std::string(commandBuffer.begin(), commandBuffer.end());
            commandBuffer.clear();

            if (command == "NodeCount") {
                auto nodeCount = Planner->NodeCount();
                WriteToSink(OutSink, "NodeCount: " + std::to_string(nodeCount) + "\n");
            } else if (command == "FindShortestPath") {
                TNodeID src = 1, dest = 2; // Example values, replace with actual parsing
                std::vector<TNodeID> path;
                double distance = Planner->FindShortestPath(src, dest, path);
                WriteToSink(OutSink, "Shortest Path Distance: " + std::to_string(distance) + "\n");
            } else if (command == "FindFastestPath") {
                TNodeID src = 1, dest = 2; // Example values, replace with actual parsing
                std::vector<TTripStep> path;
                double time = Planner->FindFastestPath(src, dest, path);
                WriteToSink(OutSink, "Fastest Path Time: " + std::to_string(time) + "\n");
            } else {
                WriteToSink(ErrSink, "Unknown command: " + command + "\n");
            }
        }
        return true;
    }

    void WriteToSink(std::shared_ptr<CDataSink> sink, const std::string &message) {
        std::vector<char> buffer(message.begin(), message.end());
        sink->Write(buffer);
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