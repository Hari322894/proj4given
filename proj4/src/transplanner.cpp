#include "TransportationPlannerCommandLine.h"
#include "FileDataSource.h"
#include "FileDataSink.h"
#include "FileDataFactory.h"
#include "DijkstraTransportationPlanner.h"
#include <memory>

int main(int argc, char *argv[]) {
    auto cmdsrc = std::make_shared<FileDataSource>("input.txt");
    auto outsink = std::make_shared<FileDataSink>("output.txt");
    auto errsink = std::make_shared<FileDataSink>("error.txt");
    auto results = std::make_shared<FileDataFactory>();
    auto config = std::make_shared<STransportationPlannerConfig>();
    auto planner = std::make_shared<CDijkstraTransportationPlanner>(config);

    CTransportationPlannerCommandLine commandLine(cmdsrc, outsink, errsink, results, planner);
    if (!commandLine.ProcessCommands()) {
        return 1;
    }

    return 0;
}