#include "TransportationPlannerCommandLine.h"
#include <memory>

int main(int argc, char *argv[]) {
    auto cmdsrc = std::make_shared<CDataSource>();
    auto outsink = std::make_shared<CDataSink>();
    auto errsink = std::make_shared<CDataSink>();
    auto results = std::make_shared<CDataFactory>();
    auto planner = std::make_shared<CTransportationPlanner>();

    CTransportationPlannerCommandLine commandLine(cmdsrc, outsink, errsink, results, planner);
    if (!commandLine.ProcessCommands()) {
        return 1;
    }

    return 0;
}