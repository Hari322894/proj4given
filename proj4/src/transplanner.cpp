#include "TransportationPlannerCommandLine.h"
#include "FileDataSource.h"
#include "FileDataSink.h"
#include "FileDataFactory.h"
#include "DijkstraTransportationPlanner.h"
#include "STransportationPlannerConfig.h"
#include <memory>

int main(int argc, char *argv[]) {
    auto cmdsrc = std::make_shared<CFileDataSource>("input.txt");
    auto outsink = std::make_shared<CFileDataSink>("output.txt");
    auto errsink = std::make_shared<CFileDataSink>("error.txt");
    auto results = std::make_shared<CFileDataFactory>("");
    auto streetmap = std::make_shared<CStreetMap>(); // Assuming CStreetMap is defined and has a default constructor
    auto bussystem = std::make_shared<CBusSystem>(); // Assuming CBusSystem is defined and has a default constructor
    auto config = std::make_shared<STransportationPlannerConfig>(streetmap, bussystem);
    auto planner = std::make_shared<CDijkstraTransportationPlanner>(config);

    CTransportationPlannerCommandLine commandLine(cmdsrc, outsink, errsink, results, planner);
    if (!commandLine.ProcessCommands()) {
        return 1;
    }

    return 0;
}