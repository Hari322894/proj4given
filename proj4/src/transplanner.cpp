#include "TransportationPlannerCommandLine.h"
#include "FileDataSource.h"
#include "FileDataSink.h"
#include "FileDataFactory.h"
#include "DijkstraTransportationPlanner.h"
#include "STransportationPlannerConfig.h"
#include <memory>

class MockStreetMap : public CStreetMap {
public:
    std::size_t NodeCount() const noexcept override {
        return 0;
    }

    std::size_t WayCount() const noexcept override {
        return 0;
    }

    std::shared_ptr<SNode> NodeByIndex(std::size_t index) const noexcept override {
        return nullptr;
    }

    std::shared_ptr<SNode> NodeByID(TNodeID id) const noexcept override {
        return nullptr;
    }

    std::shared_ptr<SWay> WayByIndex(std::size_t index) const noexcept override {
        return nullptr;
    }

    std::shared_ptr<SWay> WayByID(TWayID id) const noexcept override {
        return nullptr;
    }
};

class MockBusSystem : public CBusSystem {
public:
    std::size_t StopCount() const noexcept override {
        return 0;
    }

    std::size_t RouteCount() const noexcept override {
        return 0;
    }

    std::shared_ptr<SStop> StopByIndex(std::size_t index) const noexcept override {
        return nullptr;
    }

    std::shared_ptr<SStop> StopByID(TStopID id) const noexcept override {
        return nullptr;
    }

    std::shared_ptr<SRoute> RouteByIndex(std::size_t index) const noexcept override {
        return nullptr;
    }

    std::shared_ptr<SRoute> RouteByName(const std::string &name) const noexcept override {
        return nullptr;
    }
};

int main(int argc, char *argv[]) {
    auto cmdsrc = std::make_shared<CFileDataSource>("input.txt");
    auto outsink = std::make_shared<CFileDataSink>("output.txt");
    auto errsink = std::make_shared<CFileDataSink>("error.txt");
    auto results = std::make_shared<CFileDataFactory>("");
    auto streetmap = std::make_shared<MockStreetMap>();
    auto bussystem = std::make_shared<MockBusSystem>();
    auto config = std::make_shared<STransportationPlannerConfig>(streetmap, bussystem);
    auto planner = std::make_shared<CDijkstraTransportationPlanner>(config);

    CTransportationPlannerCommandLine commandLine(cmdsrc, outsink, errsink, results, planner);
    if (!commandLine.ProcessCommands()) {
        return 1;
    }

    return 0;
}