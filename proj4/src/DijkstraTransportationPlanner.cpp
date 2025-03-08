#include "DijkstraTransportationPlanner.h"
#include "GeographicUtils.h"
#include <queue>
#include <limits>
#include <unordered_map>

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(const std::shared_ptr<STransportationPlannerConfig> &Config)
    : m_Config(Config) {}

CDijkstraTransportationPlanner::TPathResult CDijkstraTransportationPlanner::FindShortestPath(TNodeID StartNodeID, TNodeID EndNodeID,
                                                                                             std::vector<TNodeID> &Path) {
    Path.clear();
    if (StartNodeID == EndNodeID) return 0.0;

    std::unordered_map<TNodeID, double> Distances;
    std::unordered_map<TNodeID, TNodeID> Previous;
    auto Compare = [&Distances](const TNodeID &A, const TNodeID &B) { return Distances[A] > Distances[B]; };
    std::priority_queue<TNodeID, std::vector<TNodeID>, decltype(Compare)> Queue(Compare);

    for (const auto &Node : m_Config->m_StreetMap->Nodes()) {
        Distances[Node.first] = std::numeric_limits<double>::infinity();
    }
    Distances[StartNodeID] = 0.0;
    Queue.push(StartNodeID);

    while (!Queue.empty()) {
        TNodeID Current = Queue.top();
        Queue.pop();
        if (Current == EndNodeID) break;

        for (const auto &Neighbor : m_Config->m_StreetMap->Neighbors(Current)) {
            double Alt = Distances[Current] + SGeographicUtils::HaversineDistanceInMiles(
                m_Config->m_StreetMap->NodePosition(Current), m_Config->m_StreetMap->NodePosition(Neighbor));
            if (Alt < Distances[Neighbor]) {
                Distances[Neighbor] = Alt;
                Previous[Neighbor] = Current;
                Queue.push(Neighbor);
            }
        }
    }

    if (Distances[EndNodeID] == std::numeric_limits<double>::infinity()) {
        return CPathRouter::NoPathExists;
    }

    for (TNodeID At = EndNodeID; At != StartNodeID; At = Previous[At]) {
        Path.push_back(At);
    }
    Path.push_back(StartNodeID);
    std::reverse(Path.begin(), Path.end());
    return Distances[EndNodeID];
}

CDijkstraTransportationPlanner::TPathResult CDijkstraTransportationPlanner::FindFastestPath(TNodeID StartNodeID, TNodeID EndNodeID,
                                                                                            std::vector<TTripStep> &Path) {
    Path.clear();
    if (StartNodeID == EndNodeID) return 0.0;

    std::unordered_map<TNodeID, double> Times;
    std::unordered_map<TNodeID, TNodeID> Previous;
    auto Compare = [&Times](const TNodeID &A, const TNodeID &B) { return Times[A] > Times[B]; };
    std::priority_queue<TNodeID, std::vector<TNodeID>, decltype(Compare)> Queue(Compare);

    for (const auto &Node : m_Config->m_StreetMap->Nodes()) {
        Times[Node.first] = std::numeric_limits<double>::infinity();
    }
    Times[StartNodeID] = 0.0;
    Queue.push(StartNodeID);

    while (!Queue.empty()) {
        TNodeID Current = Queue.top();
        Queue.pop();
        if (Current == EndNodeID) break;

        for (const auto &Neighbor : m_Config->m_StreetMap->Neighbors(Current)) {
            double Distance = SGeographicUtils::HaversineDistanceInMiles(
                m_Config->m_StreetMap->NodePosition(Current), m_Config->m_StreetMap->NodePosition(Neighbor));
            double Speed = m_Config->m_StreetMap->WayMaxSpeed(Current, Neighbor);
            double Alt = Times[Current] + Distance / Speed;
            if (Alt < Times[Neighbor]) {
                Times[Neighbor] = Alt;
                Previous[Neighbor] = Current;
                Queue.push(Neighbor);
            }
        }
    }

    if (Times[EndNodeID] == std::numeric_limits<double>::infinity()) {
        return CPathRouter::NoPathExists;
    }

    for (TNodeID At = EndNodeID; At != StartNodeID; At = Previous[At]) {
        Path.push_back({CTransportationPlanner::ETransportationMode::Walk, At});
    }
    Path.push_back({CTransportationPlanner::ETransportationMode::Walk, StartNodeID});
    std::reverse(Path.begin(), Path.end());
    return Times[EndNodeID];
}

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector<TTripStep> &Path, std::vector<std::string> &Description) {
    Description.clear();
    if (Path.empty()) return false;

    for (const auto &Step : Path) {
        std::string StepDescription;
        switch (Step.Mode) {
            case CTransportationPlanner::ETransportationMode::Walk:
                StepDescription = "Walk";
                break;
            case CTransportationPlanner::ETransportationMode::Bus:
                StepDescription = "Take Bus";
                break;
            case CTransportationPlanner::ETransportationMode::Bike:
                StepDescription = "Bike";
                break;
            default:
                StepDescription = "Unknown mode";
                break;
        }
        Description.push_back(StepDescription);
    }
    return true;
}

std::size_t CDijkstraTransportationPlanner::NodeCount() const {
    return m_Config->m_StreetMap->Nodes().size();
}

std::shared_ptr<CNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t Index) const {
    auto Nodes = m_Config->m_StreetMap->Nodes();
    if (Index >= Nodes.size()) return nullptr;

    std::vector<std::pair<TNodeID, std::shared_ptr<CNode>>> NodeList(Nodes.begin(), Nodes.end());
    std::sort(NodeList.begin(), NodeList.end(), [](const auto &A, const auto &B) {
        return A.first < B.first;
    });
    return NodeList[Index].second;
}