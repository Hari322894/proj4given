#include "DijkstraTransportationPlanner.h"

struct CDijkstraTransportationPlanner::SImplementation{

    SImplementation(std::shared_ptr<SConfiguration> config){

    };
    
    std::size_t NodeCount() const noexcept{
        return 0;
    };
    
    std::shared_ptr<CStreetMap::SNode> SortedNodeByIndex(std::size_t index) const noexcept {
        return 0;
    };

    double FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path) {
        return 0;
    };

    double FindFastestPath(TNodeID src, TNodeID dest, std::vector< TTripStep > &path) {
        return 0;
    };

    bool GetPathDescription(const std::vector< TTripStep > &path, std::vector< std::string > &desc) const {
        return 0;
    };
};

CDijkstraTransportationPlanner::CDijkstraTransportationPlanner(std::shared_ptr<SConfiguration> config){
    DImplementation = std::make_unique<SImplementation>(config);
};

CDijkstraTransportationPlanner::~CDijkstraTransportationPlanner(){

};


std::size_t CDijkstraTransportationPlanner::NodeCount() const noexcept{
    return DImplementation->NodeCount();
};

std::shared_ptr<CStreetMap::SNode> CDijkstraTransportationPlanner::SortedNodeByIndex(std::size_t index) const noexcept{
    return DImplementation->SortedNodeByIndex(index);
};


double CDijkstraTransportationPlanner::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path) {
    return DImplementation->FindShortestPath(src,dest,path);
};

double CDijkstraTransportationPlanner::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TTripStep > &path) {
    return DImplementation->FindFastestPath(src,dest,path);
};

bool CDijkstraTransportationPlanner::GetPathDescription(const std::vector< TTripStep > &path, std::vector< std::string > &desc) const {
    return DImplementation->GetPathDescription(path,desc);
};