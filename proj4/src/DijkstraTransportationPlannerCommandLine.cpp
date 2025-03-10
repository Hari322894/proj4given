#include "TransportationPlannerConfig.h"

struct CTransportationPlannerCommandLine::SImplementation{

    SImplementation(std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink, std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results, std::shared_ptr<CTransportationPlanner> planner){

    };
    
    bool ProcessCommands(){

    };
};
        
        
CTransportationPlannerCommandLine::CTransportationPlannerCommandLine(std::shared_ptr<CDataSource> cmdsrc, std::shared_ptr<CDataSink> outsink, std::shared_ptr<CDataSink> errsink, std::shared_ptr<CDataFactory> results, std::shared_ptr<CTransportationPlanner> planner){
    DImplementation = std::make_unique<SImplementation>(cmdsrc, outsink, errsink, results, planner);
};

CTransportationPlannerCommandLine::~CTransportationPlannerCommandLine(){

};

bool CTransportationPlannerCommandLine::ProcessCommands(){
    return DImplementation->ProcessCommands();
};

