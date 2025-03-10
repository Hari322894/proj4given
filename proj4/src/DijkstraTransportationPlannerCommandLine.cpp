#include "TransportationPlannerCommandLine.h"
#include "TransportationPlanner.h"
#include "DataSource.h"
#include "DataSink.h"
#include "DataFactory.h"

struct CTransportationPlannerCommandLine::SImplementation {
    std::shared_ptr<CDataSource> CommandSource;
    std::shared_ptr<CDataSink> OutputSink;
    std::shared_ptr<CDataSink> ErrorSink;
    std::shared_ptr<CDataFactory> ResultFactory;
    std::shared_ptr<CTransportationPlanner> Planner;
    
    SImplementation(std::shared_ptr<CDataSource> cmdsrc, 
                   std::shared_ptr<CDataSink> outsink, 
                   std::shared_ptr<CDataSink> errsink, 
                   std::shared_ptr<CDataFactory> results, 
                   std::shared_ptr<CTransportationPlanner> planner) 
        : CommandSource(cmdsrc), OutputSink(outsink), ErrorSink(errsink), 
          ResultFactory(results), Planner(planner) {
    }
   
    bool ProcessCommands() {
        // Your command processing logic here
        // Just a placeholder for now
        return true;
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
    // Destructor is fine as is - no dynamic allocations to clean up
}

bool CTransportationPlannerCommandLine::ProcessCommands() {
    return DImplementation->ProcessCommands();
}