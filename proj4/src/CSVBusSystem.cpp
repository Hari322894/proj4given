#include "BusSystem.h"
#include "DSVReader.h"
#include "CSVBusSystem.h"
#include <memory>
#include <string>
#include <iostream>
#include <unordered_map>

struct CCSVBusSystem::SImplementation{
    std::shared_ptr< CDSVReader > stop;
    std::shared_ptr< CDSVReader > route;


    struct SStop : public CBusSystem::SStop{
        TStopID stopID;
        CStreetMap::TNodeID stopNodeID;
        SStop(TStopID SID, CStreetMap::TNodeID SNodeID)
        {
            stopID = SID;
            stopNodeID = SNodeID;
        }

        ~SStop(){};

        TStopID ID() const noexcept{
            return stopID;
        }
        CStreetMap::TNodeID NodeID() const noexcept{
            return stopNodeID;
            
        }
    };

    struct SRoute : public CBusSystem::SRoute{
        std::string RouteName;
        std::vector<TStopID> RoutestopID;
        ~SRoute(){};

        SRoute(std::string SName)
        {
            RouteName = SName;
        }

        std::string Name() const noexcept{  
            return RouteName;
        }
        std::size_t StopCount() const noexcept{
            return RoutestopID.size();
        }
        TStopID GetStopID(std::size_t index) const noexcept{
            if (index < RoutestopID.size()){
                return RoutestopID[index];
            }
            return InvalidStopID;
        }
    };
    
    std::vector < std::shared_ptr < SStop > > Allstop;
    std::vector < std::shared_ptr <SRoute> > Allroute;
    
    
    SImplementation(std::shared_ptr< CDSVReader > stopsrc, std::shared_ptr< CDSVReader > routesrc){
        std::vector<std::string> StringVector;
        stopsrc->ReadRow(StringVector);
        routesrc->ReadRow(StringVector);

        while(!stopsrc->End())
        {
            stopsrc->ReadRow(StringVector);
            auto Astop = std::make_shared<SStop>(std::stoull(StringVector[0]), std::stoull(StringVector[1]));
            Allstop.push_back(Astop);
        }
        
        while(!routesrc->End())
        {
            
            int mark = 0;
            routesrc->ReadRow(StringVector);
            for(int i = 0; i < Allroute.size(); i++)
            {
                if(Allroute[i]->RouteName == StringVector[0]){
                    
                    Allroute[i]->RoutestopID.push_back(std::stoull((StringVector[1])));
                    mark = 1;
                }
            }
            if(mark == 0)
            {
                std::vector <TStopID> RoutestopID = {std::stoull(StringVector[1])};
                auto Aroute = std::make_shared<SRoute> (StringVector[0]);
                Aroute->RoutestopID = RoutestopID;
                Allroute.push_back(Aroute);
            }
        }
        
    };

    std::size_t StopCount() const noexcept{
        return Allstop.size();
    }
    std::size_t RouteCount() const noexcept{
        return Allroute.size();
    }
    std::shared_ptr<SStop> StopByIndex(std::size_t index) const noexcept{
        
        if(index >= Allstop.size())
        {
            return nullptr;
        }
        return Allstop[index];

    }

    std::shared_ptr<SStop> StopByID(TStopID id) const noexcept{
        
        for(int i = 0; i < Allstop.size(); i++)
        {
            if (Allstop[i]->ID() == id){
                return Allstop[i];
            }
        }
        return nullptr;
    }
    
    std::shared_ptr<SRoute> RouteByIndex(std::size_t index) const noexcept{
        
        if(index >= Allroute.size())
        {
            return nullptr;
        }
        return Allroute[index];

    }
    std::shared_ptr<SRoute> RouteByName(const std::string &name) const noexcept{

        for(int i = 0; i < Allroute.size(); i++)
        {
            if (Allroute[i]->Name() == name){
                return Allroute[i];
            }
        }
        return nullptr;

    }
};
    
CCSVBusSystem::CCSVBusSystem(std::shared_ptr< CDSVReader > stopsrc, std::shared_ptr< CDSVReader > routesrc){
        DImplementation = std::make_unique<SImplementation>(stopsrc, routesrc);
};

CCSVBusSystem::~CCSVBusSystem(){

};

std::size_t CCSVBusSystem::StopCount() const noexcept{
    return DImplementation->StopCount();
};

std::size_t CCSVBusSystem::RouteCount() const noexcept{
    return DImplementation->RouteCount();
};

std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByIndex(std::size_t index) const noexcept{
    return DImplementation->StopByIndex(index);
};

std::shared_ptr<CBusSystem::SStop> CCSVBusSystem::StopByID(TStopID id) const noexcept{
    return DImplementation->StopByID(id);
};

std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByIndex(std::size_t index) const noexcept{
    return DImplementation->RouteByIndex(index);
};

std::shared_ptr<CBusSystem::SRoute> CCSVBusSystem::RouteByName(const std::string &name) const noexcept{
    return DImplementation->RouteByName(name);
};