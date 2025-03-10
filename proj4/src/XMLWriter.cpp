#include "XMLWriter.h"
#include "StringUtils.h"
#include <expat.h>
#include <deque>
#include <iostream>

struct CXMLWriter::SImplementation{
    std:: shared_ptr< CDataSink > DSink;
    //parser?

    std::vector< std::string > ICElements;
    void SpecialCharReplace(std::string &str){
        
        str = StringUtils::Replace(str,"&","&amp;");
        str = StringUtils::Replace(str,"\"","&quot;");
        str = StringUtils::Replace(str,"'","&apos;");
        str = StringUtils::Replace(str,"<","&lt;");
        str = StringUtils::Replace(str,">","&gt;");
    };
    SImplementation(std::shared_ptr< CDataSink > sink){
        DSink = sink;
    };
    ~SImplementation(){

    }

    bool Flush(){//FIX LATER
        while(!ICElements.empty()){
            std::vector<char> Buffer;
            std::string NameData;
            //std::cout<<"made into while loop"<<std::endl;
            //std::cout<<"ICELEMENT size:"<<ICElements.size()<<std::endl;
            for(int i = ICElements.size()-1; i>=0;i-- ){
                Buffer.clear();
                //std::cout<<"ICENAME"<<ICElements[i]<<std::endl;
                NameData = ICElements[i];
                Buffer.push_back('<');
                Buffer.push_back('/');
                for(int i = 0; i< NameData.length();i++){
                    Buffer.push_back(NameData[i]);
                }
            
                Buffer.push_back('>');
               // std::cout<<"SHOUDL APPEAR 3 times"<<std::endl;
                
                
                ICElements.pop_back();
                
                if(ICElements.size() > 0){
                    Buffer.push_back('\n');
                }
                //std::cout<<"ABOUT TO WRITE YAYY"<<Bufferstd::endl;
                DSink->Write(Buffer);
            }

        }
        if(ICElements.empty()){
            return true;
        }
    };


    bool WriteEntity(const SXMLEntity &entity){
        std::vector<char> Buffer;
        std::string stringpiece;
        //std::vector< std::string > IncompletElements;
        if(entity.DType == SXMLEntity::EType::StartElement ||entity.DType == SXMLEntity::EType::CompleteElement){
           // std::cout<<"MADE IT INTO LOOP"<<std::endl;
            if(entity.DType == SXMLEntity::EType::StartElement){
                ICElements.push_back(entity.DNameData);
            }
            //std::cout<<"dnamedata is:"<<stringpiece<<std::endl;
            stringpiece  = entity.DNameData;
           // std::cout<<"dnamedata is:"<<stringpiece<<std::endl;
            Buffer.push_back('<');
            for(int i = 0; i<stringpiece.length();i++){
                Buffer.push_back(stringpiece[i]);
                //std::cout<<"buffer insterted"<<std::endl;
            }
            //insert attributes
            for(auto &Attr : entity.DAttributes){
                std::string AttKey = std::get<0>(Attr);
                std::string AttValue = std::get<1>(Attr);
                SpecialCharReplace(AttKey);
                SpecialCharReplace(AttValue);
                //std::cout<<"REPLACEDCHAR"<<AttValue<<std::endl;
                Buffer.push_back(' ');
                for(int i = 0; i<AttKey.length();i++){

                    Buffer.push_back(AttKey[i]);
                }
                Buffer.push_back('=');
                Buffer.push_back('\"');
                for(int i = 0; i<AttValue.length();i++){
                    Buffer.push_back(AttValue[i]);
                }
                Buffer.push_back('\"');
            }
            if(entity.DType == SXMLEntity::EType::CompleteElement){
                Buffer.push_back('/');
            }
            Buffer.push_back('>');
            
            DSink->Write(Buffer);
            return true;
            

        }
        if(entity.DType == SXMLEntity::EType::EndElement){
            stringpiece  = entity.DNameData;
            Buffer.push_back('<');
            Buffer.push_back('/');
            for(int i = 0; i<stringpiece.length();i++){
                Buffer.push_back(stringpiece[i]);
            }
            Buffer.push_back('>');
            ICElements.pop_back();
            DSink->Write(Buffer);
            return true;

        }
        
        if(entity.DType == SXMLEntity::EType::CharData){
            stringpiece = entity.DNameData;
            SpecialCharReplace(stringpiece);
            for(int i = 0; i<stringpiece.length();i++){
                Buffer.push_back(stringpiece[i]);
            }
            DSink->Write(Buffer);
            return true;

        }

    };

};

CXMLWriter :: CXMLWriter(std::shared_ptr< CDataSink > sink){
    DImplementation = std::make_unique<SImplementation>(sink);
}

CXMLWriter:: ~CXMLWriter(){

}

bool CXMLWriter::Flush(){
    return DImplementation->Flush();
}

bool CXMLWriter::WriteEntity(const SXMLEntity &entity){
    return DImplementation->WriteEntity(entity);
}