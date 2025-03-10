#include "DSVReader.h"
#include "StringUtils.h"
#include <string>
#include <iostream>

 
struct CDSVReader::SImplementation{
    std::shared_ptr< CDataSource > DSVRsource;
    char DSVRdelimiter;
    std::string str;
    char current_char;
    char next_char;
    int ignore;

    SImplementation(std::shared_ptr< CDataSource > src, char delimiter){
    DSVRsource = src;
    DSVRdelimiter = delimiter;
    };

    bool End() const{
    return DSVRsource->End();
    };

    bool ReadRow(std::vector< std::string > &row){
    row.clear();
    if(DSVRsource->End())
    {
        return false;
    }
    ignore = 0;
    while(!DSVRsource->End())
    {
        DSVRsource->Get(current_char);
        if(current_char == DSVRdelimiter && ignore != 1) //if meet the delimiter and the delimiter is not part of the string
        {
            row.push_back(str);
            str = "";
            ignore = 0;
        }
        else if(current_char == '\n') //if meet the newline character, break the loop
        {
            row.push_back(str);
            str = "";
            return true;
        }
        else if(current_char == '\"') //if meet the quotation mark
        {
            DSVRsource->Peek(next_char);//peek to see if it is double quotation
            if(next_char == '\"') //if it is double quotation, peek to see if it is before/after the string
            {
                DSVRsource->Peek(next_char);
                if(next_char == DSVRdelimiter)//if the double quotation mark is after the string
                {
                    DSVRsource->Get(current_char);
                    row.push_back(str);
                    str = "";
                    ignore = 0;
                }
                else//if the double quotation mark is before the string
                {
                    DSVRsource->Get(current_char);
                    str = str + current_char;
                }
            }
            else if (next_char == DSVRdelimiter)//if the delimiter is following the quotaation mark
            {
                DSVRsource->Get(current_char);
                row.push_back(str);
                str = "";
                ignore = 0;
            }
            else
            {
                ignore = 1;
            }
        }
        else//normal cases, add the current char to the string in order to push back it later
        {
            str = str + current_char;
        }
    }
    row.push_back(str);
    return true;

    }

};

        
CDSVReader::CDSVReader(std::shared_ptr< CDataSource > src, char delimiter){
    DImplementation = std::make_unique<SImplementation>(src, delimiter); //init Simplementation
}

bool CDSVReader::End() const{
    return DImplementation->End();
}

CDSVReader::~CDSVReader(){

}

bool CDSVReader::ReadRow(std::vector< std::string > &row){
    return DImplementation->ReadRow(row);
}