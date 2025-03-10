#include "DSVWriter.h"
#include "StringUtils.h"
#include "StringUtils.h"
#include "StringDataSink.h"
#include <string>
#include <iostream>



struct CDSVWriter::SImplementation{
    std::shared_ptr< CDataSink > DSVWsink;
    char DSVWdelimiter;
    char DSVWquoteall;
    int count = 0;


    SImplementation(std::shared_ptr< CDataSink > sink, char delimiter, bool quoteall){
        DSVWsink = sink;
        DSVWdelimiter = delimiter;
        DSVWquoteall = quoteall;

    };

    bool WriteRow(const std::vector<std::string> &row){
        int quoteornot;
        if(count)
        {
            DSVWsink->Put('\n');
        }
        for (int i = 0; i < row.size(); i++)
        {
            quoteornot = 0;   
            for (int j = 0; j < row[i].length(); j++) //determine if it is needed to have double quote for special cases
            {
                if(row[i][j] == ',' || row[i][j] == '\"' )
                {
                    quoteornot = 1;
                    break;
                }
            }
            if(DSVWquoteall) DSVWsink->Put('\"'); //start insert sub-section
            if(quoteornot) DSVWsink->Put('\"');

            for (int j = 0; j < row[i].length(); j++)
            {
                if(row[i][j] == '\"')//check if it need double quote
                {
                    DSVWsink->Put('\"');
                    DSVWsink->Put('\"');
                }
                else
                {
                DSVWsink->Put(row[i][j]);
                }
            }
            if (quoteornot) DSVWsink->Put('\"');
            if (DSVWquoteall) DSVWsink->Put('\"'); //finish insert sub-section of string

            if(i != row.size()-1)//insert delimiter
            {
            DSVWsink->Put(DSVWdelimiter);
            }

            
        }
        count ++;
        return 1;
    };


};


        
CDSVWriter::CDSVWriter(std::shared_ptr< CDataSink > sink, char delimiter, bool quoteall){
    DImplementation = std::make_unique<SImplementation>(sink, delimiter, quoteall);
}


CDSVWriter::~CDSVWriter(){

}

bool CDSVWriter::WriteRow(const std::vector<std::string> &row){
    return DImplementation->WriteRow(row);


}


