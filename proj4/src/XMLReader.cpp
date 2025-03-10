#include "XMLReader.h" // includes the XMLReader class definition
#include <expat.h>     // XML parsing library (Expat)
#include <queue>       // std::queue for storing parsed XML entities
#include <memory>      // for std::shared_ptr and std::unique_ptr
#include <vector>      // for std::vector used to buffer data chunks

// implements the XML Reader using a struct to handle XML parsing
struct CXMLReader::SImplementation {
    // shared pointer to the data source for reading XML input
    std::shared_ptr<CDataSource> DataSource;
    // XML parser (from Expat) to handle parsing
    XML_Parser Parser;
    // queue to store parsed XML entities for processing
    std::queue<SXMLEntity> EntityQueue;
    // indicates the end of the data source
    bool IsEndOfData;
    // buffer to accumulate character data between XML tags
    std::string CharDataBuffer;

    //handler for start element tags
    static void StartElementHandler(void* userData, const char* name, const char** attributes) {
        //cast user data to our implementation structure
        auto* impl = static_cast<SImplementation*>(userData);
        //flush any pending character data before handling the new element
        impl->FlushCharData();

        // create a new entity for the start element
        SXMLEntity entity;
        entity.DType = SXMLEntity::EType::StartElement; //set entity type to StartElement
        entity.DNameData = name; //assign element name

        //process attributes, if any
        if (attributes) {
            for (int i = 0; attributes[i]; i += 2) {
                if (attributes[i + 1]) {
                    entity.DAttributes.emplace_back(attributes[i], attributes[i + 1]); //add attribute name value pair
                }
            }
        }

        //add the entity to the queue
        impl->EntityQueue.push(entity);
    }

    //handler for end element tags
    static void EndElementHandler(void* userData, const char* name) {
        auto* impl = static_cast<SImplementation*>(userData);
        //flush any pending character data before handling the end element
        impl->FlushCharData();

        //create a new entity for the end element
        SXMLEntity entity;
        entity.DType = SXMLEntity::EType::EndElement;
        entity.DNameData = name;

        //add the end element entity to the queue
        impl->EntityQueue.push(entity);
    }

    //handler for character data between XML tags
    static void CharDataHandler(void* userData, const char* data, int length) {
        auto* impl = static_cast<SImplementation*>(userData);
        //append valid character data to the buffer
        if (data && length > 0) {
            impl->CharDataBuffer.append(data, length);
        }
    }

    //constructor to initialize the implementation
    SImplementation(std::shared_ptr<CDataSource> src)
        : DataSource(std::move(src)), IsEndOfData(false) {
        //create the XML parser
        Parser = XML_ParserCreate(nullptr);

        // set up parser handlers
        XML_SetUserData(Parser, this);
        XML_SetElementHandler(Parser, StartElementHandler, EndElementHandler);
        XML_SetCharacterDataHandler(Parser, CharDataHandler);
    }

    // destructor to clean up resources after
    ~SImplementation() {
        XML_ParserFree(Parser); // free the Expat parser
    }

    // flush accumulated character data into the entity queue
    void FlushCharData() {
        if (!CharDataBuffer.empty()) {
            SXMLEntity entity;
            entity.DType = SXMLEntity::EType::CharData; // set entity type to CharData
            entity.DNameData = CharDataBuffer; // assign buffered data

            // add the entity to the queue and clear the buffer
            EntityQueue.push(entity);
            CharDataBuffer.clear();
        }
    }

    // read the next entity from the XML input
    bool ReadEntity(SXMLEntity& entity, bool skipCharData) {
        // read until an entity is available or end of input is reached
        while (EntityQueue.empty() && !IsEndOfData) {
            std::vector<char> buffer(4096); // buffer to hold data
            size_t bytesRead = 0;

            // fill the buffer with data from the data source
            while (bytesRead < buffer.size() && !DataSource->End()) {
                char ch;
                if (DataSource->Get(ch)) {
                    buffer[bytesRead++] = ch;
                } else {
                    break;
                }
            }

            // check if we've reached the end of the data source
            if (bytesRead == 0) {
                IsEndOfData = true;
                XML_Parse(Parser, nullptr, 0, 1); // signal end of parsing
                break;
            }

            // parse the data 
            if (XML_Parse(Parser, buffer.data(), bytesRead, 0) == XML_STATUS_ERROR) {
                return false; // parsing error
            }
        }

        // return the next available entity
        if (!EntityQueue.empty()) {
            entity = EntityQueue.front();
            EntityQueue.pop();

            // skip character data if it is requested
            if (skipCharData && entity.DType == SXMLEntity::EType::CharData) {
                return ReadEntity(entity, skipCharData);
            }

            return true;
        }

        return false; // no more entities to process
    }
};

// constructor for CXMLReader
CXMLReader::CXMLReader(std::shared_ptr<CDataSource> src)
    : DImplementation(std::make_unique<SImplementation>(std::move(src))) {}

// destructor for CXMLReader
CXMLReader::~CXMLReader() = default;

// check if we've reached the end of the XML input
bool CXMLReader::End() const {
    return DImplementation->IsEndOfData && DImplementation->EntityQueue.empty();
}

// read the next entity from the XML input
bool CXMLReader::ReadEntity(SXMLEntity& entity, bool skipCharData) {
    return DImplementation->ReadEntity(entity, skipCharData);
}