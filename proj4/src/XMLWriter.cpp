#include "XMLWriter.h"  //header for the XMLWriter class.
#include <vector>       //used for managing the element stack as a vector.
#include <string>       //provides the std::string type for handling XML strings.

struct CXMLWriter::SImplementation {
    std::shared_ptr<CDataSink> DDataSink;   //data sink used for writing output.
    std::vector<std::string> DElementList;  //stores the stack of open elements.

    //constructor initializes the data sink.
    SImplementation(std::shared_ptr<CDataSink> sink)
        : DDataSink(sink) {}

    //writes a plain string to the data sink 
    //returns false if writing fails
    bool OutputString(const std::string& str) {
        for (char ch : str) {
            if (!DDataSink->Put(ch)) {
                return false;
            }
        }
        return true;
    }

    //writes an escaped version of the string (e.g., for special XML characters).
    //used switch and case
    bool StringEscaped(const std::string& str) {
        for (char ch : str) {
            switch (ch) {
                case '<':
                    if (!OutputString("&lt;")) {
                        return false;
                    }
                    break;
                case '>':
                    if (!OutputString("&gt;")){
                         return false;
                    }
                        break;
                case '&':
                    if (!OutputString("&amp;")){ 
                        return false;
                    }
                        break;
                case '\'':
                    if (!OutputString("&apos;")) {
                    return false;
                    }
                    break;
                case '"':
                    if (!OutputString("&quot;")){
                         return false;
                    }
                    break;
                default:
                    if (!DDataSink->Put(ch)){ 
                        return false;
                    }
            }
        }
        return true;
    }

    //closes all remaining open tags
    //returns false if writing fails
    bool FinalizeOutput() {
        for (auto it = DElementList.rbegin(); it != DElementList.rend(); ++it) {
            if (!OutputString("</") ||
                !OutputString(*it) ||
                !OutputString(">")) {
                return false;
            }
        }
        DElementList.clear();  // clear the element list once all tags are closed.
        return true;
    }

    // writes the provided XML entity to the output.
    bool OutputEntity(const SXMLEntity& entity) {
        switch (entity.DType) {
            case SXMLEntity::EType::StartElement:
                // write the opening tag for the element.
                if (!OutputString("<") ||
                    !OutputString(entity.DNameData)) {
                    return false;
                }
                // write all attributes for the element.
                for (const auto& attr : entity.DAttributes) {
                    if (!OutputString(" ") ||
                        !OutputString(attr.first) ||
                        !OutputString("=\"") ||
                        !StringEscaped(attr.second) ||
                        !OutputString("\"")) {
                        return false;
                    }
                }
                if (!OutputString(">")) {
                    return false;
                }
                DElementList.push_back(entity.DNameData);  // add element to the stack.
                break;

            case SXMLEntity::EType::EndElement:
                // write the closing tag for the element.
                if (!OutputString("</") ||
                    !OutputString(entity.DNameData) ||
                    !OutputString(">")) {
                    return false;
                }
                if (!DElementList.empty()) {
                    DElementList.pop_back();  // remove the element from the stack.
                }
                break;

            case SXMLEntity::EType::CharData:
                // write character data, escaping special characters.
                if (!StringEscaped(entity.DNameData)) {
                    return false;
                }
                break;

            case SXMLEntity::EType::CompleteElement:
                // write a self-closing tag for the element.
                if (!OutputString("<") ||
                    !OutputString(entity.DNameData)) {
                    return false;
                }
                // write all attributes for the element.
                for (const auto& attr : entity.DAttributes) {
                    if (!OutputString(" ") ||
                        !OutputString(attr.first) ||
                        !OutputString("=\"") ||
                        !StringEscaped(attr.second) ||
                        !OutputString("\"")) {
                        return false;
                    }
                }
                if (!OutputString("/>")) {
                    return false;
                }
                break;
        }
        return true;
    }
};

// constructor initializes the XMLWriter 
CXMLWriter::CXMLWriter(std::shared_ptr<CDataSink> sink)
    : DImplementation(std::make_unique<SImplementation>(sink)) {
}

// destructor 
CXMLWriter::~CXMLWriter() = default;

// flushes all remaining open elements 
bool CXMLWriter::Flush() {
    return DImplementation->FinalizeOutput();
}

// writes an XML entity to the output
bool CXMLWriter::WriteEntity(const SXMLEntity& entity) {
    return DImplementation->OutputEntity(entity);
}