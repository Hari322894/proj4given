#include "OpenStreetMap.h" // header file for COpenStreetMap class
#include "XMLReader.h" // geader file for XML parsing functionality
#include <memory> // for smart pointers like std::shared_ptr and std::unique_ptr
#include <vector>// for storing nodes and ways in dynamic arrays
#include <string>// for handling string attributes 
#include <unordered_map> //for storing and searching attributes


// defining simplementation structure first using COpenStreetMap
struct COpenStreetMap::SImplementation {
    // Forward declarations of implementation classes
    class SNodeImpl;
    class SWayImpl;
   //storing ways and nodes here
    std::vector<std::shared_ptr<SNodeImpl>> Nodes;
    std::vector<std::shared_ptr<SWayImpl>> Ways;
};

// now we define the implementation classes using CStreetMap::SNode
class COpenStreetMap::SImplementation::SNodeImpl : public CStreetMap::SNode {
public:
 // ID of the node
    TNodeID NodeID;
//coordinates of the node or the loaction
    TLocation NodeLocation;
//key and value attributes using an unordered map
    std::unordered_map<std::string, std::string> Attributes;
//getting nodes ID
    TNodeID ID() const noexcept override {
        return NodeID;
    }
//getting nodes location
    TLocation Location() const noexcept override {
        return NodeLocation;
    }
// # of attributes node has
    std::size_t AttributeCount() const noexcept override {
        return Attributes.size();
    }
//retrieving key of attribute through index
    std::string GetAttributeKey(std::size_t index) const noexcept override {
        if (index < Attributes.size()) {
            auto t = Attributes.begin();
// iterate to the required position 
            for (std::size_t i = 0; i < index; i++) {
                ++t; 
            }
// return the key at the index it's at
            return t->first; 
        }
// return empty string if index is out of bounds
        return ""; 
    }
// checking to see if the node has a attribute using key
    bool HasAttribute(const std::string &key) const noexcept override {
        return Attributes.find(key) != Attributes.end();
    }
   // Retrieve the value of  attribute if the node has an attribute
    std::string GetAttribute(const std::string &key) const noexcept override {
        auto t = Attributes.find(key);
        if (t != Attributes.end()) {
            return t->second;
        }
    //if out of bounds return ""
        return "";
    }
};
// way class, when using CStreetMap::SWay
class COpenStreetMap::SImplementation::SWayImpl : public CStreetMap::SWay {
public:
//ID of way
    TWayID WayID;
//Nodes IDS
    std::vector<TNodeID> NodeIDs;
//attributes for key and value using unordered map
    std::unordered_map<std::string, std::string> Attributes;
//Getting the way's ID
    TWayID ID() const noexcept override {
        return WayID;
    }
//# of nodes in way
    std::size_t NodeCount() const noexcept override {
        return NodeIDs.size();
    }
//Getting Node ID through index
    TNodeID GetNodeID(std::size_t index) const noexcept override {
        if (index < NodeIDs.size()) {
            return NodeIDs[index];
        }
        return CStreetMap::InvalidNodeID;
    }
//# of attributes ways has
    std::size_t AttributeCount() const noexcept override {
        return Attributes.size();
    }
//retrieving key of attribute through index
std::string GetAttributeKey(std::size_t index) const noexcept override {
    if (index < Attributes.size()) {
        auto t = Attributes.begin();
// iterate to the required position 
        for (std::size_t i = 0; i < index; i++) {
            ++t; 
        }
// return the key at the index it's at
        return t->first; 
    }
// return empty string if index is out of bounds
    return ""; 
}
// checking to see if the node has a attribute using key
bool HasAttribute(const std::string &key) const noexcept override {
    return Attributes.find(key) != Attributes.end();
}
// Retrieve the value of  attribute if the node has an attribute
std::string GetAttribute(const std::string &key) const noexcept override {
    auto t = Attributes.find(key);
    if (t != Attributes.end()) {
        return t->second;
    }
//if out of bounds return ""
    return "";
}
};

// Initialize the implementation
COpenStreetMap::COpenStreetMap(std::shared_ptr<CXMLReader> src) {
    //std::make_unique ensures exclusive ownership of the SImplementation instance
    //intializing it to DImplementation
    DImplementation = std::make_unique<SImplementation>();
    
    SXMLEntity entity;
    //std::shared_ptr is used here to allow multiple 
    //parts of the program to share ownership of nodes
    std::shared_ptr<SImplementation::SNodeImpl> currentNode = nullptr;
    // std::shared_ptr for ways allows for multiple references to the same way 
    //without worrying about manual memory management
    std::shared_ptr<SImplementation::SWayImpl> currentWay = nullptr;
   //memory management for both is now a lot easier when using shared_ptr

    
    // parsing the XML file
    while (src->ReadEntity(entity)) {
        if (entity.DType == SXMLEntity::EType::StartElement) {
            if (entity.DNameData == "node") {
                //creating a new node instance
                //Using std::make_shared optimizes memory allocation and ensures exception safety
                currentNode = std::make_shared<SImplementation::SNodeImpl>();
                currentWay = nullptr;
                
                // process the node attributes through a for loop
                for (const auto& attr : entity.DAttributes) {
                //in OSM XML file notes id lat and lon are used as node attributes
                //in the wikipedia link you sent 
                    if (attr.first == "id") {
                        currentNode->NodeID = std::stoull(attr.second);
                    } else if (attr.first == "lat") {
                //std::stod is a simpler way of returning a double while
                //trying to also check for erros such as
                // std::invalid_argument and std::out_of_range
                        currentNode->NodeLocation.first = std::stod(attr.second);
                    } else if (attr.first == "lon") {
                        currentNode->NodeLocation.second = std::stod(attr.second);
                    } else {
                        // store other attributes if needed
                        currentNode->Attributes[attr.first] = attr.second;
                    }
                }
            } else if (entity.DNameData == "way") {
                // creating a new way instance
                currentWay = std::make_shared<SImplementation::SWayImpl>();
                currentNode = nullptr;
                
                // process way attributes
                for (const auto& attr : entity.DAttributes) {
                    if (attr.first == "id") {
                        currentWay->WayID = std::stoull(attr.second);
                    } else {
                        // store other attributes again
                        currentWay->Attributes[attr.first] = attr.second;
                    }
                }
            } else if (entity.DNameData == "nd" && currentWay) {
                // Process node reference inside a way
                //using std::stoull since converting string to unsigned long long
                //when reading attribute
                for (const auto& attr : entity.DAttributes) {
                    if (attr.first == "ref") {
                        currentWay->NodeIDs.push_back(std::stoull(attr.second));
                    }
                }
            } else if (entity.DNameData == "tag") {
                // Process tag element for both nodes and ways
                std::string key, value;
                for (const auto& attr : entity.DAttributes) {
                    if (attr.first == "k") {
                        key = attr.second;
                    } else if (attr.first == "v") {
                        value = attr.second;
                    }
                }
                //pretty self explanatory
                if (!key.empty()) {
                    if (currentNode) {
                        currentNode->Attributes[key] = value;
                    } else if (currentWay) {
                        currentWay->Attributes[key] = value;
                    }
                }
            }
        } else if (entity.DType == SXMLEntity::EType::EndElement) {
            if (entity.DNameData == "node" && currentNode) {
                // Store completed node
                DImplementation->Nodes.push_back(currentNode);
                currentNode = nullptr;
            } else if (entity.DNameData == "way" && currentWay) {
                // Store completed way
                DImplementation->Ways.push_back(currentWay);
                currentWay = nullptr;
            }
        }
    }
}

// destructor
COpenStreetMap::~COpenStreetMap() = default;

// return the total count of nodes
std::size_t COpenStreetMap::NodeCount() const noexcept {
    return DImplementation->Nodes.size();
}

// return the total count of ways
std::size_t COpenStreetMap::WayCount() const noexcept {
    return DImplementation->Ways.size();
}

// retrieve node by index
std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->Nodes.size()) {
        return DImplementation->Nodes[index];
    }
    return nullptr;//if index is out of bounds do this
}

// retrieve node by ID
std::shared_ptr<CStreetMap::SNode> COpenStreetMap::NodeByID(TNodeID id) const noexcept {
    for (auto& node : DImplementation->Nodes) {
        if (node->ID() == id) {
            return node;
        }
    }
    return nullptr;//if no node with matching ID do this
}

// retrieve way by index
std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByIndex(std::size_t index) const noexcept {
    if (index < DImplementation->Ways.size()) {
        return DImplementation->Ways[index];
    }
    return nullptr;//index is out of bounds
}

// retrieve way by ID
std::shared_ptr<CStreetMap::SWay> COpenStreetMap::WayByID(TWayID id) const noexcept {
    for (auto& way : DImplementation->Ways) {
        if (way->ID() == id) {
            return way;
        }
    }
    return nullptr;//if no way with matching ID do this
}
