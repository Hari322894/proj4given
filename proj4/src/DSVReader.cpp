#include "DSVReader.h" // including header file for CDSVReader class usage

// implementing details of DSV Reader into struct function
struct CDSVReader::SImplementation {
    // shared pointer to datasource in order for reading
    std::shared_ptr<CDataSource> DataSource;
    // char variable used to separate values in the file
    char Delimiter;

    // initialize my source and delimiter before moving on any further
    SImplementation(std::shared_ptr<CDataSource> src, char delimiter)
        : DataSource(std::move(src)), Delimiter(delimiter) {}

    // reading the row which is most likely a vector of strings
    bool ReadRow(std::vector<std::string>& currentRow) {
        // begin with an empty row
        currentRow.clear();
        
        // a string to get data for each cell
        std::string currentCell;
        // a variable to store the character read from the data source
        char currentChar;
        // determines if we are inside a quoted string
        bool isInQuotes = false;
        // to track whether any data has been read
        bool data = false;
    
        // read characters until reaching the end of the data source
        while (!DataSource->End()) {
            // if unable to read a charcter return false
            if (!DataSource->Get(currentChar)) {
                return false;
            }
    
            // if a character was successfully read then we've encountered data
            data = true;
    
            // having quotes for the current characters
            if (currentChar == '"') {
                // check for double quotes in a row
                if (!DataSource->End()) {
                    char nextChar;
                    // attempt to peek at the next character from the DataSource.
                    bool peekResult = DataSource->Peek(nextChar);
                    if (peekResult && nextChar == '"') {
                        // if the next character is another quote treat it as an escaped quote
                        DataSource->Get(nextChar); // Takes the second quote
                        currentCell += '"'; // add a single quote to the current cell
                    } else if (isInQuotes) {
                        // we are inside quotes and find another quote, it’s the end of the quoted section
                        isInQuotes = false;
                    } else {
                        // we are starting a new quoted section
                        isInQuotes = true;
                    }
                } else if (isInQuotes) {
                    // if we are at the end of the file and inside quotes, close quote section
                    isInQuotes = false;
                } else {
                    // if at the end of the file and not inside quotes, begin new quote section
                    isInQuotes = true;
                }
            }
            // if we hit a delimiter and we're not inside quotes, it marks the end of the current cell
            else if (currentChar == Delimiter && !isInQuotes) {
                currentRow.push_back(currentCell); // add the completed cell to the row
                currentCell.clear(); // prepare for the next cell by clearing the cell
            }
            // end of the row detected (\n or \r return), unless inside quotes
            else if ((currentChar == '\n' || currentChar == '\r') && !isInQuotes) {
                if (!currentCell.empty() || !currentRow.empty()) {
                    currentRow.push_back(currentCell); // Add any remaining data in the cell with this line
                }
    
                // \r\n handling
                if (currentChar == '\r' && !DataSource->End()) {
                    char nextChar;
                    bool peekResult = DataSource->Peek(nextChar);
                    if (peekResult && nextChar == '\n') {
                        // if the next character is '\n', take it in to avoid treating it as part of the next row
                        DataSource->Get(nextChar);
                    }
                }
    
                return true; // successfully read the row and returns true
            }
            // adding regular character to the current cell
            else {
                currentCell += currentChar;
            }
        }
    
        // any remaining data in the current cell push it into the row
        if (!currentCell.empty() || data) {
            currentRow.push_back(currentCell);
        }
    
        // return true if any content was read
        return data;
    }
};

// constructor for DSV Reader class
CDSVReader::CDSVReader(std::shared_ptr<CDataSource> src, char delimiter)
    : DImplementation(std::make_unique<SImplementation>(src, delimiter)) {}

// destructor for DSV Reader class
CDSVReader::~CDSVReader() = default;

// check if we've reached the end of data source
bool CDSVReader::End() const {
    return DImplementation->DataSource->End();
}

// read a row of data from the source
bool CDSVReader::ReadRow(std::vector<std::string> &row) {
    return DImplementation->ReadRow(row);
}