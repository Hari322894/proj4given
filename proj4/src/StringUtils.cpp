#include "StringUtils.h"
#include <algorithm>
#include <cctype>
#include <vector>

namespace StringUtils {

std::string Slice(const std::string &str, ssize_t start, ssize_t end) noexcept {
    //At the end if the index is 0, it becomes the length of the string
    if (end == 0){
    end = str.length();
    } 
    //if the index is negative to begin with, then we adjust the index
    //to start from the begginning
    if (start < 0)
    {
    start += str.length();
    //same thing here, we want to start at end of string when end index is negative
    } 
    if (end < 0){
    end += str.length();
    } 
    //returning a substring
    return str.substr(start, end - start);
}

std::string Capitalize(const std::string &str) noexcept {
    //return the string if its empty
    if (str.empty()){
    return str;
    } 
    //storing input string here
    std::string result = str;
    //capatilize the first character
    result[0] = std::toupper(result[0]);
    //i created a for loop here based on size of result to make sure everything
    //else is lower case
    for (size_t i = 1; i < result.size(); ++i) {
        result[i] = std::tolower(result[i]);
    }
    //return all characters
    return result;
}

std::string Upper(const std::string &str) noexcept {
    //input string here as result
    std::string result = str;
    //iterate to make all characters uppercase
    for (size_t i = 0; i < result.size(); ++i) {
        result[i] = std::toupper(result[i]);
    }
    //return all characters remaining
    return result;
}

std::string Lower(const std::string &str) noexcept {
    //input string here
    std::string result = str;
    //iterate to make all characters lowercase
    for (size_t i = 0; i < result.size(); ++i) {
        result[i] = std::tolower(result[i]);
    }
    //return all characters remaining
    return result;
}

std::string LStrip(const std::string &str) noexcept {
    size_t start = 0;

    // go through each character from the start
    while (start < str.size()) {
        char currentChar = str[start];

        // if the current character is not a whitespace we stop the code
        if (currentChar != ' ' && currentChar != '\t' && currentChar != '\n' && currentChar != '\r') {
            break;
        }

        //move to next character if it is a whitespace
        start++;
    }

    // retuurn everything from first non white space and forward
    return str.substr(start);
}

std::string RStrip(const std::string &str) noexcept {
    size_t end = str.size();

    // go through each character from the end
    while (end > 0) {
        char currentChar = str[end - 1];

        //if the current character is not a whitespace we stop the code
        if (currentChar != ' ' && currentChar != '\t' && currentChar != '\n' && currentChar != '\r') {
            break;
        }

        // move a position backward
        end--;
    }

    // return everything to the last non whitespace character
    return str.substr(0, end);
}


std::string Strip(const std::string &str) noexcept {
    // first, i am removing trailing whitespace using RStrip
    std::string rstripResult = RStrip(str);

    // then i am removing leading whitespace from the result using LStrip
    return LStrip(rstripResult);

    //return Lstrip(Rstrip(str)) works here but im trying to see my thoughts worked out
}


std::string Center(const std::string &str, int width, char fill) noexcept {
    // calculating total padding needed
    int padding = width - static_cast<int>(str.size());  
     // if no padding is needed, return the original string
     if (padding <= 0){
     return str; 
     } 
    // calculate padding for left and right side
    int left_pad = padding / 2;  
    int right_pad = padding - left_pad;  

    // return the centered string with fill characters on both sides
    return std::string(left_pad, fill) + str + std::string(right_pad, fill);
}


std::string LJust(const std::string &str, int width, char fill) noexcept {
    // calculate padding needed
    //error if not using static_cast<int> cause of mismatch type
    int padding = width - static_cast<int>(str.size()); 
    // if no padding is needed, return the original string 
    if (padding <= 0) {
        return str;  
    }
    // return the left string with fill characters on the right
    return str + std::string(padding, fill);
}


std::string RJust(const std::string &str, int width, char fill) noexcept {
    // calculate padding needed
    int padding = width - static_cast<int>(str.size());  
    // if no padding is needed, return the original string
    if (padding <= 0) {
        return str;  
    }
    // return the right string with fill characters on the left
    return std::string(padding, fill) + str;
}



std::string Replace(const std::string &str, const std::string &old, const std::string &rep) noexcept {
    // if old is an empty string js return the original string
    if (old.empty()) {
        return str;
    }
    //input string here as result and start searching
    // for string at begginning of the string
    std::string result = str;  
    size_t pos = 0;  

    // while loop here to find and replace all occurrences of old with rep
    while ((pos = result.find(old, pos)) != std::string::npos) {
        // Replace old with rep
        result.replace(pos, old.length(), rep); 
        // Mmve the search position forward in the string 
        pos += rep.length();  
    }
//return the string result
    return result;  
}



std::vector<std::string> Split(const std::string &str, const std::string &splt) noexcept {
    // storing the resulting substrings
    std::vector<std::string> result;  

    // if the input string is empty, return an empty vector
    if (str.empty()) {
        return result;
    }

    // split the string by whitespace
    if (splt.empty()) {
        //start and end position
        size_t start = 0;  
        size_t end = 0;    

        //iterate through the string to find whitespace
        while (end < str.size()) {
            // skip leading whitespace
            while (start < str.size() && std::isspace(str[start])) {
                start++;
            }

            // if we've reached the end of the string, lets break
            if (start >= str.size()) {
                break;
            }

            // Find the end of the current
            end = start;
            while (end < str.size() && !std::isspace(str[end])) {
                end++;
            }

            // extract and add to the result vector
            result.push_back(str.substr(start, end - start));

            // move the start position to the end of the current
            start = end;
        }
         // return the vector
        return result;  
    }

    // start position of the current substring
    size_t start = 0;  
   // find the first occurrence 
    size_t end = str.find(splt);  

    // loop to split the string 
    while (end != std::string::npos) {
 // add the substring to the result
        result.push_back(str.substr(start, end - start)); 
// move the start position 
        start = end + splt.length();  
// find the next occurrence
        end = str.find(splt, start);  
    }

    // add the last substring 
    result.push_back(str.substr(start));
 // return the vector of substrings
  return result; 
}
   


std::string Join(const std::string &str, const std::vector<std::string> &vect) noexcept {
    // if the vector is empty, return an empty string
    if (vect.empty()) {
        return "";
    }
 // start with the first string in the vector
    std::string result = vect[0]; 

    // loop to append the remaining strings with the separator
    for (size_t i = 1; i < vect.size(); ++i) {
        result += str + vect[i];  // add the separator and the next string
    }
 // return the joined string
    return result; 
}
  
// Function to expand tabs in a string to spaces based on a given tab size
std::string ExpandTabs(const std::string &str, int tabsize) noexcept {
    // resulting string with expanded tabs
    std::string result;  
    // current column position in the string
    size_t column = 0;   

    // if tab size is 0, remove all tabs from the string
    if (tabsize == 0) {
        for (char c : str) {
            if (c != '\t') {
            // append non-tab characters to the result
                result += c;  
            }
        }
        // return the string with tabs removed
        return result;  
    }

    // process the input string and expand tabs
    for (char c : str) {
        if (c == '\t') {
            // calculate the number of spaces needed to reach the next tab 
            size_t spaces = tabsize - (column % tabsize);
 // append the spaces to the result
            result.append(spaces, ' '); 
// update the column position
            column += spaces;  
        } else {
            // append non-tab characters to the result
            result += c;  
 // increment the column position
            column++;    
        }
    }
// return the string with expanded tabs
    return result;  
}



   int EditDistance(const std::string &left, const std::string &right, bool ignorecase) noexcept {
    //Im using dynamic programming from my ecs 122a class i took over summer cause time complexity
    //O(m*n) complexity
    // and coding it is much easier this way

    std::string l = left;
    std::string r = right;
// convert to lowercase if ignorecase is true
    if (ignorecase) {
        l = Lower(left);
        r = Lower(right);
    }
//lengths of both strings
    size_t len1 = l.size();
    size_t len2 = r.size();
//dp table b/w substrings
    std::vector<std::vector<int>> dp(len1 + 1, std::vector<int>(len2 + 1));
//first column of dp table
    for (size_t i = 0; i <= len1; ++i) {
        dp[i][0] = i;
//first row of dp table
    }
    for (size_t j = 0; j <= len2; ++j) {
        dp[0][j] = j;
    }
//making the rest of the dp table
    for (size_t i = 1; i <= len1; ++i) {
        for (size_t j = 1; j <= len2; ++j) {
//determining cost
            int cost = 1;
        if(l[i-1] == r[j-1]){
            cost = 0;
        }
//min cost of deletion,insertion and subsitution
            int deletion = dp[i - 1][j] + 1;
            int insertion = dp[i][j - 1] + 1;
            int substitution = dp[i - 1][j - 1] + cost;
//store min cost with std::min
            dp[i][j] = std::min({deletion, insertion, substitution});
        }
    }
//return distance
    return dp[len1][len2];
}

}