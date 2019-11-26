#include "StringUtils.h" 	  			 	 
#include <algorithm> 
#include <cctype>
#include <cstdio>
#include <iostream> 

namespace StringUtils{
    
std::string Slice(const std::string &str, ssize_t start, ssize_t end){ //https://stackoverflow.com/questions/27992264/c-equivalent-of-python-string-slice
    // Your code goes here
    std::string SlicedString = str;
    //std::cout << "String length: "<< SlicedString.length()<< std::endl;
    /*if (start > SlicedString.length()){ //https://stackoverflow.com/questions/3660901/a-warning-comparison-between-signed-and-unsigned-integer-expressions
        std::cout << "Start val: "<< start<< std::endl;
        return "";*/
    
    if (start < 0){
        start += SlicedString.length();
    }
    if (end <= 0){
        end += SlicedString.length();
    }
    int NewEnd = end - start;
   // std::cout << " new end: " << NewEnd<< std::endl;
    if (NewEnd < 0){
        return "";
    }
    SlicedString.substr(start, NewEnd);
    //std::cout << "Start: " << start << " End: " << NewEnd << std::endl;
    //std::cout << "Sliced String: "<< SlicedString << std::endl;

    return SlicedString.substr(start, NewEnd);
}

std::string Capitalize(const std::string &str){
    // Your code goes here
    std::string CapitalString;
    for (char c: str){ //https://www.techiedelight.com/convert-string-lowercase-cpp/
        CapitalString += tolower(c);
        
    }
    CapitalString[0] = toupper(CapitalString[0]);
    //std::cout <<"String: "<< CapitalString;
    return CapitalString;
}

std::string Title(const std::string &str){
    // Your code goes here
    /* String is "__HELLO123world!"
    Expected output "__Hello123World!"


    */
    std::string TitleString;
    int PrevIdx = 0, CurrIdx = 0;
    for (char c: str){
    	TitleString += tolower(c);
    	//std::cout << "Print current idx: " <<CurrIdx;
    	if(isalpha(TitleString[0])){
    		TitleString[0] = toupper(TitleString[0]);
    	}
    	if (isalpha(TitleString[CurrIdx]) == 0){ //if not alphabetic
    		PrevIdx = CurrIdx;
    		CurrIdx++;
    	}
    	else{
    		if (isalpha(TitleString[PrevIdx])){
    			//previous char is alphabetic no need to capitalize
    			PrevIdx = CurrIdx;
    			CurrIdx++;
    		}

    		else{ //curridx of str is alpha and prev idx is not alpha
    			TitleString[CurrIdx] = toupper(TitleString[CurrIdx]);
    			PrevIdx = CurrIdx;
    			CurrIdx++;


    		}

    	}
    }
    	return TitleString;
}

std::string LStrip(const std::string &str){
    // Your code goes here
    /* "    Test String"


    */
    std::string EmptyString = "";
    std::string LeftTrim = str;
    int Idx = 0;
    for (std::size_t x = 0, length = str.length(); x < length; ++x){ //https://stackoverflow.com/questions/5135291/is-using-string-length-in-loop-efficient
    	if (LeftTrim[Idx] != ' '){
    		break;
    	}
    	else{
    		Idx ++;
    	}
    }
    LeftTrim = LeftTrim.substr(Idx,LeftTrim.length());
    return LeftTrim;
}

std::string RStrip(const std::string &str){
    // Your code goes here
    std::string RightTrim = str;
    int Idx = 0;
    for (int i = str.size() - 1; i >= 0; i--){ //www.techiedelight.com/loop-characters-string-backwards-cpp/
    	if (RightTrim[Idx] != ' '){
    		break;
    	}
    	else{
    		Idx ++;
    	}
    }
    RightTrim = RightTrim.substr(0,str.size()-Idx);
    return RightTrim;
}

std::string Strip(const std::string &str){
    // Your code goes here
    std::string StrippedString = str;
    StrippedString = LStrip(str);
    //std::cout << "LStrip String: " << StrippedString;
    StrippedString = RStrip(StrippedString);
    //std::cout << "Both Stripped String: " << StrippedString;
    return StrippedString;
}

std::string Center(const std::string &str, int width, char fill){
    // Your code goes here
    std::string CenteredString = str;
    int Padding = width - str.length();
    int LeftPadding = Padding/2;
    int RightPadding = Padding - LeftPadding;
    if (Padding <= 0){
    	return str;
    }
    else{
    	CenteredString = std::string(LeftPadding, fill) + str; //stackoverflow.com/questions/166630/how-to-repeat-a-string-a-variable-number-of-times-in-c
    	CenteredString = CenteredString + std::string(RightPadding, fill);
    	}
  
    return CenteredString;
}

std::string LJust(const std::string &str, int width, char fill){
    // Your code goes here
    std::string LeftJust = str;
    int Padding = width - str.length();
    LeftJust = str + std::string(Padding, fill);
    return LeftJust;
}

std::string RJust(const std::string &str, int width, char fill){
    // Your code goes here
    std::string RightJust = str;
    int Padding = width - str.length();
    RightJust = std::string(Padding, fill) + str;
    return RightJust;
}

std::string Replace(const std::string &str, const std::string &old, const std::string &rep){
    // Your code goes here
    /*"aabbccaaabbbcccaaaa","aa","ee"), "eebbcceeabbbccceeee"
    push all 
    */
    std::string ReplacedString = str;
    size_t Idx = 0;
    size_t RepLength = rep.length();
    while (true){ //stackoverflow.com/questions/4643512/replace-substring-with-another-substring-c
    	Idx = str.find(old, Idx);
    	if (Idx == std::string::npos) break;
    	ReplacedString.replace(Idx,RepLength, rep);
    	Idx += RepLength;
/*
FIGURE OUT HOW THIS FUNCTION WORKS
*/
    }

    return ReplacedString;
}

std::vector< std::string > Split(const std::string &str, const std::string &splt){
    // Your code goes here
	std::vector<std::string> SplitVec;
	std::string FoundString;
    std::string TempString = str;
	size_t Idx = 0;
	size_t PrevFound = 0;
    if (splt == ""){
        while (true){           
            if (Idx >= str.length()){
                break;
            }
            else{
                while (Idx < str.length() && !isspace(str[Idx])){ //when its not white space 
                    FoundString += str[Idx];
                    //std::cout << "Foundstring in splt: "<< FoundString<< std::endl;
                    Idx++;
                }
            }
            if (FoundString != ""){
                SplitVec.push_back(FoundString);
                FoundString = "";         
                //std::cout << "Idx value in outside while loop: "<< Idx << std::endl;
            }
            Idx++;

        }
        return SplitVec;
    }
    else{
        while (true){
            size_t Found = str.find(splt,PrevFound);
            //std::cout <<"found: " <<Found << std::endl;
            if(Found == std::string::npos){
                break;
            }
            if(PrevFound == Found){
                SplitVec.push_back("");
                PrevFound = Found + splt.length();
            }
            else{
                FoundString = str.substr(PrevFound, Found-PrevFound);
                //std::cout <<"FoundString: "<< FoundString <<" Prevfound: "<< PrevFound << std::endl;
                SplitVec.push_back(FoundString);
                PrevFound = Found + splt.length();
            } 
        }
    }

	
	SplitVec.push_back(Slice(str,PrevFound,str.length()));
  	

/*
	std::string SplitString = str, TempStr;
	size_t Idx = 0;
	size_t SpltLen = splt.length();
	for (std::size_t i = 0; i < str.length();i++){ //iterate through the string length
		Idx = str.find(splt, Idx); //find the idex where the first occurence of splt occurs
		if (Idx != std::string::npos){ //stackoverflow.com/questions/2340281/check-if-a-string-contains-a-string-in-c
			
			TempStr = SplitString.substr(Idx+1);
			std::cout <<"Idx: "<< Idx <<" TempStr: "<<TempStr << std::endl;
		}
		else{
			break;
		}
		Idx += SpltLen;
		
	}
*/	
    return SplitVec;
}

std::string Join(const std::string &str, const std::vector< std::string > &vect){
    // Your code goes here
    std::string JoinString;
    for (size_t i = 0; i < vect.size();i++){
        //std::cout <<"i: " << i <<std::endl;
        //std::cout <<"vec size: "<< vect.size()<< std::endl;
        JoinString += vect[i];
        if (i == vect.size() - 1){
            break;
        }
        else{
            JoinString += str;
        }

        //std::cout << "JoinString: " <<JoinString << std::endl;
    }
    return JoinString;
}

std::string ExpandTabs(const std::string &str, int tabsize){
    // Your code goes here
    /* "H/tELLO"


    */
    std::string ExpandedString;
    int TabLength;
    /*if (tabsize == 0){
    	ExpandedString = Replace(str,"\t");
    	return ExpandedString;
    }*/

    for (size_t i = 0; i < str.length(); i++){
    	if (str.at(i) != '\t'){
    		ExpandedString += str.at(i);
    	}
    	else{
    		if(tabsize == 0){
    			continue;
    		}
    		TabLength = tabsize - (ExpandedString.length() % tabsize);
    		for (int j = 0; j < TabLength; j++){
    			ExpandedString += " ";
    		}

    	}
    }

    return ExpandedString;
}

int EditDistance(const std::string &left, const std::string &right, bool ignorecase){
    // Your code goes here
    const std::size_t LeftLen = left.length();
    const std::size_t RightLen = right.length();
    std::string LeftString;
    std::string RightString;
    if (ignorecase){
        for (size_t i = 0; i < LeftLen; i++){
            LeftString += tolower(left[i]);       
        }
        for (size_t j = 0; j < RightLen; j++){
            RightString += tolower(right[j]);
        }
    }
    else{
        LeftString = left;
        RightString = right;
    }
    int MinEditDist[LeftLen+1][RightLen+1];
    /* When i is 0 so left or right is empty then dist is length of the other
    cost is i when 0 right chars1
    */
    for (size_t i = 0; i <= LeftLen; i++){
        MinEditDist[i][0] = i;
    }
    for(size_t j = 0; j <= RightLen; j++){
        MinEditDist[0][j] = j;
    }

    for (size_t i = 1; i <= LeftLen; i++){
        for(size_t j = 1; j <= RightLen; j++){
            if (LeftString[i-1] == RightString[j-1]){
                MinEditDist[i][j] = MinEditDist[i-1][j-1]; //diagonal of the table. characters are the same, so use prev result
            }
            else{ //characters are not the same
                int InsertCost = MinEditDist[i][j-1];
                int DeleteCost = MinEditDist[i-1][j];
                int ReplaceCost = MinEditDist[i-1][j-1];
                int MinCost = std::min(std::min(InsertCost,DeleteCost),ReplaceCost);
                MinEditDist[i][j] = MinCost + 1; 
            }
        }
    }


    return MinEditDist[LeftLen][RightLen];
}

}