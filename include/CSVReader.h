#ifndef CSVREADER_H 	  			 	 
#define CSVREADER_H

#include <istream>
#include <string>
#include <vector>
#include <list>
#include <csv.h>

class CCSVReader{
    protected:
		std::istream& DInput;
		struct csv_parser DParser;
		std::list<std::vector<std::string>> DBufferedRows;
		std::vector<std::string> DCurrentRow;
		static void EndOfColumn(void* str, size_t len, void* reader);
		static void EndOfRow(int ch, void* reader);
        
    public:
        CCSVReader(std::istream &in);
        ~CCSVReader();
        
        bool End() const;
        bool ReadRow(std::vector< std::string > &row);
};

#endif
