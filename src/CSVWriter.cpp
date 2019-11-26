#include "CSVWriter.h"


CCSVWriter::CCSVWriter(std::ostream& ou) : DOutput(ou) {
}

CCSVWriter::~CCSVWriter() {
}

bool CCSVWriter::WriteRow(const std::vector< std::string >& row){
	char DBuffer[1024];

}
