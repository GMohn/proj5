#include "CSVReader.h"

CCSVReader::CCSVReader(std::istream &in) : DInput(in){
	csv_init(&DParser, 0);
}

CCSVReader::~CCSVReader(){
	csv_free(&DParser);
}
void CCSVReader::EndOfColumn(void* str, size_t len, void *reader) {
	auto Reader = static_cast<CCSVReader *>(reader);
	auto Column = std::string(static_cast<const char*>(str), len);
	Reader->DCurrentRow.push_back(Column);
}

void CCSVReader::EndOfRow(int ch, void* reader) {
	auto Reader = static_cast<CCSVReader*>(reader);
	Reader->DBufferedRows.push_back(Reader->DCurrentRow);
	Reader->DCurrentRow.clear();
}
bool CCSVReader::End() const{
	if (!DInput.eof()) {
		DInput.peek();
	}
	return DInput.eof() && DBufferedRows.empty();
}

bool CCSVReader::ReadRow(std::vector< std::string > &row){
	while (DBufferedRows.empty()) {
		char Buffer[1024];
		DInput.read(Buffer, sizeof(Buffer));
		csv_parse(&DParser, Buffer, DInput.gcount(), EndOfColumn, EndOfRow, this);
		if (DInput.eof()) {
			csv_fini(&DParser, EndOfColumn, EndOfRow, this);
			break;
		}
	}
	if (DBufferedRows.empty()) {
		return false;
	}
	row = DBufferedRows.front();
	DBufferedRows.pop_front();
	return true;
}