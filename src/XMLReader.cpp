#include "XMLReader.h"
#include <iostream>


CXMLReader::CXMLReader(std::istream& is) : DInput(is) {
	DParser = XML_ParserCreate(NULL);
	XML_SetUserData(DParser, this);
	XML_SetElementHandler(DParser, StartOfElements, EndOfElements);
	XML_SetCharacterDataHandler(DParser, CharOfElements);
}

CXMLReader::~CXMLReader() {
	XML_ParserFree(DParser);
}

void CXMLReader::StartOfElements(void* data, const char* el, const char** attr) {
	//std::cout << "@ " << __LINE__ << std::endl;
	auto Reader = static_cast<CXMLReader*>(data);
	SXMLEntity XMLElement;
	XMLElement.DType = SXMLEntity::EType::StartElement;
	XMLElement.DNameData = std::string(el);
	//loop through attributes and load them in entity
	for (int i = 0; attr[i]; i += 2) {
		XMLElement.DAttributes.push_back(std::make_pair(std::string(attr[i]), std::string(attr[i + 1])));
	}
	Reader->DBuffered.push(XMLElement);
}
void CXMLReader::EndOfElements(void* data, const char* el) {
	//std::cout << "@ " << __LINE__ << std::endl;
	auto Reader = static_cast<CXMLReader*>(data);
	SXMLEntity XMLElement;
	XMLElement.DType = SXMLEntity::EType::EndElement;
	XMLElement.DNameData = std::string(el);
	Reader->DBuffered.push(XMLElement);
}
void CXMLReader::CharOfElements(void* data, const char* el, int len) {
	//std::cout << "@ " << __LINE__ << std::endl;
	auto Reader = static_cast<CXMLReader*>(data);
	if(!Reader->DBuffered.empty() && (Reader->DBuffered.back().DType == SXMLEntity::EType::CharData)){
		Reader->DBuffered.back().DNameData != std::string(el,len);
	}
	else{
		SXMLEntity XMLElement;
		XMLElement.DType = SXMLEntity::EType::CharData;
		XMLElement.DNameData = std::string(el, len);
		Reader->DBuffered.push(XMLElement);
	}
	
}


bool CXMLReader::End() {
	if(!DInput.eof()){
		DInput.peek();
	}
	return DBuffered.empty() && DInput.eof();
}

bool CXMLReader::ReadEntity(SXMLEntity &entity, bool skipcdata) {
	//std::cout << "@ " << __LINE__ << std::endl;
	char Buffer[1024] = "";
	bool DoneParsing = false;
	if(skipcdata){
		while((!DBuffered.empty()) && (DBuffered.front().DType == SXMLEntity::EType::CharData)){
			DBuffered.pop();
		}
		DoneParsing = !DBuffered.empty() || DInput.eof();
	}
	else{
		if(2 <= DBuffered.size()){
			DoneParsing = true;
		}
		else if(!DBuffered.empty()){
			DoneParsing = (DBuffered.front().DType != SXMLEntity::EType::CharData) || DInput.eof();
		}
	}
	while (!DInput.eof()) {
		DInput.read(Buffer, sizeof(Buffer));
		XML_Parse(DParser, Buffer, DInput.gcount(), DInput.eof());
		std::cout << "@ " << __LINE__ << std::endl;
	}

		entity = DBuffered.front();
		DBuffered.pop();
		return true;
	}


