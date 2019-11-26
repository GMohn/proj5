#include <gtest/gtest.h>
#include "XMLReader.h"
#include <sstream>

TEST(XMLReader, EmptyTest) {
	std::stringstream Input;
	CXMLReader Reader(Input);

	EXPECT_TRUE(Reader.End());
}

TEST(XMLReader, SimpleTest) {
	std::stringstream Input("<tag><other></other></tag>");
	CXMLReader Reader(Input);
	SXMLEntity Entity;

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::StartElement);
	EXPECT_EQ(Entity.DNameData, "tag");

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::StartElement);
	EXPECT_EQ(Entity.DNameData, "other");

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::EndElement);
	EXPECT_EQ(Entity.DNameData, "other");

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::EndElement);
	EXPECT_EQ(Entity.DNameData, "tag");

	EXPECT_TRUE(Reader.End());
}

TEST(XMLReader, NotSoSimpleTest) {
	std::stringstream Input("<CD><TITLE>Eros</TITLE></CD>");
	CXMLReader Reader(Input);
	SXMLEntity Entity;

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::StartElement);
	EXPECT_EQ(Entity.DNameData, "CD");

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::StartElement);
	EXPECT_EQ(Entity.DNameData, "TITLE");

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::CharData);
	EXPECT_EQ(Entity.DNameData, "Eros");
	
	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::EndElement);
	EXPECT_EQ(Entity.DNameData, "TITLE");

	EXPECT_TRUE(Reader.ReadEntity(Entity));
	EXPECT_EQ(Entity.DType, SXMLEntity::EType::EndElement);
	EXPECT_EQ(Entity.DNameData, "CD");

	EXPECT_TRUE(Reader.End());
}