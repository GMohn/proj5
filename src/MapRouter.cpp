#include "MapRouter.h" 	  			 	 
#include <cmath>

const CMapRouter::TNodeID CMapRouter::InvalidNodeID = -1;

CMapRouter::CMapRouter(){

}

CMapRouter::~CMapRouter(){

}

double CMapRouter::HaversineDistance(double lat1, double lon1, double lat2, double lon2){
    auto DegreesToRadians = [](double deg){return M_PI * (deg) / 180.0;};
	double LatRad1 = DegreesToRadians(lat1);
	double LatRad2 = DegreesToRadians(lat2);
	double LonRad1 = DegreesToRadians(lon1);
	double LonRad2 = DegreesToRadians(lon2);
	double DeltaLat = LatRad2 - LatRad1;
	double DeltaLon = LonRad2 - LonRad1;
	double DeltaLatSin = sin(DeltaLat/2);
	double DeltaLonSin = sin(DeltaLon/2);	
	double Computation = asin(sqrt(DeltaLatSin * DeltaLatSin + cos(LatRad1) * cos(LatRad2) * DeltaLonSin * DeltaLonSin));
	const double EarthRadiusMiles = 3959.88;
	
	return 2 * EarthRadiusMiles * Computation;
}        

double CMapRouter::CalculateBearing(double lat1, double lon1,double lat2, double lon2){
    auto DegreesToRadians = [](double deg){return M_PI * (deg) / 180.0;};
    auto RadiansToDegrees = [](double rad){return 180.0 * (rad) / M_PI;};
    double LatRad1 = DegreesToRadians(lat1);
	double LatRad2 = DegreesToRadians(lat2);
	double LonRad1 = DegreesToRadians(lon1);
	double LonRad2 = DegreesToRadians(lon2);
    double X = cos(LatRad2)*sin(LonRad2-LonRad1);
    double Y = cos(LatRad1)*sin(LatRad2)-sin(LatRad1)*cos(LatRad2)*cos(LonRad2-LonRad1);
    return RadiansToDegrees(atan2(X,Y));
}

bool CMapRouter::LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes){
    CXMLReader OSMReader(osm);
    
    while(!OSMReader.End()){
        SXMLEntity Entity;
        if(OSMReader.ReadEntity(Entity)){
            if(Entity.DType == SXMLEntity::EType::StartElement){
                if(Entity.DNameData == "node"){
                    auto ID = std::stoul(Entity.AttributeValue("id"));
                    double Latitude = std::stod(Entity.AttributeValue("lat"));
                    double Longitude = std::stod(Entity.AttributeValue("lon"));                 
                    SNode TempNode;
                    TempNode.DNodeID = ID;
                    //TempNode.Coords = std::pair<double,double>((Latitude),(Longitude));
                    TempNode.DLatitude = Latitude;
                    TempNode.DLongitude = Longitude;
                    DNodeIDToNodeIndex[ID] = DNodes.size();
                    DNodes.push_back(TempNode);
                }
            }
        }
    }
    CCSVReader RouteReader(routes);
    while(!RouteReader.End()){
        
    }
    return true;
    //SXMLEntity xmlEntity;
    //xmlReader.ReadEntity(&xmlEntity);
}

size_t CMapRouter::NodeCount() const{
    return DNodes.size();
}

CMapRouter::TNodeID CMapRouter::GetSortedNodeIDByIndex(size_t index) const{
    //return the nodeID of the corresponding index
    auto counter = DNodes.begin();
    for(auto i = 0;i < index; i++){
        counter++;
    }
    return counter->DNodeID;
}

CMapRouter::TLocation CMapRouter::GetSortedNodeLocationByIndex(size_t index) const{
    //return pair of coordinates x,y of the corresponding index
    auto counter = DNodes.begin();
    //std::pair <double,double> Coords;
    //get second and 3rd elemnt of adjacency list
    //2nd & 3rd need to be pair
    if(index < DNodes.size()){
        for(size_t i = 0;i < index; i++){
            counter++;
        }
        //return result;
        return std::pair<double,double>((counter->DLatitude),(counter->DLongitude));
        
    }
    else{ 
        return std::make_pair(180.0, 360.0);
    }
}

CMapRouter::TLocation CMapRouter::GetNodeLocationByID(TNodeID nodeid) const{
    // Your code HERE
}

CMapRouter::TNodeID CMapRouter::GetNodeIDByStopID(TStopID stopid) const{
    // Your code HERE
}

size_t CMapRouter::RouteCount() const{
    // return DRoute.size();
    
}

std::string CMapRouter::GetSortedRouteNameByIndex(size_t index) const{
    // Your code HERE
}

bool CMapRouter::GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops){
    // Your code HERE
}

double CMapRouter::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    // Your code HERE
}

double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path){
    // Your code HERE
}

bool CMapRouter::GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const{
    // Your code HERE
}
