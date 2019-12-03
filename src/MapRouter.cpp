#include "MapRouter.h" 	  			 	 
#include <cmath>
#include <sstream>
#include <iostream>
#include<bits/stdc++.h> 
#include<set>

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
    const double WalkingSpeed = 3.0;
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
                if(Entity.DNameData == "way"){
                    //nodeid
                    std::vector<TNodeIndex> TempIndices;
                    bool IsOneWay = false;
                    bool IsRoundAbout = false;
                    double MaxSpeed = 25;
                    while((Entity.DNameData != "way") || (Entity.DType != SXMLEntity::EType::EndElement)){
                        OSMReader.ReadEntity(Entity);
                        if(Entity.DType == SXMLEntity::EType::StartElement){
                            if(Entity.DNameData == "nd"){
                                auto NodeID = std::stoul(Entity.AttributeValue("ref"));
                                auto Search = DNodeIDToNodeIndex.find(NodeID);
                                if( Search != DNodeIDToNodeIndex.end()){
                                    //push value of node index
                                    TempIndices.push_back(Search->second);
                                }
                            }
                            if(Entity.DNameData == "tag"){
                                if(Entity.AttributeValue("k") == "oneway"){
                                    IsOneWay = Entity.AttributeValue("v") == "yes";
                                }
                                if(Entity.AttributeValue("k") == "maxspeed"){
                                    std::stringstream TempStream(Entity.AttributeValue("v"));
                                    TempStream>>MaxSpeed;
                                }
                                if(Entity.AttributeValue("k") == "junction"){
                                    IsRoundAbout = Entity.AttributeValue("v") == "roundabout";
                                }
                            }
                        }
                    }
                    size_t TempIndicesSize = TempIndices.size();
                    for(size_t Index = 0; Index + 1 < TempIndicesSize; Index++){
                        SEdge TempEdge;
                        auto FromNode = TempIndices[Index];
                        auto ToNode = TempIndices[Index+1];
                        TempEdge.DDistance = HaversineDistance(DNodes[FromNode].DLatitude, DNodes[FromNode].DLongitude,DNodes[ToNode].DLatitude, DNodes[ToNode].DLongitude);
                        TempEdge.DTime = TempEdge.DDistance / WalkingSpeed;
                        TempEdge.DOtherNodeIndex = ToNode;
                        DNodes[FromNode].DEdges.push_back(TempEdge);
                        if(!IsOneWay){
                            TempEdge.DOtherNodeIndex = FromNode;
                            DNodes[ToNode].DEdges.push_back(TempEdge);
                        }
                        else{
                            //TODO keep track of back edges for walking
                            

                        }
                    }
                }                     
            } 
        }
    }
    //CSV Readers
    //stops.csv reader stop_id,node_id
    CCSVReader StopReader(stops);
    CCSVReader RouteReader(routes);
    std::vector<std::string> Row;
    StopReader.ReadRow(Row);
    size_t StopColumn = -1;
    size_t NodeColumn = -1;
    for (size_t Index = 0; Index < Row.size(); Index++){
        if(Row[Index] == "stop_id"){
            StopColumn = Index;
        }
        if(Row[Index] == "node_id"){
            NodeColumn = Index;
        }
    }
    while(!StopReader.End()){
        if(StopReader.ReadRow(Row)){
            TStopID StopID = std::stoul(Row[StopColumn]);
            TNodeID NodeID = std::stoul(Row[NodeColumn]);
            DStopIDToNodeID[StopID] = NodeID;
            auto Search = DNodeIDToNodeIndex.find(NodeID);
            if(Search != DNodeIDToNodeIndex.end()){
                DStopIDToNodeIndex[StopID] = Search -> second;
            }
        }
    }
    //routes.csv reader  route,stop_id 
    size_t RouteColumn = -1;
    size_t StopRouteColumn = -1;
    //link stop ids from stop.csv
    RouteReader.ReadRow(Row);
    for(size_t Index = 0; Index < Row.size(); Index++){
        if(Row[Index] == "route"){
            RouteColumn = Index;
        }
        if(Row[Index] == "stop_id"){
            StopRouteColumn = Index;
        }
    }
    //double check to make sure this is correct
    //store routes[A,[20,21,23]] = size 1
    //https://stackoverflow.com/questions/698345/i-need-to-have-a-key-with-multiple-values-what-datastructure-would-you-recommen
    while(!RouteReader.End()){
        if(RouteReader.ReadRow(Row)){
            TStopID StopRouteID = std::stoul(Row[StopRouteColumn]);
            TRouteID RouteID = std::string(Row[RouteColumn]);
            DRouteToStopID[RouteID].emplace_back(StopRouteID);
            auto SearchStop = DRouteToStopID.find(RouteID);
            if(SearchStop != DRouteToStopID.end()){
                DRouteToStopID[RouteID] = (SearchStop -> second);
                //TODO link stop ids to node ids

                //std::cout << "DROUTE Values: "<<DRouteToStopID[RouteID] <<std::endl;
            }
        }
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
    auto Search = DNodeIDToNodeIndex.find(nodeid);
    if (Search != DNodeIDToNodeIndex.end()){
        auto NodeIndex = Search->second;
        return TLocation(DNodes[NodeIndex].DLatitude,DNodes[NodeIndex].DLongitude);
    }
    return TLocation(180.0, 360.0);
}

CMapRouter::TNodeID CMapRouter::GetNodeIDByStopID(TStopID stopid) const{
    auto Search = DStopIDToNodeID.find(stopid);
    if(Search != DStopIDToNodeID.end()){
        return Search->second;
    }
    return InvalidNodeID;
}

size_t CMapRouter::RouteCount() const{
    return DRouteToStopID.size();
    
}

std::string CMapRouter::GetSortedRouteNameByIndex(size_t index) const{
    auto counter = DRouteToStopID.begin();
    for(auto i = 0;i < index; i++){
        counter++;
    }
    return counter->first;

}

bool CMapRouter::GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops){
    //populate stops with route (A)
    //find Key(route)
    //DRouteToStopID[A,[20,21,23]]
    auto Search = DRouteToStopID.find(route);
    //get size of value 
    auto VectorSize = DRouteToStopID[route].size();
    //populate 'stops vector' with value of A [20,21,23] https://stackoverflow.com/questions/4263640/find-mapped-value-of-map
    if (Search != DRouteToStopID.end()){
        //loop through vector of values [20,21,23]
        auto RouteValues = Search->second;
        //std::cout << "Route Values: "<<RouteValues[0] <<std::endl;
        //for(auto i : Search->second){
        //    stops.push_back(i.first);
        //}
        for(auto it = DRouteToStopID.begin();it!=DRouteToStopID.end();it++){
            std::cout << "@ " << __LINE__ << std::endl;
            if(it-> first == route){
                //stops.push_back(RouteValues);
                //stops.push_back(DRouteToStopID.second[it]);
                for(auto iter = 0; iter < VectorSize ;iter++){
                    std::cout << "@ " << __LINE__ << std::endl;
                    stops.push_back(RouteValues[iter]);
                }
            }
        }
        //stops.pop_back();
        return true;
    }
    return false;
}

double CMapRouter::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    //node id keyd to prevnode and the distance between
    auto SizeOfMap = NodeCount();
    //src = 4, dest = 3 in TNodeID. We want them as TNodeIdx
    auto SearchSource = DNodeIDToNodeIndex.find(src);
    if(SearchSource != DNodeIDToNodeIndex.end()){
        std::cout << "@ " << __LINE__ << std::endl;
        src = SearchSource -> second; //src is now 3
    }
    //populate Unvisitednodes
    std::vector<double> MinDistance(SizeOfMap,INT_MAX);
    //[max,max,max,max,max,max]
    //DNode[3]on the first DEdge distance
    MinDistance[DNodes[src].DEdges[0].DDistance] = 0;
    //[max,max,max,max,max,max]
    std::set<std::pair<double,double>> ActiveVertices;
    ActiveVertices.insert({0,src});
    while(!ActiveVertices.empty()){
        std::cout << "@ " << __LINE__ << std::endl;
        auto where = ActiveVertices.begin()->second;
        if(where == dest){
            return MinDistance[where];
        }
        //erase node thats been visited
        ActiveVertices.erase(ActiveVertices.begin());
        for(auto i = 0; i < SizeOfMap; i++){
            //doesnt work
            if(MinDistance[i]>DNodes[i].DEdges[0].DDistance){
                std::cout << "@ " << __LINE__ << std::endl;
                ActiveVertices.erase({MinDistance[i],i});
                MinDistance[i] = MinDistance[where]+DNodes[i].DEdges[0].DDistance;
                ActiveVertices.insert({MinDistance[i],i});
            }
        }
        return INT_MAX;
    }
   // for(size_t i = 0; i < SizeOfMap; i++){
      // 
        //UnvisitedDNodeID.push(0.0,GetSortedNodeIDByIndex(i));
   // }
    
    //nodeID,pair is unvisited
    //where nodeID is all vertexes in map and pair prev node, shortest dist from src
    //1[?,0],2[1,dist],3[2,dist],4[3,dist],5[2,dist],6[1,dist]
    //https://stackoverflow.com/questions/29560245/c-how-to-get-first-and-second-element-of-pair-if-used-as-key-in-map
    //path is my visited
    //if path.find(dest) then return distance 
    //find shortests distance between two nodes with haversine distance
    
    //TNodeID StartingNode = UnvisitedNodes.find(src);

}

double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path){
    // Your code HERE
}

bool CMapRouter::GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const{
    // Your code HERE
}
