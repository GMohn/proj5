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
double CMapRouter::Dijkstras(TNodeIndex src, TNodeIndex dest, std::vector< TNodeIndex> &path, char type){
    std::vector<double> Distance(DNodes.size(),std::numeric_limits<double>::max());
    std::vector<TNodeIndex> Previous(DNodes.size(),InvalidNodeID);
    std::vector<TNodeIndex> PriorityHeap;
    auto Compare = [&Distance](TNodeIndex idx1, TNodeIndex idx2){
        return (Distance[idx1] < Distance[idx2]);
    };
    Distance[src] = 0;
    Previous[src] = src;
    PriorityHeap.push_back(src);
    while(!PriorityHeap.empty()){
        std::make_heap(PriorityHeap.begin(), PriorityHeap.end(), Compare);
        auto Current = PriorityHeap.front();
        std::pop_heap(PriorityHeap.begin(),PriorityHeap.end(), Compare);
        PriorityHeap.pop_back();
        for(auto Edge : DNodes[Current].DEdges){
            auto AltDist = Distance[Current] + (type == 'D' ? Edge.DDistance / Edge.DMaxSpeed : (type == 'd' ? Edge.DDistance : Edge.DTime));
            if(AltDist <Distance[Edge.DOtherNodeIndex]){
                Distance[Edge.DOtherNodeIndex] = AltDist;
                if(Previous[Edge.DOtherNodeIndex] == InvalidNodeID){
                    PriorityHeap.push_back(Edge.DOtherNodeIndex);
                }
                Previous[Edge.DOtherNodeIndex] = Current;
            }
        }
    }
    return Distance[dest];

}

bool CMapRouter::LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes){
    CXMLReader OSMReader(osm);
    const double WalkingSpeed = 3.0;
    std::unordered_map<TNodeIndex, std::vector<SEdge>> EdgesToAdd;
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
                        std::cout<<"Adding Edge: "<<std::endl;
                        SEdge TempEdge, TempEdgeBack;
                        auto FromNode = TempIndices[Index];
                        auto ToNode = TempIndices[Index+1];
                        //creating edge going forward
                        TempEdge.DDistance = HaversineDistance(DNodes[FromNode].DLatitude, DNodes[FromNode].DLongitude,DNodes[ToNode].DLatitude, DNodes[ToNode].DLongitude);
                        TempEdge.DTime = TempEdge.DDistance / WalkingSpeed;
                        TempEdge.DOtherNodeIndex = ToNode;
                        DNodes[FromNode].DEdges.push_back(TempEdge);
                        //creating a edge going back
                        TempEdgeBack.DDistance = TempEdge.DDistance;
                        TempEdgeBack.DTime = TempEdge.DDistance / WalkingSpeed;
                        TempEdgeBack.DOtherNodeIndex = FromNode;
                        DNodes[ToNode].DEdges.push_back(TempEdgeBack);
                        if(!IsOneWay){
                            std::cout<<"Add edges"<<std::endl;
                            TempEdge.DOtherNodeIndex = FromNode;
                            DNodes[ToNode].DEdges.push_back(TempEdge);
                        }
                        else{
                            if(EdgesToAdd.end() == EdgesToAdd.find(ToNode)){
                                EdgesToAdd[ToNode] = {};    
                            }
                            TempEdgeBack.DOtherNodeIndex = FromNode;
                            EdgesToAdd[ToNode].push_back(TempEdge);
                            /*if(Index == TempIndicesSize ){
                                std::cout<<"Added edge to NodeID 1"<<std::endl;
                                ToNode = TempIndices[0];
                                DNodes[ToNode].DEdges.push_back(TempEdge);
                            }
                            //not at last node and can connect both ways
                                std::cout<<"Added undirected edges"<<std::endl;
                                DNodes[FromNode].DEdges.push_back(TempEdge);
                                TempEdge.DOtherNodeIndex = FromNode;
                                DNodes[ToNode].DEdges.push_back(TempEdge);*/
                        }
                    }
                }                     
            } 
        }
    }
    //CSV Readers
    //stops.csv reader stop_id,node_id
    CCSVReader StopReader(stops);
    
    std::vector<std::string> Row;
    StopReader.ReadRow(Row);
    SEdge TempBusEdge;
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
            std::cout << "@ " << __LINE__ << std::endl;
            TStopID StopID = std::stoul(Row[StopColumn]);
            TNodeID NodeID = std::stoul(Row[NodeColumn]);
            DStopIDToNodeID[StopID] = NodeID;
            auto Search = DNodeIDToNodeIndex.find(NodeID);
            if(Search != DNodeIDToNodeIndex.end()){
                DStopIDToNodeIndex[StopID] = Search -> second; //this is my node index
                //update edge with busedge based on nodeids
                //auto StopNodeIdx = DStopIDToNodeIndex[StopID];
                //TempBusEdge.DTime = TempEdge.DDistance / MaxSpeed;
                //TempBusEdge.DOtherNodeIndex = StopNodeIdx+1;
                //DNodes[StopNodeIdx].DEdges.push_back(TempBusEdge);
            }

        }
    }
    CCSVReader RouteReader(routes);
    //routes.csv reader  route,stop_id 
    size_t RouteColumn = -1;
    //size_t StopColumn = -1;
    //link stop ids from stop.csv
    RouteReader.ReadRow(Row);
    for(size_t Index = 0; Index < Row.size(); Index++){
        if(Row[Index] == "route"){
            RouteColumn = Index;
        }
        if(Row[Index] == "stop_id"){
            StopColumn = Index;
        }
    }
    //double check to make sure this is correct
    //store routes[A,[20,21,23]] = size 1
    //std::vector<TRouteID>TempStops;
    //https://stackoverflow.com/questions/698345/i-need-to-have-a-key-with-multiple-values-what-datastructure-would-you-recommen
    while(!RouteReader.End()){
        if(RouteReader.ReadRow(Row)){
            TStopID StopID = std::stoul(Row[StopColumn]);
            std::string RouteName = Row[RouteColumn];
            //DRouteToStops[RouteName].emplace_back(StopRouteID);
            
            //link stop ids to node ids
            //auto SearchNodeId = DStopIDToNodeID.find(StopID)
            //DStopIDToNodeID[StopRouteID] = SearchNodeId;
            if(DRouteToStops.end()== DRouteToStops.find(RouteName)){
                DRouteToStops[RouteName] = {};
                DSortedRouteNames.push_back(RouteName);
                //DRouteToStops[RouteName] = (SearchStop -> second);
                //DStopIDToNodeID.push_back(StopRouteID)
                //std::cout << "DROUTE Values: "<<DRouteToStops[RouteID] <<std::endl;
            }
            DRouteToStops[RouteName].push_back(StopID);
        }
    }
    for(auto RouteInfo : DRouteToStops){
        for(size_t Index = 0; Index + 1 < RouteInfo.second.size(); Index++){
            auto StartStop = RouteInfo.second[Index];
            auto EndStop = RouteInfo.second[Index+1];
            auto StartIndex = DNodeIDToNodeIndex[DStopIDToNodeID[StartStop]];
            auto EndIndex = DNodeIDToNodeIndex[DStopIDToNodeID[EndStop]];
            auto EdgePair = std::make_pair(StartIndex, EndIndex);
            if(DBusEdgeToRouteNames.end() == DBusEdgeToRouteNames.find(EdgePair)){
                DBusEdgeToRouteNames[EdgePair] = {};
            }
            DBusEdgeToRouteNames[EdgePair].push_back(RouteInfo.first);
        }
    }
    for(auto EdgeInfo : DBusEdgeToRouteNames){
        std::vector<TNodeIndex> Path;
        TNodeIndex SourceIndex = EdgeInfo.first.first;
        TNodeIndex DestIndex = EdgeInfo.first.second;
        double Time = Dijkstras(SourceIndex,DestIndex,Path,'D');
        DBusEdgeToPath[EdgeInfo.first] = Path;
        if(EdgesToAdd.end() == EdgesToAdd.find(SourceIndex)){
            EdgesToAdd[SourceIndex] = {};
        }
        SEdge TempEdge;
        TempEdge.DOtherNodeIndex = DestIndex;
        TempEdge.DTime = Time + 1/120.0;
        TempEdge.DDistance = std::numeric_limits<double>::max();
        TempEdge.DMaxSpeed = 1;
        EdgesToAdd[SourceIndex].push_back(TempEdge);

    }

    for(auto EdgeInfo: EdgesToAdd){
        for(auto Edge : EdgeInfo.second){
            DNodes[EdgeInfo.first].DEdges.push_back(Edge);
        }
    }

    std::sort(DSortedRouteNames.begin(),DSortedRouteNames.end());
    std::sort(DSortNodeIDs.begin(),DSortNodeIDs.end());
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
    return DRouteToStops.size();
    
}

std::string CMapRouter::GetSortedRouteNameByIndex(size_t index) const{
    auto counter = DRouteToStops.begin();
    for(auto i = 0;i < index; i++){
        counter++;
    }
    return counter->first;

}

bool CMapRouter::GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops){
    //populate stops with route (A)
    //find Key(route)
    //DRouteToStops[A,[20,21,23]]
    auto Search = DRouteToStops.find(route);
    //get size of value 
    auto VectorSize = DRouteToStops[route].size();
    //populate 'stops vector' with value of A [20,21,23] https://stackoverflow.com/questions/4263640/find-mapped-value-of-map
    if (Search != DRouteToStops.end()){
        //loop through vector of values [20,21,23]
        auto RouteValues = Search->second;
        //std::cout << "Route Values: "<<RouteValues[0] <<std::endl;
        //for(auto i : Search->second){
        //    stops.push_back(i.first);
        //}
        for(auto it = DRouteToStops.begin();it!=DRouteToStops.end();it++){
            std::cout << "@ " << __LINE__ << std::endl;
            if(it-> first == route){
                //stops.push_back(RouteValues);
                //stops.push_back(DRouteToStops.second[it]);
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
    std::unordered_map<TNodeIndex,TNodeIndex> VisitedFrom;
    //node id keyd to prevnode and the distance between
    auto SizeOfMap = NodeCount();
    //src = 4, dest = 3 in TNodeID. We want them as TNodeIdx
    auto SearchSource = DNodeIDToNodeIndex.find(src)->second;
    /*if(SearchSource != DNodeIDToNodeIndex.end()){
        std::cout << "@ " << __LINE__ << std::endl;
        src = SearchSource; //src is now idx 3
    }*/
    auto SearchDest = DNodeIDToNodeIndex.find(dest)->second;
    //if(SearchDest != DNodeIDToNodeIndex.end()){
        //std::cout << "@ " << __LINE__ << std::endl;
        //dest = SearchDest -> second; //src is now 3
    //}
    typedef std::pair<double,double> DPair;
    //populate Unvisitednodes
    std::vector<double> MinDistance(SizeOfMap,std::numeric_limits<double>::max());
    //[max,max,max,max,max,max]
    //DNode[3]on the first DEdge distance
    MinDistance[SearchSource] = 0;
    //[max,max,max,max,max,max]
    std::priority_queue<DPair,std::vector<DPair>,std::greater<DPair>> ActiveVertices;
    ActiveVertices.push({0,SearchSource});
    while(!ActiveVertices.empty()){
        std::cout << "@ " << __LINE__ << std::endl;
        auto CurrPair = ActiveVertices.top();
        ActiveVertices.pop();
        //std::cout<<"Active Size : "<<ActiveVertices.size()<<" @ Line: "<< __LINE__<<std::endl;
        //get node Index
        auto CurrNodeIndex = CurrPair.second;
        auto CurrDist = CurrPair.first;
        //std::cout<<"NodeIdx: "<<CurrNodeIndex<<" SearchDest: "<<SearchDest<< std::endl;
        if(CurrNodeIndex == SearchDest){
            //create path by indices
            std::vector<TNodeIndex>PathIndices;
            //pushing backwards            
            auto Current = SearchDest;
            std::cout << "@ " << __LINE__ << std::endl;
            //Construct Backwards path from VisitedFrom
            while(Current != SearchSource){ 
                //std::cout << "Current: "<<Current <<"@ " << __LINE__ << std::endl;
                //std::cout << "SearchSource: "<<SearchSource<< std::endl;
                auto CurrNodeID = DNodes[Current].DNodeID;
                PathIndices.push_back(CurrNodeID);
                Current = VisitedFrom[Current];
                
            }            
            //Populate path by reversing backwards VisitedFrom
            //use node id
            path.push_back(src);
            std::cout << "@ " << __LINE__ << std::endl;
            auto PathSize = PathIndices.size()-1;
            std::cout << "PathSize: " << PathSize<< std::endl;
            for(int i = PathSize;i >= 0; i--){
                std::cout << "i: " << i << std::endl;
                path.push_back(PathIndices[i]);
                //std::cout << "@ " << __LINE__ << std::endl;
            }
            return CurrDist;
        }
        //auto DNodeIndex = DNodeIDToNodeIndex[CurrNodeIndex];
        auto CurrNode = DNodes[CurrNodeIndex];
        for(auto element: CurrNode.DEdges){
            auto DistOfElements = CurrDist+element.DDistance;
            auto DistOfPrevEdge = MinDistance[element.DOtherNodeIndex];
            std::cout <<element.DOtherNodeIndex<<std::endl;
            
            if(DistOfElements<DistOfPrevEdge){
                std::cout << "@ " << __LINE__ << std::endl;
                //update mindistance
                MinDistance[element.DOtherNodeIndex] = DistOfElements;
                //update priorityqueue
                ActiveVertices.push({DistOfElements,element.DOtherNodeIndex});
                //std::cout<<"Active Size : "<<ActiveVertices.size()<<" @ Line: "<< __LINE__<<std::endl;
                //update VisitedFrom draw our path
                std::cout<<"Setting Visited From: "<<element.DOtherNodeIndex<< " to: "<< CurrNodeIndex<<std::endl;
                VisitedFrom[element.DOtherNodeIndex] = CurrNodeIndex;
            }
        }  

    }
    return std::numeric_limits<double>::max();


}

double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path){
    // Your code HERE
}

bool CMapRouter::GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const{
    // Your code HERE
}
