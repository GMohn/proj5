#ifndef MAPROUTER_H 	  			 	 
#define MAPROUTER_H

#include <vector>
#include <istream>
#include <CSVReader.h>
#include <XMLReader.h>
#include  <unordered_map>

class CMapRouter{
    public:
        using TNodeID = unsigned long;
        using TStopID = unsigned long;
        using TLocation = std::pair<double, double>;
        using TPathStep = std::pair<std::string, TNodeID>;
        
        static const TNodeID InvalidNodeID;
    private:
        using TNodeIndex = size_t;
        using TRouteID = std::string;
        struct SEdge{
            TNodeIndex DOtherNodeIndex;
            double DDistance;
            double DTime;
            double DMaxSpeed;
        };

        struct SNode{
            TNodeID DNodeID;
            double DLatitude;
            double DLongitude;
            std::vector<SEdge> DEdges;
        };

        std::vector< SNode > DNodes;
        std::vector<TNodeID> DSortNodeIDs;
        std::unordered_map< TNodeID, TNodeIndex> DNodeIDToNodeIndex;
        std::unordered_map<TStopID, TNodeID> DStopIDToNodeID;
        std::unordered_map<TStopID, TNodeIndex> DStopIDToNodeIndex;
        std::unordered_map<TRouteID,std::vector<TStopID>> DRouteToStopID;
        //std::unordered_map<TRouteID,std::vector<TStopID>> DRouteToNodeID; 
        
    public:
        CMapRouter();
        ~CMapRouter();
        
        static double HaversineDistance(double lat1, double lon1, double lat2, double lon2);
        static double CalculateBearing(double lat1, double lon1,double lat2, double lon2);
        
        bool LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes);
        size_t NodeCount() const;
        TNodeID GetSortedNodeIDByIndex(size_t index) const;
        TLocation GetSortedNodeLocationByIndex(size_t index) const;
        TLocation GetNodeLocationByID(TNodeID nodeid) const;
        TNodeID GetNodeIDByStopID(TStopID stopid) const;
        size_t RouteCount() const;
        std::string GetSortedRouteNameByIndex(size_t index) const;
        bool GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops);
        
        double FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path);
        double FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path);
        bool GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const;
};

#endif
