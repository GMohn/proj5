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
        
        struct SNode{
            TNodeID DNodeID;
            double DLatitude;
            double DLongitude;

        };

        std::vector< SNode > DNodes;
        std::unordered_map< TNodeID, TNodeIndex> DNodeIDToNodeIndex;
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
