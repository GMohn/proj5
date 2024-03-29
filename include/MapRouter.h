#ifndef MAPROUTER_H 	  			 	 
#define MAPROUTER_H

#include <vector>
#include <istream>
#include <CSVReader.h>
#include <XMLReader.h>
#include <unordered_map>



struct pair_hash
{
	template <class T1, class T2>
	std::size_t operator() (const std::pair<T1, T2> &pair) const
	{
		return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
	}
};

class CMapRouter{
    public:
        using TNodeID = unsigned long;
        using TStopID = unsigned long;
        using TLocation = std::pair<double, double>;
        using TPathStep = std::pair<std::string, TNodeID>;
        
        static const TNodeID InvalidNodeID;
    private:
        using TNodeIndex = size_t;
        //using TRouteID = std::string;
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
        std::unordered_map<std::string,std::vector<TStopID>> DRouteToStops;
        std::vector<std::string> DSortedRouteNames;
        std::unordered_map<std::pair<TNodeIndex,TNodeIndex>,std::vector<std::string>,pair_hash> DBusEdgeToRouteNames;
        std::unordered_map<std::pair<TNodeIndex,TNodeIndex>,std::vector<TNodeIndex>,pair_hash> DBusEdgeToPath;
        std::priority_queue<std::pair<double,TNodeID>> UnvisitedDNodeID;
        //std::unordered_map<TNodeID,SNode>DGraph;
        //std::unordered_map<TRouteID,std::vector<TStopID>> DRouteToNodeID; 

        double Dijkstras(TNodeIndex src, TNodeIndex dest, std::vector< TNodeIndex> &path, char type);
        
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
