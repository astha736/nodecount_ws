#ifndef UTILITY_H
#define UTILITY_H

#include<vector>
#include <string>

using namespace std;

class Utility
{
public:
	// Make a static variable here 
	static int nRowMap;
	static int nColMap;
	static std::vector<std::vector<int>> Map;
	static int totalFreeNodes;
	static int totalBlockedNodes;
	static int totalExploredNodes;

	static bool updateParams(int rMap, int cMap);
	static bool getFileContent(std::string fileName);
	static bool updateMap(const int x, const int y);
	static bool pickNext(const int x, const int y, int &nextX, int &nextY);
	static bool initMap(int robotRow, int robotCol);
	static bool isComplete();

};
#endif
