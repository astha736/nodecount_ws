#include <utility.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/console.h>

/*

Map Levels:
0 		-> free 
-1 		-> blocked 
1		-> explored 
-2 		-> a robot is moving towards it

*/


std::vector<std::vector<int>> Utility::Map;
int Utility::nRowMap = 0;
int Utility::nColMap = 0;
int Utility::totalFreeNodes = 0; // the original number of free nodes 
int Utility::totalBlockedNodes = 0;


int Utility::totalExploredNodes = 0; // as the code advances, the number is updated

bool Utility::updateParams(int rMap, int cMap){

	if(rMap == 0 || cMap == 0) return false;

	nRowMap = rMap;
	nColMap = cMap;
	totalFreeNodes = rMap*cMap;

	for(int i = 0; i < nRowMap; i++){
		std::vector<int> temp_col(nColMap,0);
		Map.push_back(temp_col);
	}
	return true;
}

bool Utility::pickNext(const int x, const int y, int & nextX, int & nextY){
	nextX = x;
	nextY = y;

  if(x+1 >= 0 && x+1 <= nRowMap -1){
    if(Map[x+1][y] == 0){
      Map[x+1][y] = -2;
      nextX = x+1;
      return true;
    }
  }
  if(x-1 >= 0 && x-1 <= nRowMap -1){
    if(Map[x-1][y] == 0){
      Map[x-1][y] = -2;
      nextX = x-1;
      return true;
    }
  }
  if(y+1 >= 0 && y+1 <= nColMap -1){
    if(Map[x][y+1] == 0){
      Map[x][y+1] = -2;
      nextY = y+1;
      return true;
    }
  }
  if(y-1 >= 0 && y-1 <= nColMap -1){
    if(Map[x][y-1] == 0){
      Map[x][y-1] = -2;
      nextY = y-1;
      return true;
    }
  }
  return false;
}

bool Utility::updateMap(const int x, const int y){
	// the update position can't be a 0 or -1, it can only be -2 or 1
	if(Map[x][y] == 0 || Map[x][y] == -1){
		return false;
	}
	if (Map[x][y] == -2){
		Map[x][y] = 1;
		totalExploredNodes = totalExploredNodes +1;
	}

	return true;
	
}

bool Utility::initMap(int robotRow, int robotCol, int val){
	if(val == 1 & Map[robotRow][robotCol] == 0){
		Map[robotRow][robotCol] = val;
		totalExploredNodes = totalExploredNodes +1;
		return true;
	}
	else if(val == -1  & Map[robotRow][robotCol] == -2 ){
		Map[robotRow][robotCol] = val;
		totalExploredNodes = totalExploredNodes +1;
		return true;
	}
	else return false;
}

bool Utility::isComplete(){
	if (totalExploredNodes == totalFreeNodes) return true;
	else return false;
}
