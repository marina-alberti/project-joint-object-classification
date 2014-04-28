/*
 * ApiGraph.cpp
 *
 *  Created on: Dec 13, 2013
 *      Author: marina
 */

#include "ApiGraph.hpp"


ApiGraph::ApiGraph(vector<int> objectidsin, vector<int> categoryListin) {
	objectids = objectidsin;
	categoryList = categoryListin;
}

ApiGraph::ApiGraph(vector<vector<pairScore> > in) {
	votingTable = in;
}


/* @description:
 * This function recursively finds all the possible paths in a given scene.
 * A path is a set of pairs: (obectID, categoryLabel) that assigns an object
 * category label to each object in a scene.
 */
void ApiGraph::findAllPaths() {

	int nobjects = objectids.size();
	path mypath;

	// initialize the path map
	for (int j = 0; j < objectids.size(); j++) {
		mypath[objectids.at(j)] = -1;
	}
	for (int i = 0; i < categoryList.size(); i++) {
		findAllPathsUtil(nobjects, 0, i, mypath);
	}
}

void ApiGraph::findAllPathsUtil(int nobjects, int objectidIndex, int categoryIndex, path & mypath) {

	// assign to the path a new node consisting of the object id and the category label in input
	mypath[objectids.at(objectidIndex)] = categoryList.at(categoryIndex);

	// check the "next" object id, check that it is still a valid one
	if ( (objectidIndex + 1 ) < nobjects) {

		// for each object category label
		for (int i = 0; i < categoryList.size(); i++) {

			// calls recursively this function with the next object id and the selected category label in the loop
			findAllPathsUtil(nobjects, (objectidIndex + 1 ), i, mypath);
		}
	}
	else {
		allPaths.push_back(mypath);
	}
}


/*
 * This function computes all the possible paths given the information
 * in the 'voting table' which is the output of the 'voting scheme' test function.
 * For each object in the test scene with given object instance ID,
 * only a subset of all object category labels can be assigned - which depends on the object.
 */
void ApiGraph::findAllPathsShortlisted() {

	int nobjects = votingTable.size();
	path mypath;

	// Each row of the voting table represents a different test object in the test scene.
	int objectInstanceId = votingTable.at(0).at(0).first.first;

	// For each the of categories that can be assigned to the current test object
	for (int k = 0; k < votingTable.at(0).size(); k++) {

		findAllPathsShortlistedUtil(nobjects, 0, k, mypath);
	}

}

void ApiGraph::findAllPathsShortlistedUtil(int nobjects, int objectidIndex, int categoryIndex, path & mypath) {

	mypath[votingTable.at(objectidIndex).at(categoryIndex).first.first] = votingTable.at(objectidIndex).at(categoryIndex).first.second;

	if ( (objectidIndex + 1 ) < nobjects) {

		// for each object category label
		for (int i = 0; i < votingTable.at(objectidIndex + 1).size(); i++) {
			findAllPathsShortlistedUtil(nobjects, (objectidIndex + 1 ), i, mypath);
		}
	}
	else {
		allPaths.push_back(mypath);
	}
}


void ApiGraph::printAllPaths() {

	 for( vector<path>::iterator ii=allPaths.begin(); ii!=allPaths.end(); ++ii) {

		 path currentPath = *ii;
		 cout <<"This is a new path::" << endl;

		 for( map<int,int>::iterator jj=currentPath.begin(); jj!=currentPath.end(); ++jj) {
			 cout << (*jj).first << " : " << (*jj).second << endl;

		 }
	 }
}

