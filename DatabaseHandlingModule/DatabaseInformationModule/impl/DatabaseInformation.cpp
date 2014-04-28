/*
 * DatabaseInformation.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 */

#include "DatabaseInformation.hpp"

#define DEBUG 0
#define TESTFLAG 0

DatabaseInformation::DatabaseInformation(int in) {
  numberOfScenes = 0;
  numberOfCategories = in;
}

DatabaseInformation::DatabaseInformation(vector<SceneInformation> in, int cat) {
	sceneList = in;
	numberOfScenes = in.size();
	numberOfCategories = cat;
}

/*
 * @description:
 * Stores the database information by parsing a set of XML files
 * containing the data annotations.
 */
void DatabaseInformation::loadAnnotations_KTH(vector<string> fileList) {

  for (int i = 0; i < fileList.size(); i++ ) {
    string filenameXML = fileList.at(i);

    if (TESTFLAG)  {
    	cout << "In loadAnnotationsInIDS, The XML file name is: " << filenameXML << endl;
    }

    SceneInformation currentScene;
    ApiConvertKTHDB converter;
    converter.parseFileXML(filenameXML, currentScene);
    sceneList.push_back(currentScene);
    numberOfScenes++;
    if (DEBUG) {
    	cout << "Added a new scene to the sceneList. " << endl;
    }
  }
}

/*
 * @description:
 * Stores the database information from simulated data by parsing a JSON file
 * containing the data annotations.
 */
void DatabaseInformation::loadAnnotations_Simulation(string fileAnnotations) {

  ApiConvertSimulationDB::parseFileJSON(fileAnnotations, sceneList);
  numberOfScenes = sceneList.size();
  if (DEBUG)  {
	  cout << "The number of scenes in the database is : " << numberOfScenes << endl;
  }
}

/*
 * @description:
 * Stores the database information of real world data by parsing a JSON file
 * containing the data annotations.
 */
void DatabaseInformation::loadAnnotations_RealWorld(string fileAnnotations) {
  if (TESTFLAG)  {
	  cout << "The JSON file name is: " << fileAnnotations << endl;
  }
  ApiConvertRealWorldDB::parseFileJSON(fileAnnotations, sceneList);
  numberOfScenes = sceneList.size();
  if (DEBUG)  {
	  cout << "The number of scenes in the database is : " << numberOfScenes << endl;
  }
}


void DatabaseInformation::printSceneInformation() {

	for (int i = 0; i < 1; i++) {
		SceneInformation scene = sceneList.at(i);
		int nObjects = scene.getNumberOfObjects();

		cout << endl << "Scene number " << i << endl;
		cout << "The scene contains " << nObjects << " objects." << endl;

		vector<Object> listObj = scene.getObjectList();

		for (int j = 0; j < listObj.size(); j++) {

			cout << endl << "New object" << endl;

			Object obj = listObj.at(j);
			int id = obj.getActualObjectID();
			cout << "the actual object ID is: " << id << endl;
			string category = obj.getCategoryName();
			cout << "The category name is: " << category << endl;
			string objectname = obj.getObjectName();
			cout << "The obj name is: " << objectname << endl;
			string instance = obj.getInstanceName();
			cout << "The instance name is: " << instance << endl;

			pcl::PointXYZ centroid = obj.getCentroid();
			cout << "The centroid is:   " << centroid.x << "   " << centroid.y << "  " << centroid.z  << endl;

			pcl::PointCloud<pcl::PointXYZ> bbox = obj.getBoundingBox();
			cout << "The size of the bounding box is " << bbox.size() << endl;
		}

		cout << "End of object list" << endl;
	}

	cout <<  "End of scene list " << endl;
}



