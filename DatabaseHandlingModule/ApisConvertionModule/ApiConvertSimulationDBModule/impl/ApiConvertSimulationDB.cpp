/*
 * ApiConvertSimulationDB.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 */

#include "ApiConvertSimulationDB.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <iostream>

#define DEBUG 0
#define TESTFLAG 1

using namespace boost::property_tree;


/*
 * @description:
 * This function parses the JSON file which stores information about the objects in a set of
 * 3D scenes. Iteratively parses each of the scenes and stores the information
 * into the Internal Data Structure ('SceneInformation').
*/
void ApiConvertSimulationDB::parseFileJSON(string fileNameJSON, vector<SceneInformation> & sceneInformationList) {

  int countScene = 0;
  float  _deskLength = 1.2;
  float  _deskWidth = 2;

  boost::property_tree::ptree doc;
  boost::property_tree::read_json(fileNameJSON, doc);

  // for each scene in the JSON file
  BOOST_FOREACH(boost::property_tree::ptree::value_type& sceneRoot, doc.get_child("root")) {

    SceneInformation currentScene;
    currentScene.setReferenceLength(_deskLength);
    currentScene.setReferenceWidth(_deskWidth);
    currentScene.setReferenceCentroid();
    if (DEBUG) {
      cout << "Parsing a new scene:" << endl;
    }
    ApiConvertSimulationDB::parseSceneJSON(sceneRoot, currentScene);
    sceneInformationList.push_back(currentScene);
    countScene++;
  }
}


/*
 * @description:
 * This function parses each of the scenes of the dataset, contained in the JSON file,
 * and stores the information in the Internal Data Structure ('SceneInformation').
*/
void ApiConvertSimulationDB::parseSceneJSON(boost::property_tree::ptree::value_type & sceneRoot, SceneInformation & currentScene){

  vector<Object> objectVector;
  BOOST_FOREACH (boost::property_tree::ptree::value_type& sceneField, sceneRoot.second) {

    // "type" field: contains the object category of each object and the "name" of each object
    if (strcmp(sceneField.first.c_str(), "type") == 0) {
    	int counter = 0;

    	// for each of the objects in the scene
    	BOOST_FOREACH (boost::property_tree::ptree::value_type& objectList, sceneField.second) {
    		if (DEBUG) {
    			cout << "A new object in the test scene: " << objectList.first << endl;
    			cout << "And the object category type is: " << objectList.second.get_value<std::string>() << endl;
    		}
    		Object newObject;
    		newObject.setInstanceName(objectList.first);
    		newObject.setObjectName(objectList.second.get_value<std::string>());
    		newObject.setCategoryName(objectList.second.get_value<std::string>());
    		newObject.setInstanceID(counter);
    		objectVector.push_back(newObject);
    		counter++;
    	}
    }

    // Sets the 3D position (centroid) of each object
    if (strcmp(sceneField.first.c_str(), "position") == 0) {
    	int countObject = 0;

    	// for each of the objects in the scene
    	BOOST_FOREACH (boost::property_tree::ptree::value_type& objectList, sceneField.second) {
    		pcl::PointXYZ centroid;

    		// Loops over the x y z coordinates of the current vertix
    		int countCoord = 0;
    		BOOST_FOREACH (boost::property_tree::ptree::value_type& pointCoordinate, objectList.second) {
    			if (countCoord == 0) {
    				centroid.x = pointCoordinate.second.get_value<double>();
    			}
    			else if (countCoord == 1) {
    				centroid.y = pointCoordinate.second.get_value<double>();
    			}
    			else if (countCoord == 2) {
    				centroid.z = pointCoordinate.second.get_value<double>();
    			}
    			countCoord++;
    		}
    		(objectVector[countObject]).setCentroidPoint(centroid) ;
    		countObject++;
    	}
    }

    // Sets the bounding box of each object
    if (strcmp(sceneField.first.c_str(), "bbox") == 0) {

    	int countObject = 0;

    	// for each of the objects in the scene
    	BOOST_FOREACH (boost::property_tree::ptree::value_type& objectList, sceneField.second) {

    		vector<pcl::PointXYZ> boundingBoxVertices;

    		// for each 3D vertix of the bounding box
    		BOOST_FOREACH (boost::property_tree::ptree::value_type& bboxPoint, objectList.second) {
    			pcl::PointXYZ boundingBoxVertix;

    			// Loops over the x y z coordinates of the current vertix
    			int countCoord = 0;
    			BOOST_FOREACH (boost::property_tree::ptree::value_type& bboxPointCoordinate, bboxPoint.second) {
    				if (countCoord == 0) {
    					boundingBoxVertix.x = bboxPointCoordinate.second.get_value<double>();
    				}
    				else if (countCoord == 1) {
    					boundingBoxVertix.y = bboxPointCoordinate.second.get_value<double>();
    				}
    				else if (countCoord == 2) {
    					boundingBoxVertix.z = bboxPointCoordinate.second.get_value<double>();
    				}
    				countCoord++;
    			}
    			boundingBoxVertices.push_back(boundingBoxVertix);
    		}
    		currentScene.addObject(objectVector[countObject]);
    		countObject++;
    	}
    }
  }
}


/*
 * @description:
 * Introduces a random variation in the three dimensions
 * of the object bounding box (length, width and height)
 * to randomly vary the object size for more realistic bounding box
 * representations of the simulated data.
 * The possible range of the variation is defined by the input argument
 * 'noiseAmount'.
 */
vector<pcl::PointXYZ> ApiConvertSimulationDB::addNoiseBoundingBox(vector<pcl::PointXYZ> boundingBoxVertices, int noiseAmount) {

	double x1 = boundingBoxVertices.at(0).x;
	double y1 = boundingBoxVertices.at(0).y;
	double z1 = boundingBoxVertices.at(0).z;
	double z2 = boundingBoxVertices.at(1).z;
	double x4 = boundingBoxVertices.at(3).x;
	double y4 = boundingBoxVertices.at(3).y;
	double x5 = boundingBoxVertices.at(4).x;
	double y5 = boundingBoxVertices.at(4).y;

	double length = sqrt( pow((y1 - y5), 2) + pow((x1 - x5), 2) );
	double width =  sqrt( pow((y1 - y4), 2) + pow((x1 - x4), 2) );
	double height = z2 - z1;

	if (TESTFLAG) {
		cout << "Length =  " << length << endl;
		cout << "Width =  " << width << endl;
		cout << "Height =  " << height << endl;
	}
	if (TESTFLAG) {
		cout << "The initial bounding boz is:: " << endl;
		for (int i = 0; i < boundingBoxVertices.size(); i++) {
			cout << boundingBoxVertices.at(i).x << "  " << boundingBoxVertices.at(i).y << "  " << boundingBoxVertices.at(i).z << endl;
		}
	}

	// Modify length

	int kx_int = rand() % noiseAmount;
	double kx = (double) kx_int / 100;
	int sign_term_int = rand() %  100;
	int sign_length;
	if (sign_term_int < 50) {
		sign_length = 1;
	}
	else {
		sign_length = -1;
	}
	double lm = length * kx;
	double theta = atan2((y5 - y1), (x5 - x1));
	double lx = lm * abs(cos(theta));
	double ly = lm * abs(sin(theta));
	int signx, signy;
	if (x5 > x1) {
		signx = -1;
	}
	else {
		signx = 1;
	}
	if (y5 > y1) {
		signy = -1;
	}
	else {
		signy = 1;
	}
	x1 = x1 + signx * lx * sign_length;
	y1 = y1 + signy * ly * sign_length;
	x5 = x5 - signx * lx * sign_length;
	y5 = y5 - signy * ly * sign_length;
	x4 = x4 + signx * lx * sign_length;
	y4 = y4 + signy * ly * sign_length;
	double x8 = boundingBoxVertices.at(7).x - signx * lx * sign_length;
	double y8 = boundingBoxVertices.at(7).y - signy * ly * sign_length;
	double x2 = x1;
	double y2 = y1;
	double x6 = x5;
	double y6 = y5;
	double x3 = x4;
	double y3 = y4;
	double x7 = x8;
	double y7 = y8;

	// Modify width

	int ky_int = rand() % noiseAmount;
	double ky = (double) ky_int / 100;
	sign_term_int = rand() %  100;
	int sign_width;
	if (sign_term_int < 50) {
		sign_width = 1;
	}
	else {
		sign_width = -1;
	}
	double wm = width * ky;
	//double theta = atan2((y5 - y1), (x5 - x1));
	double wx = wm * abs(sin(theta));
	double wy = wm * abs(cos(theta));
	//int signx, signy;
	if (x4 > x1) {
		signx = -1;
	}
	else {
		signx = 1;
	}
	if (y4 > y1) {
		signy = -1;
	}
	else {
		signy = 1;
	}
	x1 = x1 + signx * wx * sign_width;
	y1 = y1 + signy * wy * sign_width;
	x5 = x5 + signx * wx * sign_width;
	y5 = y5 + signy * wy * sign_width;
	x4 = x4 - signx * wx * sign_width;
	y4 = y4 - signy * wy * sign_width;
	x8 = x8 - signx * wx * sign_width;
	y8 = y8 - signy * wy * sign_width;
	x2 = x1;
	y2 = y1;
	x6 = x5;
	y6 = y5;
	x3 = x4;
	y3 = y4;
	x7 = x8;
	y7 = y8;

	// Modify height

	int kz_int = rand() % noiseAmount;
	double kz = (double) kz_int / 100;
	sign_term_int = rand() %  100;
	int signz;
	if(sign_term_int < 50) {
		signz = 1;
	}
	else {
		signz = -1;
	}
	double h = height * kz * signz;
	z2 = boundingBoxVertices.at(1).z + h;
	double z3 = boundingBoxVertices.at(2).z + h;
	double z6 = boundingBoxVertices.at(5).z + h;
	double z7 = boundingBoxVertices.at(6).z + h;

	vector<pcl::PointXYZ> bboxNew = boundingBoxVertices;
	bboxNew.at(0).x = x1;
	bboxNew.at(0).y = y1;
	bboxNew.at(1).x = x2;
	bboxNew.at(1).y = y2;
	bboxNew.at(2).x = x3;
	bboxNew.at(2).y = y3;
	bboxNew.at(3).x = x4;
	bboxNew.at(3).y = y4;
	bboxNew.at(4).x = x5;
	bboxNew.at(4).y = y5;
	bboxNew.at(5).x = x6;
	bboxNew.at(5).y = y6;
	bboxNew.at(6).x = x7;
	bboxNew.at(6).y = y7;
	bboxNew.at(7).x = x8;
	bboxNew.at(7).y	= y8;
	bboxNew.at(1).z = z2;
	bboxNew.at(2).z = z3;
	bboxNew.at(5).z = z6;
	bboxNew.at(6).z = z7;

	if (TESTFLAG) {
		cout << "The final bounding boz is:: " << endl;
		for (int i = 0; i < bboxNew.size(); i++) {
			cout << bboxNew.at(i).x << "  " << bboxNew.at(i).y << "  " << bboxNew.at(i).z << endl;
		}
	}

	return bboxNew;
}



