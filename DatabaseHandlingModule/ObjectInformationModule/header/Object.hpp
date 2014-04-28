/*
 * Object.hpp
 *
 *  Created on: Nov 17, 2013
 *      Author: marina
 */

#ifndef OBJECT_HPP_
#define OBJECT_HPP_

#include<iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "utils.hpp"


using namespace std;

class Object{

private:

	/*
	 * The name of an object instance as read  from the manual annotation tool.
	 */
	string objectName;

	/*
	 * The name of an object instance as read from the simulated dataset.
	 */
	string instanceName;

	/*
	 * A string identifier for the object categories
	 */
	string categoryName;

	/*
	 * An int identifier for the object categories
	 */
	int actualObjectID;

	/* An int identifier for the object instances representing the instance id */
	int instanceID;

	int predictedObjectID;

	pcl::PointCloud<pcl::PointXYZ> boundingBox;
	pcl::PointXYZ centroid;


public:

	Object();
	void setObjectName(string);
	void setPredictedObjectID(int i) { predictedObjectID = i; }
	void setActualObjectID(int i) { actualObjectID = i; }
	void setInstanceName(string);
	void setCategoryName(string);
	void setInstanceID(int i) { instanceID = i; }

	void setObjectParameters(vector<pcl::PointXYZ>, string = "", string = "");

	void setBoundingBox(vector<pcl::PointXYZ>);
	void setCentroidPoint(pcl::PointXYZ);
	void setCentroid();


	int getActualObjectID() { return actualObjectID; }
	int getPredictedObjectID() { return predictedObjectID; }
	string getObjectName() { return objectName; }
	pcl::PointCloud<pcl::PointXYZ> getBoundingBox();
	pcl::PointXYZ getCentroid();
	string getInstanceName() { return instanceName; }
	string getCategoryName() { return categoryName; }
	int getInstanceID() { return instanceID; }

};

#endif /* OBJECT_HPP_ */
