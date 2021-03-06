/*
 * SceneInformation.hpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 */

#ifndef SCENEINFORMATION_HPP_
#define SCENEINFORMATION_HPP_


#include <string.h>
#include <boost/property_tree/ptree.hpp>
#include "Object.hpp"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class SceneInformation{

private:

	int numberOfObjects;

	vector<Object> objectList;

	string sceneType;

	/*
	Information about the reference object (desk)
	*/
	pcl::PointXYZ referenceCentroid;
	float referenceLength;
	float referenceWidth;

	string sceneFold;
	string dateString;

public:

	SceneInformation();

	void addObject(Object &);

	void setType(string);
	void setReferenceLength(float);
	void setReferenceWidth(float);
	void setReferenceCentroid();
	void setSceneFold(string i) {sceneFold = i;}
	void setSceneDateString(string i) {dateString = i; }

	vector<Object> getObjectList();
	string getType();
	float getReferenceLength();
	float getReferenceWidth();
	pcl::PointXYZ getReferenceCentroid();
	int getNumberOfObjects() { return numberOfObjects; }
	string getSceneFold() {return sceneFold; }
	string getSceneDateString() { return dateString; }

	void showSceneInformation();

	vector<int> getObjectIds();

};

#endif /* SCENEINFORMATION_HPP_ */
