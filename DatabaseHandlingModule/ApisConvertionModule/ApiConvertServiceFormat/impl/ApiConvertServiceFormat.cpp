/*
 * ApiConvertServiceFormat.cpp
 *
 *  Created on: Jan 9, 2014
 *      Author: marina
 */


#include "ApiConvertServiceFormat.hpp"


/*
 * @description:
 * This function stores the information of each of the 3D objects of a scene
 * into the Internal Data Structure ('SceneInformation').
 */
void ApiConvertServiceFormat::parseScene(vector<string> objectList, vector<vector<pcl::PointXYZ> > bboxList, vector<pcl::PointXYZ> poseList, SceneInformation & currentScene){

	int counter = 0;

	for (vector<string>::iterator it = objectList.begin(); it != objectList.end(); ++it) {
        Object newObject;
        newObject.setInstanceName(*it);
        newObject.setInstanceID(counter);
        newObject.setCentroidPoint(poseList.at(counter));
        newObject.setBoundingBox(bboxList.at(counter));
        currentScene.addObject(newObject);
        counter++;
    }
}


