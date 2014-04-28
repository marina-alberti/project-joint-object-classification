/*
 * ApiFeatureExtractionSceneSingleObject.cpp
 *
 *  Created on: Nov 18, 2013
 *      Author: marina
 */


#include "ApiFeatureExtractionSceneSingleObject.hpp"

/*
 * @description:
 * Static function that handles the extraction of features of
 * individual objects, from a scene ('SceneInformation').
 */
void ApiFeatureExtractionSceneSingleObject::extract(SceneInformation & scene, SceneSingleObjectFeature & out) {

	vector<Object> objectList = scene.getObjectList();
	pcl::PointXYZ centroid = scene.getReferenceCentroid();

	// for each object in the scene
	for (vector<Object>::iterator it = objectList.begin(); it != objectList.end(); it++ ) {

		SingleObjectFeature currentObjectFeature;
		ApiFeatureExtractionSingleObject fe;
		fe.extractFeatures( *it, centroid, currentObjectFeature);
		out.addSingleObjectFeature(currentObjectFeature);

    }
}


/*
 * @description:
 * Static function that handles the extraction of features of
 * individual objects, from a scene ('SceneInformation'),
 * when no information is available about a reference object
 * in the scene.
 */
void ApiFeatureExtractionSceneSingleObject::extractNoReference(SceneInformation & scene, SceneSingleObjectFeature & out) {

	vector<Object> objectList = scene.getObjectList();

	// for each object in the scene
	for (vector<Object>::iterator it = objectList.begin(); it != objectList.end(); it++ ) {

		SingleObjectFeature currentObjectFeature;
		ApiFeatureExtractionSingleObject fe;
		fe.extractFeaturesNoReference( *it,  currentObjectFeature);
		out.addSingleObjectFeature(currentObjectFeature);

    }
}
