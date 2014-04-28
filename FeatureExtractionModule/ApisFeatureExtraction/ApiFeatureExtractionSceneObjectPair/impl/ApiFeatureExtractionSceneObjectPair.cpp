/*
 * ApiFeatureExtractionSceneObjectPair.cpp
 *
 *  Created on: Nov 22, 2013
 *      Author: marina
 */

#include "ApiFeatureExtractionSceneObjectPair.hpp"

#define DEBUG 1

/*
 * @description:
 * Static function that handles the extraction of pairwise spatial
 * features of object pairs, from a scene ('SceneInformation').
 */
void ApiFeatureExtractionSceneObjectPair::extract(SceneInformation & scene, SceneObjectPairFeature & out) {

	vector<Object> objectList = scene.getObjectList();

	// for each object in the scene as the reference
	for (vector<Object>::iterator it = objectList.begin(); it != objectList.end(); it++ ) {

		// for each other object in the scene as target
		for(vector<Object>::iterator it2 = objectList.begin(); it2 != objectList.end(); ++it2) {

			 if (it2 != it) {

				ObjectPairFeature currentObjectFeature;
				ApiFeatureExtractionObjectPair fe;
				fe.extractFeatures( *it, *it2, currentObjectFeature);
				out.addObjectPairFeature(currentObjectFeature);
			 }
		}
    }
}

