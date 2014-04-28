/*
 * ApiFeatureExtractionDatabaseObjectPair.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: marina
 */


#include "ApiFeatureExtractionDatabaseObjectPair.hpp"

/*
 * @description:
 * Static function that handles the extraction of pairwise features of
 * object pairs, from a set of scenes ('SceneInformation') in
 * a database ('DatabaseInformation')
 */
void ApiFeatureExtractionDatabaseObjectPair::extract(DatabaseInformation & database, DatabaseObjectPairFeature & out) {

	vector<SceneInformation> sceneList = database.getSceneList();

	// for each scene in the database
	for (vector<SceneInformation>::iterator it = sceneList.begin(); it != sceneList.end(); it++ ) {

			SceneObjectPairFeature currentFeature;
			ApiFeatureExtractionSceneObjectPair::extract( *it, currentFeature);
			out.addSceneObjectPairFeature(currentFeature);
	}

	int numberOfCategories = database.getNumberOfCategories();
	out.setNumberOfCategories(numberOfCategories);

}





