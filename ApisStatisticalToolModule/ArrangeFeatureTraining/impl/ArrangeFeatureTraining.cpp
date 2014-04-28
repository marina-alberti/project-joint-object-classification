/*
 * ArrangeFeatureTrianing.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: marina
 */

#include "ArrangeFeatureTraining.hpp"

#define TESTFLAG 0
#define DEBUG 0

/*
 * @description:
 * Rearranges the features extracted from 'individual objects'
 * in a set of scenes contained in a training database
 * ('DatabaseSingleObjectFeature').
 * The features are organized in a data structure ('FMSingleObject',
 * <nObjectCategories X nExamples X featDim> ) that will be used
 * in the learning phase, to call the functions of the statistical tool. *
 */
void ArrangeFeatureTraining::setFeatureMatrixSingleObject(DatabaseSingleObjectFeature & featuredb, vector<vector<vector<float> > > & FMSingleObject ) {

	int numberOfCategories = featuredb.getNumberOfCategories();

	  // Iterates over set of object class IDs, predefined
	  for (int i = 0; i < numberOfCategories ; i++) {

	    if (DEBUG) {
	      cout << "Object class ID: i =  " << i << endl;
	    }

	    vector<SceneSingleObjectFeature> listSceneSingleObjectFeature =  featuredb.getListSceneSingleObjectFeature();
	    int countScene = 0;
	    int nScene = listSceneSingleObjectFeature.size();
	    vector<vector<float> > currentSceneSingleObjectFeature;

	    // Iterates over all scenes in the database
	    for(vector<SceneSingleObjectFeature>::iterator it = listSceneSingleObjectFeature.begin(); it != listSceneSingleObjectFeature.end(); ++it) {

	      vector<SingleObjectFeature> allFeatureCurrentObject = (*it).getListSingleObjectFeature();

	      // Iterates over all sets of features (from different objects) in the current scene
	      for(vector<SingleObjectFeature>::iterator it2 = allFeatureCurrentObject.begin(); it2 != allFeatureCurrentObject.end(); ++it2) {

	        int currentID  = (*it2).getObjectID();

	        // If the set of features is from the currently considered object class category
	        if ( currentID == i) {

	          vector<float> currentSingleObjectFeature = (*it2).getAllFeatures();
	          currentSceneSingleObjectFeature.push_back(currentSingleObjectFeature);
	          if (DEBUG) {
	        	    cout << "In setFeatureMatrixSingleObject: added set of features " << endl;
	        	    cout << "size is : " << currentSingleObjectFeature.size() << endl;
			  }

	        }
	      }
	      countScene++;
	    }
	    FMSingleObject.push_back(currentSceneSingleObjectFeature);
	  }

}

/*
 * @description:
 * Rearranges the features extracted from 'object pairs'
 * in a set of scenes contained in a training database
 * ('DatabaseObjectPairFeature').
 * The features are organized in a data structure ('FMPairObject',
 * <nObjectCategories X nObjectCategories X nExamples X featDim> ) that will be used
 * in the learning phase, to call the functions of the statistical tool. *
 */
void ArrangeFeatureTraining::setFeatureMatrixObjectPair(DatabaseObjectPairFeature & featuredb, vector<vector<vector<vector<float> > > > & FMPairObject) {

	int numberOfCategories = featuredb.getNumberOfCategories();

	  // Iterates over set of object class IDs, predefined
	  for (int i = 0; i < numberOfCategories ; i++) {

	    vector<vector<vector<float > > > vectorFeaturesObject1;

	    // Iterates over set of object class IDs, predefined
	    for (int j = 0; j < numberOfCategories ; j++) {

	        if (DEBUG) {
	          cout << "In setFeatureMatrix : object class IDs: i =  " << i << " and j = " << j << endl;
	        }

	        vector<vector<float> > vectorFeaturesObject2;
	        vector<SceneObjectPairFeature> listSceneObjectPairFeature = featuredb.getListSceneObjectPairFeature();
	        int countScene = 0;

	        // Iterates over all scenes
	        for(vector<SceneObjectPairFeature>::iterator it = listSceneObjectPairFeature.begin(); it != listSceneObjectPairFeature.end(); ++it) {

	        	vector<ObjectPairFeature> listObjectPairFeature = (*it).getListObjectPairFeature();

	        	// Iterate over all sets of features (from different object pairs) in the current scene
	        	for(vector<ObjectPairFeature>::iterator it2 = listObjectPairFeature.begin(); it2 != listObjectPairFeature.end(); ++it2) {

	        		// if the current set of features is from the considered object pair classes
	        		if ( ((*it2).getObjectID1() == i ) && ( (*it2).getObjectID2() == j ) ) {

	        			vector<float> featList = (*it2).getAllFeatures();
	        			vectorFeaturesObject2.push_back(featList);
	        		}
	        	}
	        	countScene++;
	        }
	        vectorFeaturesObject1.push_back(vectorFeaturesObject2);
	    }
	    FMPairObject.push_back(vectorFeaturesObject1);
	  }
}


void ArrangeFeatureTraining::printFeatureMatrixSingleObject(vector<vector<vector<float> > > & FMSingleObject) {

	for (int i = 0; i < FMSingleObject.size(); i++) {

		cout << "Category  " << i << endl;
		for (int j = 0; j < FMSingleObject.at(i).size(); j++) {
			for (int z = 0; z < FMSingleObject.at(i).at(j).size(); z++) {
				cout << FMSingleObject.at(i).at(j).at(z) << " " ;
			}
			cout << endl;

		}
	}
}

void ArrangeFeatureTraining::printFeatureMatrixObjectPair(vector<vector<vector<vector<float> > > > & FMPairObject) {

	for (int i = 0; i < FMPairObject.size(); i++) {
		for (int i2 = 0; i2 < FMPairObject.at(i).size(); i2++) {
			for (int j = 0; j < FMPairObject.at(i).at(i2).size(); j++) {
				for (int z = 0; z < FMPairObject.at(i).at(i2).at(j).size(); z++) {
					cout << FMPairObject.at(i).at(i2).at(j).at(z) << " " ;
				}
				cout << endl;
			}
		}
	}
}

void ArrangeFeatureTraining::printFeatureSingleObjectToFile(vector<vector<vector<float> > > & FMSingleObject, string fileName) {

	ofstream myfile;
	string filepath = fileName;
	const char * filepathC = filepath.c_str();
	myfile.open (filepathC);

	for (int i = 0; i < FMSingleObject.size(); i++) {

		// for each object class category
		for (int j = 0; j < FMSingleObject.at(i).size(); j++) {

			for (int z = 0; z < FMSingleObject.at(i).at(j).size(); z++) {
				myfile << FMSingleObject.at(i).at(j).at(z) << ", " ;
			}
			myfile << i ;
			myfile << endl;
		}
	}
	myfile.close();

}


void ArrangeFeatureTraining::printFeatureObjectPairToFile(vector<vector<vector<vector<float> > > > & FMPairObject, string fileName) {

	ofstream myfile;
	string filepath = fileName;
	const char * filepathC = filepath.c_str();
	myfile.open (filepathC);

	for (int i = 0; i < FMPairObject.size(); i++) {

		for (int i2 = 0; i2 < FMPairObject.at(i).size(); i2++) {

			for (int j = 0; j < FMPairObject.at(i).at(i2).size(); j++) {

				for (int z = 0; z < FMPairObject.at(i).at(i2).at(j).size(); z++) {

					myfile << FMPairObject.at(i).at(i2).at(j).at(z) << ", " ;
				}
				myfile << i << ", " << i2 ;
				myfile << endl;
			}
		}
	}
	myfile.close();
}

