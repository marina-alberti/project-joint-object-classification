/*
 * test2.cpp
 *
 *  Created on: Dec 6, 2013
 *      Author: marina
 */

#include <stdio.h>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include "utils.hpp"
#include "SceneInformation.hpp"
#include "ApiConvertKTHDB.hpp"
#include "DatabaseInformation.hpp"
#include "ApiFeatureExtractionDatabaseSingleObject.hpp"
#include "ApiFeatureExtractionDatabaseObjectPair.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "ArrangeFeatureTraining.hpp"
#include "DatabaseSingleObjectFeature.hpp"
#include "DatabaseObjectPairFeature.hpp"
#include "Training.hpp"
#include "ModelTrainedIO.hpp"
#include "ArrangeFeatureTestScene.hpp"
#include "Test.hpp"
#include "ApiStatisticsDatabase.hpp"

#define DEBUG 1

using namespace std;

int main() {

	// Reads the data annotation in XML files into IDS
	string dir = "./data/data_more_objects/";
	vector<string> listXMLfiles =  storeFileNames(dir);
	DatabaseInformation db;
	db.loadAnnotations_KTH(listXMLfiles);

	cout << "the size of the database is: " << db.getNumberOfScenes() << endl;
	db.printSceneInformation();

	// feature extraction

	DatabaseSingleObjectFeature dbSof;
	ApiFeatureExtractionDatabaseSingleObject::extract(db, dbSof);
	DatabaseObjectPairFeature dbOpf;
	ApiFeatureExtractionDatabaseObjectPair::extract(db, dbOpf);

	// arrange the features

	vector<vector<vector<float> > > FMSingleObject;
	ArrangeFeatureTraining::setFeatureMatrixSingleObject(dbSof, FMSingleObject);
	vector<vector<vector<vector<float> > > > FMObjectPair;
	ArrangeFeatureTraining::setFeatureMatrixObjectPair(dbOpf, FMObjectPair);

	// print
	cout << "size of feature matrix is: " <<  FMSingleObject.size() << endl;
	cout << "size of feature matrix dim 2 is: " <<  FMSingleObject.at(0).size() << endl;
	cout << "size of feature matrix dim 3 is: " << FMSingleObject.at(0).at(0).size() << endl;

	ArrangeFeatureTraining::printFeatureMatrixSingleObject(FMSingleObject);
	ArrangeFeatureTraining::printFeatureMatrixObjectPair(FMObjectPair);

	// Learning

	int nclusters = 2;
	int normalizationOption = 1;
	Training doTraining;
	doTraining.learnGMMSingleObjectFeature(FMSingleObject, nclusters, normalizationOption);
	doTraining.learnGMMObjectPairFeature(FMObjectPair, nclusters, normalizationOption);

	string folder = "params";
	ModelTrainedIO::storeTrainingToFile(doTraining, folder);

	// compute object frequencies and co-occurrence

	vector<double> frequenciesSingleObject = ApiStatisticsDatabase::computeFrequenciesSingleObject(db);
	vector<vector<double> > frequenciesObjectPair = ApiStatisticsDatabase::computeFrequenciesObjectPair(db);

	// Test

	Test testingScene;
	string paramsfolder = "params";

	cout << "Going to load the files " << endl;

	ModelTrainedIO::loadTrainedGMMsFile(paramsfolder, testingScene);

	return 0;

}


