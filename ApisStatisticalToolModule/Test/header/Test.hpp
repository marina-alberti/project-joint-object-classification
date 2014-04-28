/*
 * Test.hpp
 *
 *  Created on: Nov 25, 2013
 *      Author: marina
 */

#ifndef TEST_HPP_
#define TEST_HPP_

#include "ArrangeFeatureTestScene.hpp"
#include "Training.hpp"
#include "utils.hpp"
#include "ApiGraph.hpp"
#include "ros/ros.h"

using namespace std;

typedef pair<int, int> idCategoryPair;
typedef pair<idCategoryPair, double> pairScore;
typedef map<int, double> mapCategoryConfidence;

class Test {

private:


	// SingleObject
	vector<vector<double> > meanNormalizationSingleObject;
	vector<vector<double> > stdNormalizationSingleObject;
	vector<vector<double> > minFeatSingleObject;
	vector<vector<double> > maxFeatSingleObject;

	// ObjectPair
	vector<vector<vector<double> > > meanNormalizationObjectPair;
	vector<vector<vector<double> > > stdNormalizationObjectPair;
	vector<vector<vector<double> > > minFeatObjectPair;
	vector<vector<vector<double> > > maxFeatObjectPair;

	// The EM output parameters: means, co-variance matrices, and weights of the mixture components

	//  <nObjectCategories x featDim x nClusters>
	vector<cv::Mat> meansSingleObject;

	//  <nObjectCategories x nClusters>
	vector<cv::Mat> weightsSingleObject;

	// <nObjectCategories x nClusters x featDim x featDim>
	vector< vector<cv::Mat> > covsSingleObject;

	vector<vector<cv::Mat> > meansObjectPair;
	vector<vector<cv::Mat> > weightsObjectPair;
	vector<vector<vector<cv::Mat> > > covsObjectPair;

	vector<double> thresholdsSingleObject;

	// The occurrence and co-occurrence frequency of object categories
	vector<double> frequencySingleObject;
	vector<vector<double> > frequencyObjectPair;

public:

	// Functions for loading the data from the training / learning on the training database
	void loadTrainedGMMs(Training & trainedParams);
	void loadLearnedObjectCategoryFrequency(vector<double> frequencySingleObjectin, vector<vector<double> > frequencyObjectPairin );

	path predictObjectClassesOnlySOF(ArrangeFeatureTestScene &, int normalization);
	path voting(ArrangeFeatureTestScene & testfeatures, int normalization, vector<vector<double> > & votingTable);
	path exhaustiveSearch(ArrangeFeatureTestScene & testfeatures, int normalization, vector<path> allPaths);
	path exhaustiveSearchAfterVoting(ArrangeFeatureTestScene & testfeatures, vector<vector<pairScore> > votingTable, int normalization);
	path optimizationGreedy(ArrangeFeatureTestScene & testfeatures, vector<vector<pairScore> > votingTable, int normalization);
	vector<vector<pairScore> > prepareVotingTableOptimizationSOFbasedScores(ArrangeFeatureTestScene &, int normalization, vector<int> categoryList);
	double computeScoreClique(ArrangeFeatureTestScene & testfeatures, int normalization, path myPath);
	double computeScoreCliqueProduct(ArrangeFeatureTestScene & testfeatures, int normalization, path myPath);
	double computeScoreCliqueVotingStrategy(ArrangeFeatureTestScene & testfeatures, int normalization, path myPath);

	static SingleObjectFeature findSOF(ArrangeFeatureTestScene & testfeatures, int currentInstanceID);
	static ObjectPairFeature findOPF(ArrangeFeatureTestScene & testfeatures, int currentInstanceID, int targetInstanceID);
	static idCategoryPair findMaximumScoreVotingTable(vector<vector<pairScore> > votingTable);
	static vector<vector<pairScore> > createVotingTableComplete(vector<vector<double> > votingTableOld, ArrangeFeatureTestScene features );
	static vector<vector<pairScore> > createShortlistedVotingTable(vector<vector<pairScore> > inputVotingTable);
	static void printPath(path);

	// Set functions for the private data members
	void setmeanNormalizationSingleObject(vector<vector<double> > in ) { meanNormalizationSingleObject = in; }
	void setstdNormalizationSingleObject(vector<vector<double> > in ) { stdNormalizationSingleObject = in; }
	void setminFeatSingleObject(vector<vector<double> > in ) { minFeatSingleObject = in; }
	void setmaxFeatSingleObject(vector<vector<double> > in ) { maxFeatSingleObject = in; }
	void setmeanNormalizationObjectPair(vector<vector<vector<double> > >  in ) { meanNormalizationObjectPair = in; }
	void setstdNormalizationObjectPair(vector<vector<vector<double> > >  in ) { stdNormalizationObjectPair = in; }
	void setminFeatObjectPair(vector<vector<vector<double> > >  in ) { minFeatObjectPair = in; }
	void setmaxFeatObjectPair(vector<vector<vector<double> > > in ) { maxFeatObjectPair = in; }
	void setmeansSingleObject( vector<cv::Mat> in ) { meansSingleObject = in ; }
    void setweightsSingleObject (vector<cv::Mat> in ) { weightsSingleObject = in;}
    void setcovsSingleObject(vector<vector<cv::Mat> > in) {covsSingleObject = in; }
    void setmeansObjectPair(vector<vector<cv::Mat> > in) {meansObjectPair = in; }
    void setweightsObjectPair(vector<vector<cv::Mat> > in ) {weightsObjectPair = in; }
    void setcovsObjectPair(vector<vector<vector<cv::Mat> > > in) {covsObjectPair = in;}
    void setThresholdSingleObject(vector<double> in) { thresholdsSingleObject = in; }

	void printmeanNormalizationSingleObject();
	void printmeanNormalizationObjectPair();

	void setfrequencySingleObject(vector<double> in) {frequencySingleObject = in; }
	void setfrequencyObjectPair(vector<vector<double> > in) {frequencyObjectPair = in; }

};


#endif /* TEST_HPP_ */
