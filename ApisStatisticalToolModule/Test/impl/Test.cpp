/*
 * Test.cpp
 *
 *  Created on: Nov 25, 2013
 *      Author: marina
 */

#include "Test.hpp"

#define TESTFLAG 0


path Test::predictObjectClassesOnlySOF(ArrangeFeatureTestScene & testfeatures, int normalization) {

	vector<SingleObjectFeature> listSOF = testfeatures.getListSOF();
	vector<vector<double> > vectorProbabilitiesObjects;

	// For each object instance in the object List of the scene
	for (int i  = 0; i < listSOF.size(); i++ ) {

		if (TESTFLAG) {
	        cout << std::endl << "Predict object class for unknown object in the object list : " << i << endl;
		}
		// Gets the 'single object features' (SOF) for the current object
		vector<float> features = listSOF.at(i).getAllFeatures();

	    // vectorProb: a vector containing a likelihood value for each of the considered object categories
	    vector<double> vectorProb;
	    double maxProbValue;
	    int maxProbClassIndex;
	    int countModels = 0;

	    if (DEBUG) {
	    	cout << " Actual class is:  " << listSOF.at(i).getObjectID() << endl;
	    }

	    for(int countCategory = 0; countCategory < meansSingleObject.size(); countCategory++  ) {

	    	bool outlier = false;

	    	// Extracts mean, cov, and weight coefficients for current GMM model
	    	cv::Mat _means = meansSingleObject.at(countModels); 				//  <dims x nclusters>
	    	cv::Mat _weights = weightsSingleObject.at(countModels);  			//  <nclusters x 1>
	    	vector<cv::Mat> _covs = covsSingleObject.at(countModels);      		//  <nclusters x dims x dims>

	    	if (DEBUG) {
	    		cout << endl << "The mean matrix of current GMM model is : "  << endl << _means << endl;
	        	cout << "The weights of current GMM model are : "  << _weights << endl;
	        	cout << "The covs of current GMM model are : "  << _covs.at(0) << endl;
	    	}

			// Feature matrix normalization
	    	vector<float> normalizedFeatMat;

			if (normalization == 1) {
				vector<double> meansVector = meanNormalizationSingleObject.at(countModels);
				vector<double> stdVector = stdNormalizationSingleObject.at(countModels);
				normalizedFeatMat = StatisticalTool::doNormalizationFeatureVector(features, meansVector, stdVector);
			}
			else if (normalization == 2) {				vector<double> maxVector = maxFeatSingleObject.at(countModels);
				vector<double> minVector = minFeatSingleObject.at(countModels);
				normalizedFeatMat = StatisticalTool::doNormalizationMinMaxFeatureVector(features, maxVector, minVector);
			}
			else if (normalization == 0) {
				normalizedFeatMat = features;
			}

	    	// Computes probability / likelihood value for the current GMM model
	    	double prob = StatisticalTool::computeGMMProbability(features, _means, _covs, _weights );

	    	// Computes a-priori probability of object classes in terms of frequency of appearance in the training database
	    	double currentObjectCategoryFreq = frequencySingleObject[countModels];

	    	// Computes the a-posterior probability: product of a-priori and likelihood
	    	double probPost = (prob) * (currentObjectCategoryFreq);

	    	if (DEBUG) {
	    		cout << " aposteriori for Object class : ";
	    		cout << countModels << "  is  =  " << probPost;
	    		cout << endl << "  with object category frequency  =   " << (currentObjectCategoryFreq) << endl;
	    	}

	    	if ( countModels == 0) {
	    		maxProbValue = -100000;
	    		maxProbClassIndex = -1;
	    	}

	    	if ( (probPost > maxProbValue) && (outlier == false)  ) {
	    		maxProbValue = probPost;
	    		maxProbClassIndex = countModels;
	    	}
	    	// Push back probability of current GMM model into vector of probabilities for current object
	    	vectorProb.push_back(probPost);

	    	if (DEBUG) {
	    		cout << " Compute likelihood for object category type : " << countModels
	    				<< "   likelihood = " << probPost << endl;	    	}
	    	countModels++;
	    }
    	if (TESTFLAG) {
    		cout << "The actual class is " << listSOF.at(i).getObjectID()  << endl;
    		cout << "The maximum probability class is " << maxProbClassIndex << endl << endl;
    	}
	    vectorProbabilitiesObjects.push_back(vectorProb);
	}
	path outputPath;

	// for each test object
	for (int i = 0; i < vectorProbabilitiesObjects.size(); i++) {
		vector<double> scoresCurrentTestObject = vectorProbabilitiesObjects.at(i);
		int maximumCategoryLabel = computeMaximum(scoresCurrentTestObject);
		int objectInstanceId =  listSOF.at(i).getInstanceID();
		outputPath[objectInstanceId] = maximumCategoryLabel;
	}
	printPath(outputPath);
	return outputPath;
}



path Test::voting(ArrangeFeatureTestScene & testfeatures, int normalization, vector<vector<double> > & votingTable ) {

	vector<vector<ObjectPairFeature> > matrixOPF = testfeatures.getMatrixOPF();
	vector<SingleObjectFeature> listSOF = testfeatures.getListSOF();

	for (int c = 0; c < listSOF.size(); c++) {
		vector <double> temp;
		for (int cc = 0; cc < meansSingleObject.size(); cc++) {
			temp.push_back(0);
		}
		votingTable.push_back(temp);
	}

	if (TESTFLAG) {
		cout << "inside voting 1" << endl;
	}

	// for each pair of test objects
	for (int p = 0; p < listSOF.size(); p++) {
		for (int q = 0; q < listSOF.size(); q++) {

			if (p != q) {

				SingleObjectFeature refSOF = listSOF.at(p);
				SingleObjectFeature targetSOF = listSOF.at(q);
				ObjectPairFeature opf = matrixOPF.at(p).at(q);
				vector<float> reffeatures = refSOF.getAllFeatures();
				vector<float> targetfeatures = targetSOF.getAllFeatures();
				vector<float> pairfeatures = opf.getAllFeatures();

				// // for each possible combination of object category labels for the considered test object
				for (int i = 0; i < meansSingleObject.size(); i++) {
					for (int j = 0; j < meansSingleObject.size(); j++) {

						// THE REFERENCE
						cv::Mat refmeans = meansSingleObject.at(i); 				//  dims x nclusters
						cv::Mat refweights = weightsSingleObject.at(i);  			//  nclusters x 1
						vector<cv::Mat> refcovs = covsSingleObject.at(i);      		//  nclusters x dims x dims

				    	vector<float> refnormalizedFeatMat;
						if (normalization == 1) {
							vector<double> meansVector = meanNormalizationSingleObject.at(i);
							vector<double> stdVector = stdNormalizationSingleObject.at(i);
							refnormalizedFeatMat = StatisticalTool::doNormalizationFeatureVector(reffeatures, meansVector, stdVector);
						}
						else if (normalization == 2) {
							vector<double> maxVector = maxFeatSingleObject.at(i);
							vector<double> minVector = minFeatSingleObject.at(i);
							refnormalizedFeatMat = StatisticalTool::doNormalizationMinMaxFeatureVector(reffeatures, maxVector, minVector);
						}
						else if (normalization == 0) {
							refnormalizedFeatMat = reffeatures;
						}

						double refprob = StatisticalTool::computeGMMProbability(reffeatures, refmeans, refcovs, refweights );

						// Computes a-priori probability of object classes in terms of frequency of appearance in the training database
						double refObjectCategoryFreq = frequencySingleObject[i];
						double refprobPosterior = refprob * refObjectCategoryFreq;

						// THE TARGET
						cv::Mat targetmeans = meansSingleObject.at(j); 					//  dims x nclusters
						cv::Mat targetweights = weightsSingleObject.at(j);  			//  nclusters x 1
						vector<cv::Mat> targetcovs = covsSingleObject.at(j);      		//  nclusters x dims x dims

						vector<float> targetnormalizedFeatMat;
						if (normalization == 1) {

							vector<double> meansVector = meanNormalizationSingleObject.at(j);
							vector<double> stdVector = stdNormalizationSingleObject.at(j);
							targetnormalizedFeatMat = StatisticalTool::doNormalizationFeatureVector(targetfeatures, meansVector, stdVector);
						}
						else if (normalization == 2) {

							vector<double> maxVector = maxFeatSingleObject.at(j);
							vector<double> minVector = minFeatSingleObject.at(j);
							targetnormalizedFeatMat = StatisticalTool::doNormalizationMinMaxFeatureVector(targetfeatures, maxVector, minVector);
						}
						else if (normalization == 0) {
							targetnormalizedFeatMat = targetfeatures;
						}
						double targetprob = StatisticalTool::computeGMMProbability(targetfeatures, targetmeans, targetcovs, targetweights );

						double targetObjectCategoryFreq = frequencySingleObject[j];

						double  targetprobPosterior =  targetprob *  targetObjectCategoryFreq;

						// THE PAIR RELATION
						cv::Mat pairmeans = meansObjectPair.at(i).at(j); 					//  dims x nclusters
						cv::Mat pairweights = weightsObjectPair.at(i).at(j);  				//  nclusters x 1
						vector<cv::Mat> paircovs = covsObjectPair.at(i).at(j);      		//  nclusters x dims x dims

						if (DEBUG) {
							cout << "In setting voting table 0  " << i << "  " << j << endl;
						}

						vector<float> pairnormalizedFeatMat;
						if (normalization == 1) {

							vector<double> meansVector = meanNormalizationObjectPair.at(i).at(j);
							vector<double> stdVector = stdNormalizationObjectPair.at(i).at(j);
							pairnormalizedFeatMat = StatisticalTool::doNormalizationFeatureVector(pairfeatures, meansVector, stdVector);
						}
						else if (normalization == 2) {

							vector<double> maxVector = maxFeatObjectPair.at(i).at(j);
							vector<double> minVector = minFeatObjectPair.at(i).at(j);
							pairnormalizedFeatMat = StatisticalTool::doNormalizationMinMaxFeatureVector(pairfeatures, maxVector, minVector);
						}
						else if (normalization == 0) {
							pairnormalizedFeatMat = pairfeatures;
						}
						if (DEBUG) {
							cout << "In setting voting table b  " << i << "  " << j << endl;
							cout << "means" << endl << pairmeans << endl;
							cout << "weights" << endl << pairweights << endl;
						}

						double pairprob = StatisticalTool::computeGMMProbability(pairfeatures, pairmeans, paircovs, pairweights );
						double  pairObjectCategoryFreq = frequencyObjectPair.at(i).at(j);
						double  pairprobPosterior =  pairprob  *  pairObjectCategoryFreq;
						double totalscore =  refprobPosterior * targetprobPosterior * pairprobPosterior;
						votingTable.at(p).at(i) += totalscore;
						votingTable.at(q).at(j) += totalscore;

						if (TESTFLAG) {

							cout << "target prob = " << targetprob << endl;
							cout << " ref prob = " << refprob << endl;
							cout << "pair prob = " << pairprob << endl;
						}

					}
				}
			}
		}
	}

	for (int c = 0; c < meansSingleObject.size(); c++) {
		for (int cc = 0; cc < listSOF.size(); cc++) {
			cout << votingTable.at(cc).at(c) << "         " ;
		}
		cout << endl;
	}

	for (int cc = 0; cc < listSOF.size(); cc++) {
		vector<double> vectorin = votingTable.at(cc);
		cout << cc << " Actual class is:  " << listSOF.at(cc).getObjectID() << endl;
		cout << " Predicted class is:  "  << computeMaximum(vectorin) << endl;
	}

	path resultsPath;
	for (int cc = 0; cc < listSOF.size(); cc++) {
		vector<double> vectorin = votingTable.at(cc);
		int objectInstanceId = listSOF.at(cc).getInstanceID();
		int categoryLabel = computeMaximum(vectorin);
		resultsPath[objectInstanceId] = categoryLabel;
	}
	printPath(resultsPath);
	return resultsPath;
}


vector<vector<pairScore> > Test::prepareVotingTableOptimizationSOFbasedScores(ArrangeFeatureTestScene & feat, int normalization, vector<int> categoryList) {

	vector<vector<pairScore> > votingTable;
	path sofpath =  predictObjectClassesOnlySOF(feat, normalization);

	// The initialization of the voting table: all the test objects (from the path)
	// and all the object categories (from the category list) and the scores are
    //  set to zeros.
	for(map<int, int>::iterator iter = sofpath.begin(); iter != sofpath.end(); ++iter ) {
		vector<pairScore> tmp;
		int currentInstanceID = (*iter).first;

		// for all possible object category label in the category list
		for (int i = 0; i < categoryList.size(); i++) {
			int currentCategory =  categoryList.at(i);
			idCategoryPair ids;
			ids.first = currentInstanceID;
			ids.second = currentCategory;
			pairScore currentPairScore;
			currentPairScore.first = ids;
			currentPairScore.second = 0;
			tmp.push_back(currentPairScore);
		}
		votingTable.push_back(tmp);
	}


	int counttestobjects = 0;

	for(map<int, int>::iterator iter = sofpath.begin(); iter != sofpath.end(); ++iter ) {

		int currentInstanceID = (*iter).first;
		int currentCategory = (*iter).second;
		cv::Mat means = meansSingleObject.at(currentCategory);
		cv::Mat weights = weightsSingleObject.at(currentCategory);
		vector<cv::Mat> covs = covsSingleObject.at(currentCategory);
		SingleObjectFeature currentSOF = findSOF(feat, currentInstanceID);
		vector<float> soffeaturevector = currentSOF.getAllFeatures();

		double sofscore = StatisticalTool::computeGMMProbability(soffeaturevector, means, covs, weights );

		// compute a-priori probability of object classes in terms of frequency of appearance in the training database
		double objectCategoryFreq = frequencySingleObject[currentCategory];

		double score =  sofscore * objectCategoryFreq;

		idCategoryPair ids;
		ids.first = currentInstanceID;
		ids.second = currentCategory;
		pairScore currentPairScore;
		currentPairScore.first = ids;
		currentPairScore.second = score;

		int indexCategory = computeEqualPosition(categoryList, currentCategory);

		votingTable.at(counttestobjects).at(indexCategory) = currentPairScore;
		counttestobjects++;
	}
	return votingTable;
}



vector<vector<pairScore> > Test::createVotingTableComplete(vector<vector<double> > votingTableOld, ArrangeFeatureTestScene features ) {

	vector<SingleObjectFeature> listSOF = features.getListSOF();
	vector<vector<pairScore> > votingTable;

	// rows = test objects, columns = categories
	for (int i = 0; i < votingTableOld.size(); i++) {

		int objectID = listSOF.at(i).getInstanceID();
		vector<pairScore> tmpPairScoreVector;

		for (int j = 0; j < votingTableOld.at(i).size(); j++) {

			idCategoryPair currentIdCategoryPair;
			pairScore currentPairScore;
			currentIdCategoryPair.first = objectID;
			currentIdCategoryPair.second = j;
			currentPairScore.first = currentIdCategoryPair;
			currentPairScore.second = votingTableOld.at(i).at(j);
			tmpPairScoreVector.push_back(currentPairScore);

		}
		votingTable.push_back(tmpPairScoreVector);
	}
	return votingTable;
}

/*
 * @description:
 * This function finds the maximum score in the voting table:
 * finds the maximum value of the score and the corresponding row and column
 * indexes in the voting table: (objectID, categoryLabel)_maximum.
*/
idCategoryPair Test::findMaximumScoreVotingTable(vector<vector<pairScore> > votingTable) {

	double currentMaximum = votingTable.at(0).at(0).second;
	idCategoryPair currentMaximumIndexes = votingTable.at(0).at(0).first;

	for (int i = 0; i < votingTable.size(); i++) {
		for (int j = 0; j < votingTable.at(i).size(); j++) {
			double tableElement = votingTable.at(i).at(j).second;
			if (tableElement > currentMaximum) {
				currentMaximum = tableElement;
				currentMaximumIndexes = votingTable.at(i).at(j).first;
			}
		}
	}
	return currentMaximumIndexes;
}


/*
 * @description:
 * for each test object, finds the shortlist of possible object categories.
 * The shortlist contains: (categoryLabel, score) for each element in this shortlist.
 */
vector<vector<pairScore> > Test::createShortlistedVotingTable(vector<vector<pairScore> > inputVotingTable) {

	vector<vector<pairScore> > outputVotingTable;

	// This threshold on the score values is used to decide either to include
	// the pair: (objectID categoryLabel) into the shortlist of possible object
	// categories for the current objectID.
	double thresholdScore = 0;

	// for each test object
	for (int i = 0; i < inputVotingTable.size(); i++) {

		vector<pairScore> tmpPairScoreVector;

		// for each categoryLabel previously assigned to the considered test object
		for (int j = 0; j < inputVotingTable.at(i).size(); j++) {

			// the score for the considered pair (objectid, categoryLabel)
			double tableElementScore = inputVotingTable.at(i).at(j).second;

			// if the score is above the threshold
			if (tableElementScore > thresholdScore) {

				// add the element to the new voting table
				tmpPairScoreVector.push_back(inputVotingTable.at(i).at(j));
			}
		}
		outputVotingTable.push_back(tmpPairScoreVector);
	}
	return outputVotingTable;
}



path Test::optimizationGreedy(ArrangeFeatureTestScene & testfeatures, vector<vector<pairScore> > votingTable, int normalization) {

	path optimizedPath;
	map<int, bool> nontraversed;
	for (int i = 0; i < votingTable.size(); i++) {
		int objectID = votingTable.at(i).at(0).first.first;
		nontraversed[objectID] = true;
	}

	idCategoryPair startIdCategoryPair = findMaximumScoreVotingTable(votingTable);
	optimizedPath[startIdCategoryPair.first] = startIdCategoryPair.second;

	// erase map by key
	nontraversed.erase(startIdCategoryPair.first);

	if (TESTFLAG) {
		cout << "the size of the map non traversed is now  " << nontraversed.size() << endl;
		cout << "The first optimum pair (objectID, catLabel) is :: " << startIdCategoryPair.first << "  " << startIdCategoryPair.second << endl;
	}

	// while there are still nodes that have not been traversed
	while (nontraversed.size() > 0) {

		if (DEBUG) {
			cout << "new loop in the whole loop " << endl;
		}

		// data structure for storing the scores of the clique/path formed by
		// adding a pair (objectID, categoryLabel) to the previous optimized path
		map<idCategoryPair, double> cliqueScores;

		// For all test object
		for(int i = 0; i < votingTable.size(); i++) {

			int currentobjectID = votingTable.at(i).at(0).first.first;

			// if the test object is not traversed
			if ( nontraversed.find(currentobjectID) != nontraversed.end() ) {

				// for all categoryLabel in the short list of that test object:
				for (int j = 0; j < votingTable.at(i).size(); j++) {

					path tempPath = optimizedPath;
					int currentcategoryLabel = votingTable.at(i).at(j).first.second;
					tempPath[currentobjectID] = currentcategoryLabel;
					if (DEBUG) {
						cout << endl << endl << "inside searching the best next node before score compute:: test object = " << currentobjectID << " cat = " << currentcategoryLabel << endl;
					}

					// computes the cost of the new total path
					double currentPathScore = computeScoreCliqueVotingStrategy(testfeatures, normalization, tempPath);

					// stores the cost in an ad-hoc data structure
					idCategoryPair currentidCategoryPair = votingTable.at(i).at(j).first;
					cliqueScores[currentidCategoryPair] = currentPathScore;
				}
			}
		}

		// Finds the maximum of the scores from the list of scores,
		// and retrieves the corresponding next best node = optimum node (objectID, categoryLabel)
		map<idCategoryPair, double>::iterator startiter = cliqueScores.begin();
		double tmpMaximum = (*startiter).second;
		idCategoryPair tmpMaximumIdCategoryPair = (*startiter).first;

		for( map<idCategoryPair, double>::iterator iter=cliqueScores.begin(); iter!=cliqueScores.end(); ++iter) {
			if ( (*iter).second > tmpMaximum) {
				tmpMaximum = (*iter).second;
				tmpMaximumIdCategoryPair = (*iter).first;
			}
		}

		// Sets the objectID of next best node as TRAVERSED
		nontraversed.erase(tmpMaximumIdCategoryPair.first);

		// Adds the node to the optimized path
		optimizedPath[tmpMaximumIdCategoryPair.first] = tmpMaximumIdCategoryPair.second;

	}
	printPath(optimizedPath);

	return optimizedPath;

}



void Test::printPath(path in) {
	cout << endl;
	for( map<int,int>::iterator jj=in.begin(); jj!=in.end(); ++jj) {
		 cout << (*jj).first << " : " << (*jj).second << endl;
	}
}



path Test::exhaustiveSearch(ArrangeFeatureTestScene & testfeatures, int normalization, vector<path> allPaths) {

	if (TESTFLAG) {
		cout << "exhaustiveSearch: begin." << endl;
		cout << "size of allPaths =  " << allPaths.size() << endl;
	}

	// the output is a vector of double where each element is the score of a path - from the path list allPaths.
	vector<double> allPathScores;

	// for each path in allPaths - the list of paths
	for (int i = 0; i < allPaths.size(); i++) {

		path currentPath = allPaths.at(i);

		if (TESTFLAG) {
			cout << "exhaustiveSearch: path number" << i << endl;
		}

		double currentPathScore = computeScoreCliqueVotingStrategy(testfeatures,  normalization, currentPath);
		if (TESTFLAG) {
			cout << "exhaustiveSearch: path number" << i << "  After computing score clique " << endl;
		}

		allPathScores.push_back(currentPathScore);

		if (DEBUG) {
			if (currentPathScore > 0) {
				cout << "The path at round N. " << i << " is: " ;
				printPath(currentPath);
				cout << " Score is = " << currentPathScore << endl << endl;
			}
		}
	}

	int maximumPosition = computeMaximum(allPathScores);
	double maximumPathScore = computeMaximumValue(allPathScores);
	path maximumScorePath = allPaths.at(maximumPosition);

	if (TESTFLAG) {
		cout << "The maximum score path is::" << endl;
		printPath(maximumScorePath);
		cout << " Score is = " << maximumPathScore << endl << endl;
	}

	if (TESTFLAG) {
		cout << "exhaustiveSearch: end" << endl << endl;
	}

	return maximumScorePath;
}

path Test::exhaustiveSearchAfterVoting(ArrangeFeatureTestScene & testfeatures, vector<vector<pairScore> > votingTable, int normalization) {

	path maximumScorePath;
	ApiGraph graphPaths(votingTable);
	graphPaths.findAllPathsShortlisted();
	vector<path> allPaths = graphPaths.getAllPaths();
	maximumScorePath = exhaustiveSearch(testfeatures, normalization, allPaths);
	return maximumScorePath;
}


/*
 * @description:
 * This function coputes the score associated to a clique of the graph.
 * In a clique, we have a set of objects from the test scene and their
 * associated object category labels.
 */
double Test::computeScoreClique(ArrangeFeatureTestScene & testfeatures, int normalization, path myPath) {

	double currentPathScore = 1;
	double currentSOFScore = 0;
	double currentOPFScore = 0;

	if (DEBUG) {
		cout << "computeScoreClique: begin" << endl;
		cout << "The length of the temp path is :: " << myPath.size() << endl;
	}

	// for all the object instances in the path (which are all the unknown test objects in the test scene)
	for(map<int, int>::iterator iter = myPath.begin(); iter != myPath.end(); ++iter ) {

		int currentInstanceID = (*iter).first;
		int currentCategory = (*iter).second;
		cv::Mat means = meansSingleObject.at(currentCategory); 					//  dims x nclusters
		cv::Mat weights = weightsSingleObject.at(currentCategory);  			//  nclusters x 1
		vector<cv::Mat> covs = covsSingleObject.at(currentCategory);      		//  nclusters x dims x dims
		SingleObjectFeature currentSOF = findSOF(testfeatures, currentInstanceID);
		vector<float> soffeaturevector = currentSOF.getAllFeatures();

		double sofscore = StatisticalTool::computeGMMProbability(soffeaturevector, means, covs, weights );

		// compute a-priori probability of object classes in terms of frequency of appearance in the training database
		double objectCategoryFreq = frequencySingleObject[currentCategory];

		currentPathScore = currentPathScore * sofscore * objectCategoryFreq;

		currentSOFScore = currentSOFScore + sofscore * objectCategoryFreq;

		if (DEBUG) {
			cout << "computeScoreClique: SOF ended:: the objectID considered is " << currentInstanceID << endl;
		}

		// for all the OTHER object instances - considered as "target" object while the current object is the "reference" object
		for(map<int, int>::iterator iter2 = myPath.begin(); iter2 != myPath.end(); iter2++ ) {

			// we do not want to consider a pair where we take the same unknown test object twice
			if ((*iter2) != (*iter)) {

				int targetInstanceID = (*iter2).first;
				int targetCategory = (*iter2).second;

				if (DEBUG) {
					cout << "computeScoreClique: OPF start " ;
					cout << "the objectID considered are " << currentInstanceID << " and " << targetInstanceID << endl;
				}

				cv::Mat pairmeans = meansObjectPair.at(currentCategory).at(targetCategory);
				cv::Mat pairweights = weightsObjectPair.at(currentCategory).at(targetCategory);
				vector<cv::Mat> paircovs = covsObjectPair.at(currentCategory).at(targetCategory);
				ObjectPairFeature currentOPF = findOPF(testfeatures, currentInstanceID, targetInstanceID);
				vector<float> opffeaturevector = currentOPF.getAllFeatures();
				double opfscore =  StatisticalTool::computeGMMProbability(opffeaturevector, pairmeans, paircovs, pairweights );

				double  pairObjectCategoryFreq = frequencyObjectPair.at(currentCategory).at(targetCategory);

				currentPathScore = currentPathScore * opfscore * pairObjectCategoryFreq ;

				currentOPFScore = currentOPFScore + opfscore * pairObjectCategoryFreq ;

				if (DEBUG) {
					cout << "computeScoreClique: OPF ended " ;
					cout << "the objectID considered are " << currentInstanceID << " and " << targetInstanceID << endl;
				}
			}
		}
	}
	currentPathScore = currentOPFScore + currentSOFScore;

	return currentPathScore;
}



double Test::computeScoreCliqueProduct(ArrangeFeatureTestScene & testfeatures, int normalization, path myPath) {

	double totalPathScore = 1;
	if (DEBUG) {
		cout << "computeScoreCliqueProduct: begin" << endl;
		cout << "The length of the temp path is :: " << myPath.size() << endl;
	}

	for(map<int, int>::iterator iter = myPath.begin(); iter != myPath.end(); ++iter ) {

		int currentInstanceID = (*iter).first;
		int currentCategory = (*iter).second;
		cv::Mat means = meansSingleObject.at(currentCategory); 					//  dims x nclusters
		cv::Mat weights = weightsSingleObject.at(currentCategory);  			//  nclusters x 1
		vector<cv::Mat> covs = covsSingleObject.at(currentCategory);      		//  nclusters x dims x dims
		SingleObjectFeature currentSOF = findSOF(testfeatures, currentInstanceID);
		vector<float> soffeaturevector = currentSOF.getAllFeatures();

		double sofscore = StatisticalTool::computeGMMProbability(soffeaturevector, means, covs, weights );

		// compute a-priori probability of object classes in terms of frequency of appearance in the training database
		double objectCategoryFreq = frequencySingleObject[currentCategory];

		totalPathScore = totalPathScore * sofscore * objectCategoryFreq;

		if (DEBUG) {
			cout << "computeScoreClique: SOF ended:: the objectID considered is " << currentInstanceID << endl;
		}

	}
	// for all the PAIRS in the path, computation of the likelihoods OPF
	for(map<int, int>::iterator iter = myPath.begin(); iter != myPath.end(); ++iter ) {

		// for all the OTHER object instances - considered as "target" object while the current object is the "reference" object
		for(map<int, int>::iterator iter2 = myPath.begin(); iter2 != myPath.end(); iter2++ ) {

			// we do not want to consider a pair where we take the same unknown test object twice
			if ((*iter2) != (*iter)) {

				int refInstanceID = (*iter).first;
				int refCategory = (*iter).second;
				int targetInstanceID = (*iter2).first;
				int targetCategory = (*iter2).second;

				if (DEBUG) {
					cout << "computeScoreClique: OPF start " ;
					cout << "the objectID considered are " << refInstanceID << " and " << targetInstanceID << endl;
				}

				cv::Mat pairmeans = meansObjectPair.at(refCategory).at(targetCategory);
				cv::Mat pairweights = weightsObjectPair.at(refCategory).at(targetCategory);
				vector<cv::Mat> paircovs = covsObjectPair.at(refCategory).at(targetCategory);
				ObjectPairFeature currentOPF = findOPF(testfeatures, refInstanceID, targetInstanceID);
				vector<float> opffeaturevector = currentOPF.getAllFeatures();
				double opfscore =  StatisticalTool::computeGMMProbability(opffeaturevector, pairmeans, paircovs, pairweights );

				double  pairObjectCategoryFreq = frequencyObjectPair.at(refCategory).at(targetCategory);

				totalPathScore = totalPathScore * opfscore * pairObjectCategoryFreq ;

				if (DEBUG) {
					cout << "computeScoreClique: OPF ended " ;
					cout << "the objectID considered are " << refInstanceID << " and " << targetInstanceID << endl;
				}
			}
		}
	}
	return totalPathScore;
}



double Test::computeScoreCliqueVotingStrategy(ArrangeFeatureTestScene & testfeatures, int normalization, path myPath) {

	// The total scene score is a SUM and for this reason the total score is initialized to 0
	double totalPathScore = 0;

	if (DEBUG) {
		cout << "computeScoreCliqueProduct: begin" << endl;
		cout << "The length of the temp path is :: " << myPath.size() << endl;
	}

	// for all the PAIRS in the path, computation of the likelihoods OPF:
	// for each object instance considered as the "reference" object
	for(map<int, int>::iterator iter = myPath.begin(); iter != myPath.end(); ++iter ) {

		// for all the OTHER object instances - considered as "target" object while the current object is the "reference" object
		for(map<int, int>::iterator iter2 = myPath.begin(); iter2 != myPath.end(); iter2++ ) {

			// we do not want to consider a pair where we take the same unknown test object twice
			if ((*iter2) != (*iter)) {

				// The score of each pair is defined as a PRODUCT and for this reason, the pair score is initialized to 0
				double currentPairScore = 1;

				// PAIR OF OBJECT REFERENCE TARGET

				int refInstanceID = (*iter).first;
				int refCategory = (*iter).second;
				int targetInstanceID = (*iter2).first;
				int targetCategory = (*iter2).second;

				if (DEBUG) {
					cout << "computeScoreClique: OPF start " ;
					cout << "the objectID considered are " << refInstanceID << " and " << targetInstanceID << endl;
				}

				cv::Mat pairmeans = meansObjectPair.at(refCategory).at(targetCategory);
				cv::Mat pairweights = weightsObjectPair.at(refCategory).at(targetCategory);
				vector<cv::Mat> paircovs = covsObjectPair.at(refCategory).at(targetCategory);
				ObjectPairFeature currentOPF = findOPF(testfeatures, refInstanceID, targetInstanceID);
				vector<float> opffeaturevector = currentOPF.getAllFeatures();

				double opfscore =  StatisticalTool::computeGMMProbability(opffeaturevector, pairmeans, paircovs, pairweights );

				double  pairObjectCategoryFreq = frequencyObjectPair.at(refCategory).at(targetCategory);

				// REFERENCE OBJECT

				cv::Mat refmeans = meansSingleObject.at(refCategory); 					//  dims x nclusters
				cv::Mat refweights = weightsSingleObject.at(refCategory);  			//  nclusters x 1
				vector<cv::Mat> refcovs = covsSingleObject.at(refCategory);      		//  nclusters x dims x dims
				SingleObjectFeature refcurrentSOF = findSOF(testfeatures, refInstanceID);
				vector<float> refsoffeaturevector = refcurrentSOF.getAllFeatures();

				double refsofscore = StatisticalTool::computeGMMProbability(refsoffeaturevector, refmeans, refcovs, refweights );

				// compute a-priori probability of object classes in terms of frequency of appearance in the training database
				double refobjectCategoryFreq = frequencySingleObject[refCategory];

				// TARGET OBJECT

				cv::Mat targetmeans = meansSingleObject.at(targetCategory); 					//  dims x nclusters
				cv::Mat targetweights = weightsSingleObject.at(targetCategory);  			//  nclusters x 1
				vector<cv::Mat> targetcovs = covsSingleObject.at(targetCategory);      		//  nclusters x dims x dims
				SingleObjectFeature targetcurrentSOF = findSOF(testfeatures, targetInstanceID);
				vector<float> targetsoffeaturevector = targetcurrentSOF.getAllFeatures();

				double targetsofscore = StatisticalTool::computeGMMProbability(targetsoffeaturevector, targetmeans, targetcovs, targetweights );

				// compute a-priori probability of object classes in terms of frequency of appearance in the training database
				double targetobjectCategoryFreq = frequencySingleObject[targetCategory];
				currentPairScore = refsofscore * refobjectCategoryFreq * targetsofscore * targetobjectCategoryFreq * opfscore * pairObjectCategoryFreq;

				totalPathScore +=  currentPairScore;

				if (DEBUG) {
					cout << "computeScoreClique: OPF ended " ;
					cout << "the objectID considered are " << refInstanceID << " and " << targetInstanceID << endl;
				}
			}
		}
	}
	return totalPathScore;
}



SingleObjectFeature Test::findSOF(ArrangeFeatureTestScene & testfeatures, int currentInstanceID) {

	SingleObjectFeature features;
	vector<SingleObjectFeature> featureList = testfeatures.getListSOF();
	for (vector<SingleObjectFeature>::iterator iter = featureList.begin(); iter != featureList.end(); iter++ ) {

		int instanceID = (*iter).getInstanceID();
		if (instanceID == currentInstanceID)  {
			features = (*iter);
		}
	}
	return features;
}


ObjectPairFeature Test::findOPF(ArrangeFeatureTestScene & testfeatures, int currentInstanceID, int targetInstanceID) {

	ObjectPairFeature features;

	// if we are not considering the same object twice
	if (currentInstanceID != targetInstanceID) {
		vector<ObjectPairFeature> featureList = testfeatures.getListOPF();
		for (vector<ObjectPairFeature>::iterator iter = featureList.begin(); iter != featureList.end(); iter++ ) {

			int instanceID1 = (*iter).getInstanceID1();
			int instanceID2 = (*iter).getInstanceID2();

			if ( (instanceID1 == currentInstanceID ) && (instanceID2 == targetInstanceID))  {
				features = (*iter);
			}
		}
	}
	else {
		cout <<"Error in Test::findOPF: the same test object is considered. " << endl;
		exit(0);
	}
	return features;
}


void Test::loadTrainedGMMs(Training & trainedParams) {

	// SingleObject
	meanNormalizationSingleObject = trainedParams.getmeanNormalizationSingleObject();
	stdNormalizationSingleObject = trainedParams.getstdNormalizationSingleObject();
	minFeatSingleObject  = trainedParams.getminFeatSingleObject();
	maxFeatSingleObject = trainedParams.getmaxFeatSingleObject();

	// ObjectPair
	meanNormalizationObjectPair = trainedParams.getmeanNormalizationObjectPair();
	stdNormalizationObjectPair = trainedParams.getstdNormalizationObjectPair();
	minFeatObjectPair = trainedParams.getminFeatObjectPair();
	maxFeatObjectPair = trainedParams.getmaxFeatObjectPair();

	// SingleObject
	meansSingleObject = trainedParams.getmeansSingleObject();
	weightsSingleObject = trainedParams.getweightsSingleObject();
	covsSingleObject = trainedParams.getcovsSingleObject();

	// ObjectPair
	meansObjectPair = trainedParams.getmeansObjectPair();
	weightsObjectPair = trainedParams.getweightsObjectPair();
	covsObjectPair = trainedParams.getcovsObjectPair();

	thresholdsSingleObject = trainedParams.geThresholdsSingleObject();

}


void Test::loadLearnedObjectCategoryFrequency(vector<double> frequencySingleObjectin, vector<vector<double> > frequencyObjectPairin ) {

	frequencySingleObject = frequencySingleObjectin;
	frequencyObjectPair = frequencyObjectPairin;
}

void Test::printmeanNormalizationSingleObject() {

	for (int i = 0; i < meanNormalizationSingleObject.size(); i++) {
		cout << endl;

		for (int j =0 ; j < meanNormalizationSingleObject.at(i).size(); j++) {
			cout << meanNormalizationSingleObject.at(i).at(j) << " " ;
		}
	}
}


void Test::printmeanNormalizationObjectPair() {

	for (int i = 0; i < meanNormalizationObjectPair.size(); i++) {
		for (int j =0 ; j < meanNormalizationObjectPair.at(i).size(); j++) {
			for (int k = 0; k < meanNormalizationObjectPair.at(i).at(j).size(); k++) {
				cout << meanNormalizationObjectPair.at(i).at(j).at(k) << " " ;
			}
			cout << endl;
		}
	}
}


