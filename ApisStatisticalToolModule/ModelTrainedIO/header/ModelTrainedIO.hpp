/*
 * ModelTrainedIO.hpp
 *
 *  Created on: Nov 23, 2013
 *      Author: marina
 */

#ifndef MODELTRAINEDIO_HPP_
#define MODELTRAINEDIO_HPP_


#include <string.h>
#include <boost/property_tree/ptree.hpp>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <cstring>
#include <dirent.h>
#include <cstdlib>
#include "Training.hpp"
#include "Test.hpp"

class ModelTrainedIO {

private:


public:

	/*
	 * @description:
	 * This function stores the learned model parameters to file
	 * so that in the test phase the trained parameters can be loaded and used.
	 */
	static void storeTrainingToFile(Training &, string);

	/*
	 * @description:
	 * This function reads and loads the learned parameters, which are stored in
	 * a text file.
	 */
	static void loadTrainedGMMsFile(string folder, Test &);

	static void storeMeanNormalizationSingleObjectFile(Training &, string);
	static void storeStdNormalizationSingleObjectFile(Training &, string);
	static void storeMinFeatSingleObjectFile(Training &, string);
	static void storeMaxFeatSingleObjectFile(Training &, string);
	static void storeMeanNormalizationObjectPairFile(Training &, string);
	static void storeStdNormalizationObjectPairFile(Training &, string);
	static void storeMinFeatObjectPairFile(Training &, string);
	static void storeMaxFeatObjectPairFile(Training &, string);

	static void storeMeansSingleObjectFile(Training &, string);
	static void storeWeightsSingleObjectFile(Training &, string);
	static void storeCovsSingleObjectFile(Training &, string);
	static void storeMeansObjectPairFile(Training &, string);
	static void storeWeightsObjectPairFile(Training &, string);
	static void storeCovsObjectPairFile(Training &, string);

	static void storeThresholdSingleObject(Training &, string);

	static void storefrequenciesSingleObject(vector<double>, string);
	static void storefrequenciesObjectPair(vector<vector<double> >, string);

	static void storefrequencies(vector<double> so, vector<vector<double> > op, string folder) {
		storefrequenciesSingleObject(so, folder);
		storefrequenciesObjectPair(op, folder);
	}


	static void loadMeanNormalizationSingleObjectFile(string folder, Test &);
	static void loadStdNormalizationSingleObjectFile(string folder, Test &);
	static void loadMinFeatSingleObjectFile(string folder, Test &);
	static void loadMaxFeatSingleObjectFile(string folder, Test &);
	static void loadMeanNormalizationObjectPairFile(string folder, Test &);
	static void loadStdNormalizationObjectPairFile(string folder, Test &);
	static void loadMinFeatObjectPairFile(string folder, Test &);
	static void loadMaxFeatObjectPairFile(string folder, Test &);

	static void loadMeansSingleObjectFile(string folder, Test &);
	static void loadWeightsSingleObjectFile(string folder, Test &);
	static void loadCovsSingleObjectFile(string folder, Test &);
	static void loadMeansObjectPairFile(string folder, Test &);
	static void loadWeightsObjectPairFile(string folder, Test &);
	static void loadCovsObjectPairFile(string folder, Test &);

	static void loadThresholdSingleObject(string folder, Test &);

	static void loadfrequenciesSingleObject(string folder, Test &);
	static void loadfrequenciesObjectPair(string folder, Test &);

	static void loadfrequencies(string folder, Test & test) {
		loadfrequenciesSingleObject(folder, test);
		loadfrequenciesObjectPair(folder, test);
	}

};



#endif /* MODELTRAINEDIO_HPP_ */
