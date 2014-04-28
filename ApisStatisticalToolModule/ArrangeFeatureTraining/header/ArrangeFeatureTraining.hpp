/*
 * ArrangeFeatureTraining.hpp
 *
 *  Created on: Nov 23, 2013
 *      Author: marina
 *
 * This class rearranges the features extracted from 'individual objects'
 * and 'object pairs' in a set of scenes. The features extracted from a
 * training dataset are organized in a data structure that will be used
 * in the learning phase, to call functions of the statistical tool.
 *
 *  @input:
 * DatabaseSingleObjectFeature
 * DatabaseObjectPairFeature
 *
 *  @output:
 * FeatureMatrixSingleObject
 * FeatureMatrixObjectPair
 *
 */

#ifndef ARRANGEFEATURETRAINING_HPP_
#define ARRANGEFEATURETRAINING_HPP_


#include <string.h>
#include <boost/property_tree/ptree.hpp>
#include "Object.hpp"
#include "ApiConvertKTHDB.hpp"
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <cstring>
#include <dirent.h>
#include <cstdlib>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/opencv.hpp>
#include "opencv2/ml/ml.hpp"
#include <cmath>
#include "utils.hpp"
#include "ApiConvertSimulationDB.hpp"
#include "DatabaseSingleObjectFeature.hpp"
#include "DatabaseObjectPairFeature.hpp"

#define NOBJECTCLASSES 7


using namespace std;

class ArrangeFeatureTraining {

private:

public:

	  // Rearranges the features (SingleObject)
	  static void setFeatureMatrixSingleObject(DatabaseSingleObjectFeature &, vector<vector<vector<float> > > & FMSingleObject);

	  // Rearranges the features (ObjectPair)
	  static void setFeatureMatrixObjectPair(DatabaseObjectPairFeature &, vector<vector<vector<vector<float> > > > & FMPairObject);

	  static void printFeatureMatrixSingleObject(vector<vector<vector<float> > > & FMSingleObject);

	  static void printFeatureMatrixObjectPair(vector<vector<vector<vector<float> > > > & FMPairObject);

	  static void printFeatureSingleObjectToFile(vector<vector<vector<float> > > & FMSingleObject, string);

	  static void printFeatureObjectPairToFile(vector<vector<vector<vector<float> > > > & FMPairObject, string);

};

#endif /* ARRANGEFEATURETRAINING_HPP_ */
