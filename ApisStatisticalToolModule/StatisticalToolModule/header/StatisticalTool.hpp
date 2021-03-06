/*
 * GMMLearning.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: marina
 */

#ifndef STATISTICALTOOL_HPP_
#define STATISTICALTOOL_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/ml/ml.hpp"
#include "utils.hpp"

using namespace std;



class StatisticalTool {

private:

public:

	static void trainGMM(vector<vector<float> >, int, cv::Mat &, cv::Mat &, vector<cv::Mat> &);

	static double computeGMMProbability(vector<float>feats, cv::Mat means, vector<cv::Mat> covs, cv::Mat weights );

	static vector<double> computeMean(cv::Mat &);
	static vector<double> computeStd(cv::Mat &, vector<double> );
	static cv::Mat doNormalization(cv::Mat &, vector<double> , vector<double>);
	static vector<double> computeMin(cv::Mat FeatMat);
	static vector<double> computeMax(cv::Mat FeatMat);
	static cv::Mat doNormalizationMinMax(cv::Mat & FeatMat, vector<double> maxvector, vector<double> minvector);

	static vector<double> computeMeanv(vector<vector<float> > &);
	static vector<double> computeStdv(vector<vector<float> > &, vector<double> );
	static vector<double> computeMinv(vector<vector<float> >  FeatMat);
	static vector<double> computeMaxv(vector<vector<float> >  FeatMat);
	static vector<vector<float> > doNormalizationv(vector<vector<float> > &, vector<double> , vector<double>);
	static vector<vector<float> > doNormalizationMinMaxv(vector<vector<float> >  & FeatMat, vector<double> maxvector, vector<double> minvector);

	static vector<float> doNormalizationFeatureVector(vector<float> & FeatMat, vector<double> meansVector, vector<double> stdVector);
	static vector<float> doNormalizationMinMaxFeatureVector(vector<float> & FeatMat, vector<double> maxvector, vector<double> minvector);


};


#endif /* STATISTICALTOOL_HPP_ */
