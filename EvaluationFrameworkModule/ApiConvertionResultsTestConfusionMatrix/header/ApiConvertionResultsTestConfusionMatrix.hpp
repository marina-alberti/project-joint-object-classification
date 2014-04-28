/*
 * ApiConvertionResultsTestConfusionMatrix.hpp
 *
 *  Created on: Dec 19, 2013
 *      Author: marina
 */

#ifndef APICONVERTIONRESULTSTESTCONFUSIONMATRIX_HPP_
#define APICONVERTIONRESULTSTESTCONFUSIONMATRIX_HPP_



#include "Test.hpp"
#include "SceneInformation.hpp"
#include "Object.hpp"
#include "ConfusionMatrix.hpp"
#include <vector>


class ApiConvertionResultsTestConfusionMatrix {

private:

public:

	static void convertResultsToMatrix(path myPath, SceneInformation scene, ConfusionMatrix & cMatrix, vector<int> categoryList);

};


#endif /* APICONVERTIONRESULTSTESTCONFUSIONMATRIX_HPP_ */
